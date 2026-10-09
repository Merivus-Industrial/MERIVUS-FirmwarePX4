#!/usr/bin/env python3
"""Run one isolated Gazebo Classic hover trial with a fixed mode and airframe."""

import argparse
import importlib.util
import json
import math
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import time
import xml.etree.ElementTree as ET


def hover_reached(position, takeoff_z):
    return (position is not None and all(math.isfinite(value) for value in
            (takeoff_z, position.z, position.vz))
            and takeoff_z - position.z >= 1.0 and abs(position.vz) <= 0.3)


def configure_wind_world(source, scenario, seed, case=None):
    if scenario not in ("wind", "gust"):
        raise ValueError(f"unsupported wind scenario: {scenario}")
    tree = ET.parse(source)
    plugin = tree.find(".//plugin[@name='wind_plugin']")
    if plugin is None:
        raise ValueError("Gazebo wind plugin not found")
    case = case or {}
    direction = case.get("direction", "1 0 0" if seed % 2 else "0 1 0")
    magnitude = case.get("magnitude_m_s", 8 if scenario == "gust" else 4.0)
    settings = ({"windDirectionMean": direction} if scenario == "wind" else {
        "windVelocityMean": "0", "windGustStart": "46",
        "windGustDuration": str(case.get("duration_s", 5)),
        "windGustVelocityMean": str(magnitude), "windGustDirectionMean": direction,
    })
    if scenario == "wind":
        settings["windVelocityMean"] = str(magnitude)
    for name, value in settings.items():
        field = plugin.find(name)
        if field is None:
            raise ValueError(f"Gazebo wind setting missing: {name}")
        field.text = value
    return tree


class Trial:
    def __init__(self, args):
        self.args = args
        self.case = json.loads(args.case.read_text(encoding="utf-8")) if args.case else {}
        self.repo = args.repo.resolve()
        self.output = args.output.resolve()
        self.output.mkdir(parents=True, exist_ok=False)
        self.rootfs = self.output / "rootfs"
        self.rootfs.mkdir()
        self.build = self.repo / "build/px4_sitl_default"
        if not (self.build / "bin/px4").is_file():
            raise FileNotFoundError("build/px4_sitl_default/bin/px4")
        module = importlib.util.spec_from_file_location("afcr_mavlink", args.dialect)
        dialect = importlib.util.module_from_spec(module)
        module.loader.exec_module(dialect)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("127.0.0.1", 14650))
        self.socket.setblocking(False)
        self.reader = dialect.MAVLink(None)
        self.writer = dialect.MAVLink(self, srcSystem=255, srcComponent=190)
        self.latest = {}
        self.sim_time = 0.0
        self.last_manual = 0.0
        self.manual_x = 0
        self.manual_enabled = True
        self.last_heartbeat = 0.0
        self.processes = []
        self.streams = []
        self.env = dict(os.environ)
        self.env.update(PX4_SIM_MODEL="gazebo-classic_iris", MERIVUS_AFCR_MODE=args.mode,
                        PX4_SIM_SPEED_FACTOR="1", PX4_SYS_AUTOSTART="10015",
                        GAZEBO_MASTER_URI="http://127.0.0.1:11459", GAZEBO_MODEL_DATABASE_URI="")
        gazebo = self.repo / "Tools/simulation/gazebo-classic/sitl_gazebo-classic"
        self.env["GAZEBO_PLUGIN_PATH"] = str(self.build / "build_gazebo-classic")
        self.env["GAZEBO_MODEL_PATH"] = str(gazebo / "models")
        self.env["LD_LIBRARY_PATH"] = self.env.get("LD_LIBRARY_PATH", "") + ":" + self.env["GAZEBO_PLUGIN_PATH"]
        config = self.output / "config"
        config.mkdir()
        self.env["PATH"] = str(config) + ":" + self.env["PATH"]
        config.joinpath("px4-rc.mavlink").write_text(
            "mavlink start -x -u 18670 -o 14650 -r 4000000 -f\n"
            "mavlink stream -r 20 -s LOCAL_POSITION_NED -u 18670\n", encoding="utf-8")
        params = self.case.get("px4_params", {})
        if any(not key.startswith("EKF2_") or not isinstance(value, (int, float))
               for key, value in params.items()):
            raise ValueError("case permits only numeric EKF2 parameters")
        config.joinpath("px4-rc.params").write_text(
            "param set COM_RC_IN_MODE 1\nparam set SDLOG_MODE 2\nparam set SDLOG_PROFILE 163\n"
            + "".join(f"param set {key} {value}\n" for key, value in sorted(params.items())), encoding="utf-8")
        disturbance = self.case.get("disturbance", args.scenario)
        world = gazebo / "worlds" / ("windy.world" if disturbance in ("wind", "gust", "periodic", "randomized_gust") else "empty.world")
        if disturbance in ("wind", "gust"):
            configured_world = self.output / "world.world"
            configure_wind_world(world, disturbance, args.seed, self.case).write(
                configured_world, encoding="unicode", xml_declaration=True)
            world = configured_world
        elif disturbance in ("periodic", "randomized_gust"):
            tree = ET.parse(world)
            existing = tree.find(".//plugin[@name='wind_plugin']")
            if existing is None:
                raise ValueError("Gazebo wind plugin not found")
            tree.find(".//world").remove(existing)
            plugin = ET.SubElement(tree.find(".//world"), "plugin",
                                   name="afcr_wind_profile", filename="libafcr_wind_profile.so")
            for key, value in {"kind": disturbance, "magnitude": self.case["magnitude_m_s"],
                               "direction": self.case["direction"], "start": 46,
                               "duration": self.case.get("duration_s", 5), "period": 3,
                               "seed": args.seed}.items():
                ET.SubElement(plugin, key).text = str(value)
            world = self.output / "world.world"
            tree.write(world, encoding="unicode", xml_declaration=True)
        model = gazebo / "models/iris/iris.sdf"
        model_case = self.case.get("airframe", {})
        sensor_case = self.case.get("sensor", {})
        if args.scenario == "payload" or model_case or sensor_case:
            tree = ET.parse(model)
            inertial = tree.find(".//model/link[@name='base_link']/inertial")
            if inertial is None or inertial.find("mass") is None:
                raise ValueError("iris base_link inertia not found")
            mass_scale = model_case.get("mass_scale", 1.2 if args.scenario == "payload" else 1)
            inertia_scale = model_case.get("inertia_scale", 1.2 if args.scenario == "payload" else 1)
            inertial.find("mass").text = str(float(inertial.findtext("mass")) * mass_scale)
            for axis in ("ixx", "iyy", "izz"):
                field = inertial.find("inertia/" + axis)
                if field is not None:
                    field.text = str(float(field.text) * inertia_scale)
            for plugin in tree.findall(".//model/plugin"):
                if plugin.get("filename") == "libgazebo_motor_model.so":
                    for name in ("timeConstantUp", "timeConstantDown"):
                        field = plugin.find(name)
                        field.text = str(float(field.text) * model_case.get("motor_delay_scale", 1))
                    field = plugin.find("maxRotVelocity")
                    field.text = str(float(field.text) * model_case.get("max_rot_velocity_scale", 1))
                elif plugin.get("filename") == "libgazebo_imu_plugin.so":
                    for name in ("gyroscopeNoiseDensity", "accelerometerNoiseDensity"):
                        field = plugin.find(name)
                        field.text = str(float(field.text) * sensor_case.get("imu_noise_scale", 1))
                    for name in ("gyroscopeTurnOnBiasSigma", "accelerometerTurnOnBiasSigma"):
                        field = plugin.find(name)
                        field.text = str(float(field.text) * sensor_case.get("imu_bias_scale", 1))
            model = self.output / "iris_case.sdf"
            tree.write(model, encoding="unicode", xml_declaration=True)
        if "gps_position_noise_scale" in sensor_case or "gps_velocity_noise_scale" in sensor_case:
            overlay = self.output / "models/gps"
            overlay.mkdir(parents=True)
            source = gazebo / "models/gps"
            shutil.copy2(source / "model.config", overlay / "model.config")
            tree = ET.parse(source / "gps.sdf")
            plugin = tree.find(".//plugin[@name='gps_plugin']")
            for name, scale in (("gpsXYNoiseDensity", sensor_case.get("gps_position_noise_scale", 1)),
                                ("gpsVXYNoiseDensity", sensor_case.get("gps_velocity_noise_scale", 1))):
                field = plugin.find(name)
                field.text = str(float(field.text) * scale)
            tree.write(overlay / "gps.sdf", encoding="unicode", xml_declaration=True)
            self.env["GAZEBO_MODEL_PATH"] = str(self.output / "models") + ":" + self.env["GAZEBO_MODEL_PATH"]
        self.world = world
        self.model = model
        (self.output / "trial.json").write_text(json.dumps({
            "mode": args.mode, "scenario": args.scenario, "seed": args.seed, "case": self.case,
            "git_head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=self.repo, text=True).strip(),
            "model": str(model), "world": str(world)
        }, indent=2) + "\n", encoding="utf-8")

    def write(self, data):
        self.socket.sendto(data, ("127.0.0.1", 18670))

    def spawn(self, command, name):
        stream = (self.output / (name + ".log")).open("w", encoding="utf-8")
        self.streams.append(stream)
        process = subprocess.Popen(command, cwd=self.rootfs, env=self.env, stdout=stream,
                                   stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True)
        self.processes.append(process)

    def cli(self, module, *args):
        return subprocess.run([str(self.build / "bin" / ("px4-" + module)), *map(str, args)],
                              cwd=self.rootfs, env=self.env, capture_output=True, text=True, timeout=10)

    def pump(self):
        now = time.monotonic()
        if now - self.last_heartbeat > 0.4:
            self.writer.heartbeat_send(6, 8, 0, 0, 4)
            self.last_heartbeat = now
        if self.manual_enabled and now - self.last_manual > 0.02:
            self.writer.manual_control_send(1, self.manual_x, 0, 500, 0, 0)
            self.last_manual = now
        while True:
            try:
                packet, _ = self.socket.recvfrom(65535)
            except BlockingIOError:
                break
            for message in self.reader.parse_buffer(packet) or []:
                if message.get_type() != "BAD_DATA":
                    self.latest[message.get_type()] = message
                    if message.get_type() == "LOCAL_POSITION_NED":
                        self.sim_time = message.time_boot_ms / 1000.0
        time.sleep(0.005)

    def wait_for(self, seconds):
        start = self.sim_time
        deadline = time.monotonic() + max(seconds * 3, 60)
        while self.sim_time - start < seconds:
            if time.monotonic() > deadline:
                raise TimeoutError("SITL time did not advance")
            self.pump()

    def run(self):
        self.spawn(["gzserver", "--verbose", "--seed", str(self.args.seed), str(self.world)], "gazebo")
        time.sleep(3)
        result = subprocess.run(["gz", "model", "--spawn-file=" + str(self.model),
                                 "--model-name=iris", "-x", "0", "-y", "0", "-z", ".5"],
                                env=self.env, capture_output=True, timeout=30)
        (self.output / "spawn.log").write_bytes(result.stdout + result.stderr)
        if result.returncode:
            raise RuntimeError("Gazebo model spawn failed")
        self.spawn([str(self.build / "bin/px4"), "-d", "-w", str(self.rootfs), str(self.build / "etc")], "px4")
        deadline = time.monotonic() + 60
        while "LOCAL_POSITION_NED" not in self.latest:
            if time.monotonic() > deadline:
                raise TimeoutError("PX4 local position unavailable")
            self.pump()
        self.wait_for(12)
        takeoff_z = self.latest["LOCAL_POSITION_NED"].z
        self.writer.command_long_send(1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0)
        self.wait_for(2)
        if self.cli("commander", "takeoff").returncode:
            raise RuntimeError("PX4 takeoff command failed")
        self.wait_for(20)
        position = self.latest.get("LOCAL_POSITION_NED")
        if not hover_reached(position, takeoff_z):
            raise RuntimeError("takeoff did not establish a stable hover at least 1 m above its start")
        start = self.sim_time
        self.wait_for(35 if self.case.get("disturbance") == "periodic" else 25)
        end = self.sim_time
        window = {"name": "normal_hover" if self.args.scenario == "normal" else self.args.scenario + "_hover",
                  "start_s": start + 5, "end_s": end - 5}
        disturbance = self.case.get("disturbance", self.args.scenario)
        if disturbance in ("gust", "periodic", "randomized_gust"):
            event_end = 58 if disturbance == "periodic" else 46 + self.case.get("duration_s", 5)
            if window["start_s"] + 2 > 46 or window["end_s"] < event_end:
                raise RuntimeError("gust event outside the scored hover window")
            window.update(event_start_s=46, event_end_s=event_end)
        (self.output / "windows.json").write_text(json.dumps([window], indent=2) + "\n", encoding="utf-8")
        self.cli("commander", "land")
        self.wait_for(20)
        self.cli("logger", "stop")
        logs = sorted((self.rootfs / "log").rglob("*.ulg"))
        if len(logs) != 1:
            raise RuntimeError(f"expected one ULog, got {len(logs)}")
        (self.output / "ulog_path.txt").write_text(str(logs[0]) + "\n", encoding="utf-8")

    def close(self):
        if self.processes:
            try:
                self.cli("commander", "land")
                self.cli("shutdown")
            except (OSError, subprocess.TimeoutExpired):
                pass
        for process in reversed(self.processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        for stream in self.streams:
            stream.close()
        self.socket.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--dialect", type=Path, required=True)
    parser.add_argument("--mode", choices=("off", "shadow", "active"), required=True)
    parser.add_argument("--scenario", choices=("normal", "wind", "gust", "payload"), required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--case", type=Path)
    args = parser.parse_args()
    trial = Trial(args)
    try:
        trial.run()
    finally:
        trial.close()


if __name__ == "__main__":
    main()
