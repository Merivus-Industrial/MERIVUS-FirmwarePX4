#!/usr/bin/env python3
"""Isolated Gazebo Classic sessions. Refuse to overwrite evidence or contact hardware."""
import argparse
import importlib.util
import hashlib
import ipaddress
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import time


class Session:
    def __init__(self, args):
        self.args = args
        self.repo = Path(args.repo).resolve()
        self.build = self.repo / 'build/px4_sitl_default'
        self.out = Path(args.output).resolve()
        self.out.mkdir(parents=True, exist_ok=False)
        self.root = self.out / 'rootfs'
        self.root.mkdir()
        self.latest = {}
        self.processes = []
        self.streams = []
        self.sim = 0.0
        self.last_heartbeat = 0.0
        self.last_manual = 0.0
        self.last_report = 0.0
        self.wall_start = time.monotonic()
        self.wire = self.out.joinpath('mavlink.bin').open('wb')
        self.messages = self.out.joinpath('messages.jsonl').open('w')
        self.events = self.out.joinpath('events.jsonl').open('w')
        spec = importlib.util.spec_from_file_location('merivus_validation', args.dialect)
        self.dialect = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.dialect)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('127.0.0.1', 14650))
        self.socket.setblocking(False)
        self.parser = self.dialect.MAVLink(None)
        self.sender = self.dialect.MAVLink(self, srcSystem=255, srcComponent=190)
        self.env = dict(os.environ)
        self.env.update(PX4_SIM_MODEL='gazebo-classic_iris', PX4_SIM_SPEED_FACTOR=str(args.speed),
                        PX4_SYS_AUTOSTART='10015', GAZEBO_MASTER_URI='http://127.0.0.1:11459',
                        GAZEBO_MODEL_DATABASE_URI='')
        gazebo = self.repo / 'Tools/simulation/gazebo-classic/sitl_gazebo-classic'
        self.gazebo = gazebo
        self.env['GAZEBO_PLUGIN_PATH'] = str(self.build / 'build_gazebo-classic')
        self.env['GAZEBO_MODEL_PATH'] = str(gazebo / 'models')
        self.env['LD_LIBRARY_PATH'] = self.env.get('LD_LIBRARY_PATH', '') + ':' + self.env['GAZEBO_PLUGIN_PATH']
        config = self.out / 'config'
        config.mkdir()
        self.env['PATH'] = str(config) + ':' + self.env['PATH']
        params = {'FTC_MON_EN': 1, 'FTC_CA_SHADOW': 1, 'FTC_CA_EN': 0, 'FTC_REC_EN': 1,
                  'FTC_REC_ACT': 0, 'FTC_SIM_EN': 0, 'FTC_IMPACT_EN': 1, 'FTC_LOC_EN': 1,
                  'FTC_SIM_MOT': args.motor, 'FTC_SIM_EFF': 1, 'FTC_SIM_RAMP': 0, 'FTC_SIM_INT': 0,
                  'COM_RC_IN_MODE': 1, 'MIS_TAKEOFF_ALT': 10, 'SDLOG_MODE': 2}
        config.joinpath('px4-rc.mavlink').write_text('mavlink start -x -u 18670 -o 14650 -r 4000000 -f\n' + ''.join('mavlink stream -r 50 -s {} -u 18670\n'.format(name) for name in ('LOCAL_POSITION_NED', 'GLOBAL_POSITION_INT', 'ATTITUDE', 'ATTITUDE_TARGET')))
        if args.groundstation:
            destination = ipaddress.ip_address(args.groundstation)
            if not destination.is_private or destination.version != 4:
                raise ValueError('GroundStation validation requires a private IPv4 address')
            with config.joinpath('px4-rc.mavlink').open('a') as stream:
                stream.write('mavlink start -x -u 18671 -o {} -t {} -r 4000000 -f\n'.format(args.groundstation_port, destination))
        config.joinpath('px4-rc.params').write_text(''.join('param set {} {}\n'.format(k, v) for k, v in params.items()))
        self.out.joinpath('scenario.json').write_text(json.dumps(dict(vars(args), runner_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()), indent=2))
        self.out.joinpath('head.txt').write_text(subprocess.check_output(['git', '-C', str(self.repo), 'rev-parse', 'HEAD']).decode())

    def write(self, data):
        self.socket.sendto(data, ('127.0.0.1', 18670))

    def event(self, name, **details):
        value = dict(event=name, sim_time=self.sim, wall_time=time.monotonic()-self.wall_start, **details)
        self.events.write(json.dumps(value, allow_nan=True) + '\n')
        self.events.flush()
        print(json.dumps(value, allow_nan=True), flush=True)

    def spawn(self, command, name):
        stream = self.out.joinpath(name+'.log').open('w')
        self.streams.append(stream)
        process = subprocess.Popen(command, cwd=str(self.root), env=self.env, stdout=stream,
                                   stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True)
        self.processes.append(process)
        return process

    def cli(self, module, *arguments):
        result = subprocess.run([str(self.build/'bin'/('px4-'+module)), *map(str, arguments)],
                                cwd=str(self.root), env=self.env, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, timeout=10)
        text = result.stdout.decode(errors='replace')
        with self.out.joinpath('cli.log').open('a') as stream:
            stream.write('$ '+module+' '+' '.join(map(str, arguments))+'\n'+text+'\n')
        if result.returncode and module != 'shutdown':
            self.event('cli_failure', module=module, arguments=arguments, output=text)
        return text

    def pump(self):
        now = time.monotonic()
        if now - self.last_heartbeat > 0.4:
            self.sender.heartbeat_send(6, 8, 0, 0, 4)
            self.last_heartbeat = now
        while True:
            try:
                data, _ = self.socket.recvfrom(65535)
            except BlockingIOError:
                break
            self.wire.write(data)
            for msg in self.parser.parse_buffer(data) or []:
                kind = msg.get_type()
                if kind == 'BAD_DATA':
                    continue
                self.latest[kind] = msg
                value = msg.to_dict()
                value['received_wall'] = now - self.wall_start
                value['wire_length'] = len(msg.get_msgbuf())
                value['sequence'] = msg.get_seq()
                self.messages.write(json.dumps(value, allow_nan=True)+'\n')
                if kind == 'LOCAL_POSITION_NED':
                    self.sim = msg.time_boot_ms * .001
                if kind == 'STATUSTEXT':
                    self.event('statustext', severity=msg.severity, text=msg.text)
        if now - self.last_report > 20:
            health = self.latest.get('MERIVUS_FTC_MOTOR_STATUS')
            pos = self.latest.get('LOCAL_POSITION_NED')
            self.event('progress', altitude=-pos.z if pos else None, motor=health.to_dict() if health else None)
            self.last_report = now
        time.sleep(.005)

    def manual(self, x=0, y=0, z=500, yaw=0):
        now = time.monotonic()
        if now-self.last_manual > .01:
            self.sender.manual_control_send(1, int(x), int(y), int(z), int(yaw), 0)
            self.last_manual = now

    def run_for(self, duration, maneuver=False):
        start = self.sim
        wall = time.monotonic()
        while self.sim-start < duration:
            if time.monotonic()-wall > max(duration*3, 60):
                raise RuntimeError('Simulation failed to advance within wall-clock deadline')
            elapsed = self.sim-start
            if maneuver:
                amplitude = self.args.amplitude
                frequencies = (3.2, 4.3, 2.1, 1.7) if self.args.profile == 'angular' else (1.3, 1.9, .5, 1.1)
                self.manual(amplitude*math.sin(elapsed*frequencies[0]), amplitude*math.sin(elapsed*frequencies[1]),
                            500+100*math.sin(elapsed*frequencies[2]), amplitude*math.sin(elapsed*frequencies[3]))
            else:
                self.manual()
            self.pump()

    def command(self, command, params):
        self.sender.command_long_send(1, 1, command, 0, *(list(params)+[0]*7)[:7])

    def gate(self):
        motor = self.latest.get('MERIVUS_FTC_MOTOR_STATUS')
        control = self.latest.get('MERIVUS_FTC_CONTROL_STATUS')
        if not motor or not control:
            return False
        valid = bool(motor.flags & self.dialect.MERIVUS_FTC_MOTOR_FLAGS_MODEL_VALID)
        valid &= bool(control.flags & self.dialect.MERIVUS_FTC_CONTROL_FLAGS_AUTHORITY_VALID)
        valid &= bool(motor.baseline_learned) and bool(motor.current_observable)
        valid &= motor.estimate_age < 2 and all(math.isfinite(v) and v <= .12 for v in motor.estimate_uncertainty[:4])
        self.event('baseline_gate', passed=bool(valid), motor=motor.to_dict(), control=control.to_dict())
        return bool(valid)

    def run(self):
        for port in (4560, 18670):
            probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM if port == 4560 else socket.SOCK_DGRAM)
            try:
                probe.bind(('127.0.0.1', port))
            finally:
                probe.close()
        self.spawn(['gzserver', '--verbose', str(self.gazebo/'worlds/empty.world')], 'gazebo')
        time.sleep(3)
        result = subprocess.run(['gz', 'model', '--spawn-file='+str(self.gazebo/'models/iris/iris.sdf'),
                                 '--model-name=iris', '-x', '0', '-y', '0', '-z', '.5'],
                                env=self.env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=30)
        self.out.joinpath('spawn.log').write_bytes(result.stdout)
        if result.returncode:
            raise RuntimeError('Gazebo model spawn failed')
        self.spawn([str(self.build/'bin/px4'), '-d', '-w', str(self.root), str(self.build/'etc')], 'px4')
        ready = time.monotonic()
        while 'LOCAL_POSITION_NED' not in self.latest:
            self.manual()
            self.pump()
            if time.monotonic()-ready > 60:
                raise RuntimeError('No PX4 local-position telemetry')
        self.event('ground')
        self.run_for(12)
        if self.args.groundstation:
            self.event('groundstation_live')
            self.run_for(60)
            streams = ('MERIVUS_FTC_MOTOR_STATUS', 'MERIVUS_FTC_CONTROL_STATUS',
                       'MERIVUS_FTC_EXTREME_STATUS', 'MERIVUS_FTC_DIAGNOSTICS')
            for name in streams:
                self.cli('mavlink', 'stream', '-u', 18671, '-s', name, '-r', 0)
            self.event('groundstation_ftc_paused_heartbeat_continues')
            self.run_for(60)
            for name, rate in zip(streams, (5, 5, 10, 1)):
                self.cli('mavlink', 'stream', '-u', 18671, '-s', name, '-r', rate)
            self.event('groundstation_ftc_resumed')
            self.run_for(40)
        self.cli('param', 'show')
        self.command(400, [1])
        self.run_for(2)
        self.cli('commander', 'takeoff')
        self.run_for(18)
        pos = self.latest.get('LOCAL_POSITION_NED')
        if not pos or -pos.z < 5:
            raise RuntimeError('Takeoff did not reach five meters')
        self.event('loiter')
        self.run_for(10)
        self.command(176, [1, 2 if self.args.profile == 'angular' else 3, 0])
        self.event('baseline_maneuver')
        self.run_for(self.args.baseline, maneuver=True)
        self.gate_passed = self.gate()
        for topic in ('ftc_model_status', 'motor_health_status', 'ftc_control_authority', 'ftc_recovery_status'):
            self.cli('listener', topic, 1)
        if self.args.effectiveness < 1:
            if not self.gate_passed:
                raise RuntimeError('Fault injection blocked: baseline gate failed')
            self.cli('param', 'set', 'FTC_SIM_EFF', self.args.effectiveness)
            self.cli('param', 'set', 'FTC_SIM_EN', 1)
            self.event('injection', command_scale=self.args.effectiveness)
            self.run_for(self.args.duration, maneuver=True)
            self.cli('param', 'set', 'FTC_SIM_EN', 0)
            self.event('fault_removed')
            self.run_for(20, maneuver=True)
        else:
            self.command(176, [1, 3, 0])
            self.run_for(20)
        self.event('landing')
        self.cli('commander', 'land')
        self.run_for(30)
        self.cli('commander', 'disarm')
        self.run_for(3)
        self.cli('mavlink', 'status')
        self.cli('uorb', 'top', '-1')
        self.cli('logger', 'status')
        for module in ('motor_health_monitor', 'ftc_control_monitor', 'ftc_extreme_state_monitor', 'ftc_recovery', 'ftc_supervisor'):
            self.cli(module, 'status')
        self.event('finished', baseline_gate=self.gate_passed)

    def close(self):
        if self.processes:
            try:
                self.cli('param', 'set', 'FTC_SIM_EN', 0)
                self.cli('commander', 'land')
                self.cli('logger', 'stop')
                self.cli('shutdown')
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
        for stream in [self.wire, self.messages, self.events, *self.streams]:
            stream.close()
        self.socket.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--repo', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--dialect', required=True)
    parser.add_argument('--effectiveness', type=float, default=1)
    parser.add_argument('--motor', type=int, choices=range(1, 5), default=1)
    parser.add_argument('--baseline', type=float, default=80)
    parser.add_argument('--duration', type=float, default=45)
    parser.add_argument('--profile', choices=('normal','angular'), default='normal')
    parser.add_argument('--amplitude', type=float, default=350)
    parser.add_argument('--speed', type=float, default=2)
    parser.add_argument('--groundstation', help='Private IPv4 destination for the separate live UI/stale stream')
    parser.add_argument('--groundstation-port', type=int, default=14550)
    args = parser.parse_args()
    session = Session(args)
    try:
        session.run()
    except Exception as error:
        session.event('failed', reason=str(error))
        raise
    finally:
        session.close()
