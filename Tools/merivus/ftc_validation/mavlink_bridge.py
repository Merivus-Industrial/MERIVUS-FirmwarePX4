#!/usr/bin/env python3
"""Frame a localhost UDP test link over SSH without changing host firewall rules."""
import argparse
import json
import select
import socket
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path


def read_frame(stream):
    prefix = stream.read(2)
    if not prefix:
        return None
    if len(prefix) != 2:
        raise EOFError('Truncated bridge frame')
    size = struct.unpack('!H', prefix)[0]
    data = stream.read(size)
    if len(data) != size:
        raise EOFError('Truncated bridge payload')
    return data


def remote():
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.bind(('127.0.0.1', 14653))
    while True:
        ready, _, _ = select.select([udp, sys.stdin.buffer], [], [])
        if udp in ready:
            data, _ = udp.recvfrom(65535)
            sys.stdout.buffer.write(struct.pack('!H', len(data))+data)
            sys.stdout.buffer.flush()
        if sys.stdin.buffer in ready:
            data = read_frame(sys.stdin.buffer)
            if data is None:
                break
            udp.sendto(data, ('127.0.0.1', 18671))


def local(args):
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=False)
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.bind(('127.0.0.1', 14654))
    udp.settimeout(.25)
    process = subprocess.Popen(['ssh', args.host, 'python3', args.remote_script, '--remote'],
                               stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    counts = {'to_groundstation': 0, 'to_sitl': 0, 'bytes_to_groundstation': 0, 'bytes_to_sitl': 0}
    stop = threading.Event()

    def receive():
        try:
            with out.joinpath('mavlink.bin').open('xb') as wire:
                while not stop.is_set():
                    data = read_frame(process.stdout)
                    if data is None:
                        break
                    counts['to_groundstation'] += 1
                    counts['bytes_to_groundstation'] += len(data)
                    wire.write(data)
                    udp.sendto(data, ('127.0.0.1', 14550))
                    udp.sendto(data, ('127.0.0.1', 14655))
        finally:
            stop.set()

    worker = threading.Thread(target=receive, daemon=True)
    worker.start()
    deadline = time.monotonic()+args.seconds
    try:
        while time.monotonic() < deadline and not stop.is_set():
            try:
                data, address = udp.recvfrom(65535)
            except socket.timeout:
                continue
            if address != ('127.0.0.1', 14550):
                continue
            process.stdin.write(struct.pack('!H', len(data))+data)
            process.stdin.flush()
            counts['to_sitl'] += 1
            counts['bytes_to_sitl'] += len(data)
    finally:
        stop.set()
        process.stdin.close()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.terminate()
            process.wait(timeout=5)
        worker.join(timeout=5)
        udp.close()
        out.joinpath('result.json').write_text(json.dumps(counts, indent=2))
        out.joinpath('ssh.log').write_bytes(process.stderr.read())
        print(json.dumps(counts))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--remote', action='store_true')
    parser.add_argument('--host', default='px4vm')
    parser.add_argument('--remote-script')
    parser.add_argument('--output')
    parser.add_argument('--seconds', type=float, default=300)
    args = parser.parse_args()
    remote() if args.remote else local(args)
