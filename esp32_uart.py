#!/usr/bin/env python3
"""USB UART client for the existing ESP32 firmware (115200, 8N1)."""
import argparse
import glob
import json
import sys
import time

import serial


def find_port():
    ports = sorted(glob.glob('/dev/serial/by-id/*'))
    if not ports:
        ports = sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))
    if len(ports) != 1:
        raise ValueError('Specify --port; candidates: ' + ', '.join(ports))
    return ports[0]


def encode_command(command):
    if not isinstance(command, dict):
        raise ValueError('Command must be a JSON object')
    data = json.dumps(command, ensure_ascii=True, separators=(',', ':'), allow_nan=False).encode('ascii')
    if len(data) > 255:
        raise ValueError('Current firmware accepts at most 255 bytes per command')
    return data + b'\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', help='Prefer /dev/serial/by-id/...')
    parser.add_argument('--timeout', type=float, default=15)
    sub = parser.add_subparsers(dest='action', required=True)
    sub.add_parser('probe', help='Check echo and tickvi_status')
    send = sub.add_parser('send', help='Send one existing JSON command')
    send.add_argument('json')
    send.add_argument('--expect', default='OK:', help='Expected response prefix')
    sub.add_parser('monitor', help='Read messages until Ctrl+C')
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error('--timeout must be positive')

    commands = []
    if args.action == 'probe':
        commands = [({'echo': True}, 'OK: echo'),
                    ({'device': 'light_controller', 'tickvi_status': True}, 'OK: tickvi_OK')]
    elif args.action == 'send':
        commands = [(json.loads(args.json), args.expect)]
    encoded = [(encode_command(cmd), expected) for cmd, expected in commands]

    # Set modem lines before opening: avoid requesting an ESP32 reset.
    # Some USB drivers still pulse lines on open; probe retries cover startup.
    with serial.Serial(port=None, baudrate=115200, bytesize=8,
                       parity='N', stopbits=1, timeout=0.2,
                       write_timeout=2, exclusive=True) as uart:
        uart.dtr = False
        uart.rts = False
        uart.port = args.port or find_port()
        uart.open()
        print('Port: ' + uart.port, file=sys.stderr)
        pending = bytearray()

        def lines():
            pending.extend(uart.read(uart.in_waiting or 1))
            while b'\n' in pending:
                line, _, rest = pending.partition(b'\n')
                pending[:] = rest
                yield line.rstrip(b'\r').decode('utf-8', errors='replace')
            if len(pending) > 65536:
                raise ValueError('UART line exceeds 64 KiB')

        if args.action == 'monitor':
            while True:
                for line in lines():
                    print(line, flush=True)

        for payload, expected in encoded:
            deadline = time.monotonic() + args.timeout
            next_send = 0
            while time.monotonic() < deadline:
                now = time.monotonic()
                if now >= next_send:
                    uart.write(payload)
                    uart.flush()
                    # Only probe operations are safe to repeat automatically.
                    next_send = now + 2 if args.action == 'probe' else float('inf')
                matched = False
                for line in lines():
                    print(line, flush=True)
                    if line.startswith('ERROR:') or line.startswith('OK: error:'):
                        raise ValueError('ESP32 rejected the command: ' + line)
                    if line.startswith(expected):
                        matched = True
                if matched:
                    break
            else:
                raise TimeoutError('No response matching ' + repr(expected))
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
    except (ValueError, OSError, serial.SerialException, TimeoutError) as exc:
        print(str(exc), file=sys.stderr)
        sys.exit(1)
