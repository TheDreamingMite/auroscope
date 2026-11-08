#!/usr/bin/env python3
"""Receive a protocol-v2 ECG session, CRC-protected quality.json and ecg.wav."""
import argparse
import base64
import io
import json
import os
from pathlib import Path
import tempfile
import time
import uuid
import wave
import zlib

import serial
from esp32_uart import find_port


def crc(data):
    return f'{zlib.crc32(data) & 0xffffffff:08x}'


def require(condition, message):
    if not condition:
        raise ValueError(message)


def quality_flags(quality):
    count = quality['samples']
    if quality.get('sample_flags_encoding') == 'base64-u8':
        flags = base64.b64decode(quality['sample_flags_base64'], validate=True)
        require(len(flags) == count and all(value <= 15 for value in flags), 'Некорректная маска качества')
        return flags
    flags = bytearray(count)
    end = 0
    names = {'lead_off': 1, 'settling': 2, 'adc_saturation': 4, 'sampling_gap': 8}
    for item in quality['invalid_ranges']:
        start, size = item['start_sample'], item['count']
        require(type(start) is int and type(size) is int and start >= end and size > 0
                and start + size <= count, 'Некорректный диапазон качества')
        require(item['reasons'] and set(item['reasons']) <= names.keys(), 'Неизвестная причина качества')
        mask = 0
        for reason in item['reasons']:
            mask |= names[reason]
        flags[start:start + size] = bytes([mask]) * size
        end = start + size
    return bytes(flags)

def validate_wav(data, quality):
    count = quality['samples']
    require(type(count) is int and count > 0, 'Некорректное число отсчётов')
    with wave.open(io.BytesIO(data), 'rb') as wav:
        require((wav.getnchannels(), wav.getsampwidth(), wav.getframerate(), wav.getnframes(), wav.getcomptype())
                == (1, 2, 250, count, 'NONE'), 'Неверный формат WAV')
        require(len(wav.readframes(count)) == count * 2, 'Обрезанные данные WAV')
    require(len(data) == 44 + count * 2, 'Неверная длина файла WAV')
    flags = quality_flags(quality)
    invalid = sum(bool(mask) for mask in flags)
    require(all(not mask or data[44+i*2:46+i*2] == b'\0\0' for i, mask in enumerate(flags)),
            'Плохой участок не заполнен нулями')
    require(invalid == quality['invalid_samples'] and invalid + quality['valid_samples'] == count,
            'Сумма отсчётов в карте качества неверна')

class Link:
    def __init__(self, uart):
        self.uart = uart
        self.active_session = None
        self.arm_requested = False

    def send(self, request):
        data = json.dumps(request, separators=(',', ':'), ensure_ascii=True).encode()
        if len(data) > 2048:
            raise ValueError('JSON request exceeds 2048 bytes')
        self.uart.write(data + b'\n')
        self.uart.flush()

    def command(self, action, session_id, **fields):
        request_id = uuid.uuid4().hex
        self.send({'device': 'light_controller', 'request_id': request_id,
                   'ecg': {'action': action, 'session_id': session_id, **fields}})
        return request_id

    def exact(self, count, timeout=5):
        deadline = time.monotonic() + timeout
        data = bytearray()
        while len(data) < count:
            if time.monotonic() >= deadline:
                raise TimeoutError('Lost binary framing; reset ESP32 before a new session')
            part = self.uart.read(count - len(data))
            if part:
                data.extend(part)
        return bytes(data)

    def message(self, deadline):
        line = bytearray()
        while time.monotonic() < deadline:
            char = self.uart.read(1)
            if not char:
                continue
            line.extend(char)
            if len(line) > 65536:
                raise ValueError('Oversized incoming line')
            if char == b'\n':
                try:
                    result = json.loads(line)
                except (ValueError, UnicodeDecodeError):
                    print(line.decode('utf-8', errors='replace').rstrip())
                    line.clear()
                    continue
                if not isinstance(result, dict):
                    line.clear()
                    continue
                payload = None
                if result.get('type') == 'ecg.chunk':
                    count = result.get('bytes')
                    if type(count) is not int or not 1 <= count <= 1024:
                        raise ValueError('Invalid chunk length; cannot trust framing')
                    payload = self.exact(count)
                return result, payload
        raise TimeoutError('Timed out waiting for ESP32')


def capture(link, config, output):
    cap_id = uuid.uuid4().hex
    link.send({'device': 'light_controller', 'request_id': cap_id, 'get_capabilities': True})
    deadline = time.monotonic() + 5
    while True:
        capabilities, _ = link.message(deadline)
        if capabilities.get('type') == 'capabilities' and capabilities.get('request_id') == cap_id:
            break
    if capabilities.get('protocol_version') != 3:
        raise ValueError('This client requires protocol_version 3 firmware')
    features = capabilities['features']
    if not features.get('ecg') or not features.get('wav'):
        raise ValueError('Device does not advertise ECG/WAV support')
    boot_id = capabilities['boot_id']
    arm_id = uuid.uuid4().hex
    link.arm_requested = True
    link.send({'device': 'light_controller', 'request_id': arm_id,
               'ecg': {'action': 'arm', **config}})
    arm_deadline = time.monotonic() + 5
    while True:
        ack, _ = link.message(arm_deadline)
        if ack.get('request_id') != arm_id:
            continue
        if ack.get('type') != 'ecg.ack' or not ack.get('accepted'):
            raise ValueError('arm was rejected: ' + json.dumps(ack, ensure_ascii=False))
        session_id = ack['session_id']
        link.active_session = session_id
        print(json.dumps(ack, ensure_ascii=False), flush=True)
        break
    deadline = (time.monotonic() + config['wait_timeout_sec'] + config['duration_sec'] + 120
                if config['wait_timeout_sec'] else float('inf'))
    advertised = {}
    buffers = {}
    verified = {}
    current_file = None
    while True:
        message, payload = link.message(deadline)
        if message.get('boot_id') and message['boot_id'] != boot_id:
            raise ValueError('ESP32 rebooted; recording has been lost')
        if message.get('type') == 'ecg.error':
            raise ValueError(json.dumps(message, ensure_ascii=False))
        if message.get('session_id') != session_id:
            if payload is not None:
                raise ValueError('Unexpected binary file from another session')
            continue
        kind = message.get('type')
        print(json.dumps(message, ensure_ascii=False), flush=True)
        event = message.get('event')
        if kind == 'ecg.event' and event in ('failed', 'cancelled', 'transfer_failed'):
            raise ValueError('Session did not complete: ' + json.dumps(message, ensure_ascii=False))
        if event == 'result_ready':
            for item in message['files']:
                if item['name'] not in ('quality.json', 'ecg.wav') or not 0 < item['bytes'] <= 262144:
                    raise ValueError('Unexpected file metadata')
                advertised[item['file_id']] = item
                buffers[item['file_id']] = bytearray()
        elif event == 'file_begin':
            item = message['file']
            file_id = item['file_id']
            if file_id not in advertised or any(item[k] != advertised[file_id][k] for k in ('bytes', 'crc32', 'name')):
                raise ValueError('File metadata changed')
            current_file = file_id
        elif kind == 'ecg.chunk':
            file_id = message['file_id']
            if file_id != current_file or file_id not in buffers:
                raise ValueError('Chunk outside file_begin/file_end')
            position = message['offset']
            if type(position) is not int or position < 0 or position + len(payload) > advertised[file_id]['bytes']:
                raise ValueError('Invalid chunk offset')
            if message['chunk_seq'] != position // 1024:
                raise ValueError('Invalid chunk sequence')
            if crc(payload) != message['crc32']:
                link.command('chunk_ack', session_id, file_id=file_id, chunk_seq=message['chunk_seq'], offset=position, status='retry')
                continue
            data = buffers[file_id]
            if position == len(data):
                data.extend(payload)
            elif position > len(data) or bytes(data[position:position + len(payload)]) != payload:
                raise ValueError('Missing or inconsistent chunk')
            link.command('chunk_ack', session_id, file_id=file_id, chunk_seq=message['chunk_seq'], offset=position, status='ok')
        elif event == 'file_end':
            item = message['file']
            file_id = item['file_id']
            data = bytes(buffers[file_id])
            if len(data) != advertised[file_id]['bytes'] or crc(data) != advertised[file_id]['crc32']:
                raise ValueError('Whole-file size/CRC mismatch')
            name = advertised[file_id]['name']
            if name == 'quality.json':
                quality = json.loads(data)
                if quality['session_id'] != session_id or quality['boot_id'] != boot_id or not quality.get('recording_complete', quality.get('recording_valid')):
                    raise ValueError('Quality metadata does not match session')
                if quality['sample_rate_hz'] != 250 or quality['samples'] != config['duration_sec'] * 250:
                    raise ValueError('Quality duration/rate mismatch')
            else:
                if 'quality.json' not in verified:
                    raise ValueError('WAV arrived before verified quality.json')
                validate_wav(data, json.loads(verified['quality.json']))
            verified[name] = data
            link.command('file_received', session_id, file_id=file_id, bytes=len(data), crc32=crc(data))
        elif event == 'completed':
            if set(verified) != {'ecg.wav', 'quality.json'}:
                raise ValueError('Completion without both verified files')
            output.mkdir(parents=True, exist_ok=True)
            final = output / session_id
            # Publish the pair together by renaming their containing directory.
            with tempfile.TemporaryDirectory(prefix='.ecg-', dir=output) as staging:
                for name, data in verified.items():
                    with open(Path(staging) / name, 'wb') as stream:
                        stream.write(data)
                        stream.flush()
                        os.fsync(stream.fileno())
                os.rename(staging, final)
            release_id = link.command('release', session_id)
            release_deadline = time.monotonic() + 5
            while True:
                response, _ = link.message(release_deadline)
                if response.get('request_id') == release_id:
                    if response.get('type') != 'ecg.ack':
                        raise ValueError('Files saved, but ESP32 release failed')
                    break
            print('Saved: ' + str(final.resolve()))
            return final


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port')
    parser.add_argument('--duration', type=int, default=30)
    parser.add_argument('--wait', type=int, default=120)
    parser.add_argument('--continuous', type=int, default=0, help='Accepted session metadata; does not affect recording')
    parser.add_argument('--output', type=Path, default=Path('recordings'))
    args = parser.parse_args()
    config = dict(duration_sec=args.duration, wait_timeout_sec=args.wait, continuous_sec=args.continuous)
    with serial.Serial(port=None, baudrate=115200, timeout=0.2, write_timeout=2, exclusive=True) as uart:
        uart.dtr = uart.rts = False
        uart.port = args.port or find_port()
        uart.open()
        # Allow startup after a driver-induced reset before querying capabilities.
        time.sleep(2)
        uart.reset_input_buffer()
        link = Link(uart)
        try:
            capture(link, config, args.output)
        except KeyboardInterrupt:
            if link.active_session:
                link.command('cancel', link.active_session)
            elif link.arm_requested:
                link.send({'device': 'light_controller', 'request_id': uuid.uuid4().hex,
                           'ecg': {'action': 'cancel'}})
            raise


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, KeyError, TimeoutError, serial.SerialException) as exc:
        raise SystemExit(str(exc))
