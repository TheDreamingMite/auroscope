#!/usr/bin/env python3
"""Интерактивные JSON-тесты подсветки и ЭКГ ESP32 через USB, 115200 8N1.

Зависимость: pyserial. Примеры:
  python test_device.py --port /dev/ttyUSB0 --suite lighting
  python test_device.py --port /dev/ttyUSB0 --suite pulse
  python test_device.py --port /dev/ttyUSB0 --suite all
  python test_device.py --list
"""
import argparse
import base64
from collections import Counter
from datetime import datetime, timezone
import glob
import io
import json
import math
from pathlib import Path
import sys
import time
import uuid
import wave
import zlib

import serial


class TestFailure(Exception):
    pass


class FramingError(TestFailure):
    pass


def require(condition, message):
    if not condition:
        raise TestFailure(message)


def checksum(data):
    return f'{zlib.crc32(data) & 0xffffffff:08x}'


def request(action, sid=None, **fields):
    ecg = dict(action=action, **fields)
    if sid is not None:
        ecg['session_id'] = sid
    return dict(device='light_controller', request_id=uuid.uuid4().hex, ecg=ecg)


def lights(hands=False, mode=1, seconds=0, stars=False, star_mode=1, star_seconds=0):
    return {'device': 'light_controller', 'zones': {
        'hands': {'action': hands, 'mode': mode, 'duration_sec': seconds},
        'stars': {'action': stars, 'mode': star_mode, 'duration_sec': star_seconds}}}


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


class Log:
    def __init__(self, directory):
        self.directory = directory
        directory.mkdir(parents=True, exist_ok=False)
        self.stream = (directory / 'events.jsonl').open('w', encoding='utf-8', buffering=1)
        self.started = time.monotonic()

    def write(self, kind, **fields):
        record = dict(time=datetime.now(timezone.utc).isoformat(),
                      elapsed_sec=round(time.monotonic() - self.started, 3), kind=kind, **fields)
        line = json.dumps(record, ensure_ascii=False, allow_nan=False)
        self.stream.write(line + '\n')
        print(line, flush=True)

    def close(self):
        self.stream.close()


class Device:
    def __init__(self, uart, log):
        self.uart, self.log = uart, log
        self.history = []
        self.partial = bytearray()
        self.boot_id = None
        self.files = {}
        self.configs = {}
        self.retry_sessions = set()
        self.retried = set()
        self.expected_retry = {}
        self.framing_ok = True
        self.last_arm_sid = None
        self.expected_errors = {}

    def send(self, command):
        encoded = json.dumps(command, separators=(',', ':'), ensure_ascii=True, allow_nan=False).encode('ascii')
        require(len(encoded) <= 2048, 'Команда длиннее 2048 байт')
        self.log.write('TX', json=command)
        self.uart.write(encoded + b'\n')
        self.uart.flush()
        return command.get('request_id')

    def exact(self, count):
        data = bytearray()
        deadline = time.monotonic() + 5
        while len(data) < count:
            if time.monotonic() >= deadline:
                self.framing_ok = False
                raise FramingError('Потеряна граница бинарного блока. Перезапустите ESP32 перед новым запуском теста.')
            data.extend(self.uart.read(count - len(data)))
        return bytes(data)

    def read(self, deadline):
        while time.monotonic() < deadline:
            char = self.uart.read(1)
            if not char:
                continue
            self.partial.extend(char)
            if len(self.partial) > 65536:
                self.framing_ok = False
                raise FramingError('Входящая строка превышает 64 КБ')
            if char != b'\n':
                continue
            line = bytes(self.partial).rstrip(b'\r\n')
            self.partial.clear()
            try:
                message = json.loads(line)
            except (ValueError, UnicodeDecodeError):
                message = line.decode('utf-8', errors='replace')
            if not isinstance(message, dict):
                message = str(message)
                self.log.write('RX_TEXT', text=message)
            else:
                self.log.write('RX', json=message)
                if self.boot_id and message.get('boot_id') and message['boot_id'] != self.boot_id:
                    raise TestFailure('Изменился boot_id: ESP32 перезапустилась')
                payload = None
                if message.get('type') == 'ecg.chunk':
                    count = message.get('bytes')
                    if type(count) is not int or not 1 <= count <= 1024:
                        self.framing_ok = False
                        raise FramingError('Некорректная длина бинарного блока')
                    payload = self.exact(count)
                    self.log.write('RX_BINARY', file_id=message.get('file_id'),
                                   offset=message.get('offset'), bytes=count, crc32=checksum(payload))
                self.receive_file(message, payload)
            self.history.append(message)
            return message
        raise TimeoutError('Истекло время ожидания ответа ESP32')

    def wait(self, predicate, timeout=5, since=None):
        index = len(self.history) if since is None else since
        deadline = time.monotonic() + timeout
        while True:
            while index < len(self.history):
                message = self.history[index]
                index += 1
                if predicate(message):
                    return message, index
            self.read(deadline)

    def rpc(self, command, expected='ecg.ack', code=None, timeout=5):
        start = len(self.history)
        self.send(command)
        result, _ = self.wait(lambda m: isinstance(m, dict) and m.get('request_id') == command['request_id'],
                              timeout, start)
        require(result.get('type') == expected, f'Ожидался {expected}, получено {result}')
        if code:
            require(result.get('code') == code, f'Ожидалась ошибка {code}, получено {result}')
        if expected == 'ecg.error':
            # Negative-test replies stay in history. Skip them on replay only
            # after their exact request ID and error code have been checked.
            self.expected_errors[command['request_id']] = result.get('code')
        return result

    def is_expected_error(self, message):
        rid = message.get('request_id')
        return (message.get('type') == 'ecg.error' and rid in self.expected_errors
                and self.expected_errors[rid] == message.get('code'))

    def legacy(self, command, expected, timeout=5):
        start = len(self.history)
        self.send(command)
        result, _ = self.wait(lambda m: isinstance(m, str) and
                              (m.startswith(expected) or m.startswith('ERROR:') or m.startswith('OK: error:')),
                              timeout, start)
        require(result.startswith(expected), f'Ожидался {expected}, получено {result}')
        return result

    def drain(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            try:
                self.read(min(deadline, time.monotonic() + 0.2))
            except TimeoutError:
                pass

    def event(self, sid, names, timeout, since):
        if isinstance(names, str):
            names = (names,)
        return self.wait(lambda m: isinstance(m, dict) and m.get('session_id') == sid
                         and m.get('type') == 'ecg.event' and m.get('event') in names, timeout, since)

    def arm(self, duration=10, wait=120, continuous=0):
        sid = 't-' + uuid.uuid4().hex
        command = request('arm', sid, duration_sec=duration, wait_timeout_sec=wait, continuous_sec=continuous)
        self.configs[sid] = command['ecg']
        self.last_arm_sid = sid
        start = len(self.history)
        ack = self.rpc(command)
        require(ack.get('accepted') is True and ack.get('state') == 'waiting_contact', 'Некорректный ACK arm')
        return sid, command, start

    def cancel(self, sid):
        start = len(self.history)
        self.rpc(request('cancel', sid))
        self.event(sid, 'cancelled', 5, start)

    def receive_file(self, message, payload):
        event, sid = message.get('event'), message.get('session_id')
        if event == 'result_ready':
            require(message.get('recording_complete', message.get('recording_valid')) is True,
                    'Запись не завершена')
            items = message['files']
            require(len(items) == 2 and {f['name'] for f in items} == {'quality.json', 'ecg.wav'},
                    'Ожидались quality.json и ecg.wav')
            for item in items:
                require(type(item['bytes']) is int and 0 < item['bytes'] <= 262144, 'Некорректный размер файла')
                self.files[item['file_id']] = dict(meta=item, sid=sid, data=bytearray(), verified=False,
                                                  begun=False, received_request=None)
        elif event == 'file_begin':
            meta = message['file']
            f = self.files[meta['file_id']]
            require(f['sid'] == sid and all(f['meta'][k] == meta[k] for k in ('bytes', 'crc32', 'name')),
                    'Метаданные файла изменились')
            require(message.get('offset', 0) <= len(f['data']), 'Нет префикса для возобновления')
            f['begun'] = True
        elif payload is not None:
            fid = message['file_id']
            require(fid in self.files, 'Блок неизвестного файла')
            f = self.files[fid]
            require(f['begun'] and f['sid'] == sid, 'Блок вне передачи своей сессии')
            offset = message['offset']
            require(type(offset) is int and 0 <= offset < f['meta']['bytes']
                    and offset + len(payload) <= f['meta']['bytes']
                    and message['chunk_seq'] == offset // 1024, 'Неверная позиция/номер блока')
            force_retry = sid in self.retry_sessions and sid not in self.retried
            if force_retry:
                self.retried.add(sid)
                self.expected_retry[(fid, offset)] = payload
                self.log.write('NOTE', message='Намеренно запрошен повтор первого блока для проверки retry')
            good = checksum(payload) == message['crc32'] and not force_retry
            if good:
                if (fid, offset) in self.expected_retry:
                    require(payload == self.expected_retry.pop((fid, offset)), 'Повторённый блок отличается от исходного')
                if offset == len(f['data']):
                    f['data'].extend(payload)
                else:
                    require(bytes(f['data'][offset:offset + len(payload)]) == payload,
                            'Пропуск или несовпадение повторного блока')
            self.send(request('chunk_ack', sid, file_id=fid, chunk_seq=message['chunk_seq'],
                              offset=offset, status='ok' if good else 'retry'))
        elif event == 'file_end':
            meta = message['file']
            fid = meta['file_id']
            f = self.files[fid]
            data = bytes(f['data'])
            require(len(data) == f['meta']['bytes'] and checksum(data) == f['meta']['crc32'],
                    'Не совпали размер/CRC целого файла')
            if f['meta']['name'] == 'quality.json':
                quality = json.loads(data)
                require(quality['boot_id'] == self.boot_id and quality['session_id'] == sid
                        and quality.get('recording_complete', quality.get('recording_valid')) is True,
                        'Карта качества другой сессии или незавершённой записи')
                if sid in self.configs:
                    require(quality['samples'] == self.configs[sid]['duration_sec'] * 250
                            and quality['sample_rate_hz'] == 250, 'Неверная длительность/частота записи')
            else:
                maps = [v for v in self.files.values() if v['sid'] == sid and v['meta']['name'] == 'quality.json']
                require(len(maps) == 1 and maps[0]['verified'], 'WAV получен до проверенной карты качества')
                validate_wav(data, json.loads(maps[0]['data']))
            f['verified'] = True
            command = request('file_received', sid, file_id=fid, bytes=len(data), crc32=checksum(data))
            f['received_request'] = self.send(command)
            self.log.write('FILE_VERIFIED', session_id=sid, name=f['meta']['name'], bytes=len(data), crc32=checksum(data))

    def save(self, sid, label, output):
        items = [f for f in self.files.values() if f['sid'] == sid]
        require(len(items) == 2 and all(f['verified'] for f in items), 'Не проверены оба файла')
        temporary = output / f'.{label}-{sid}.partial'
        final = output / f'{label}-{sid}'
        temporary.mkdir(parents=True)
        for f in items:
            (temporary / f['meta']['name']).write_bytes(f['data'])
        temporary.rename(final)
        self.log.write('FILES_SAVED', path=str(final.resolve()))
        self.rpc(request('release', sid))
        return json.loads(next(f['data'] for f in items if f['meta']['name'] == 'quality.json'))


CASES = {
    'lighting': ['L01', 'L02', 'L03', 'L04', 'L05', 'L06', 'L07', 'L08', 'L09', 'L10', 'L11', 'L12', 'L13', 'L14'],
    'pulse': ['P01', 'P02', 'P03', 'P04', 'P05', 'P06', 'P07', 'P08', 'P09', 'P10', 'P11', 'P12'],
}
DESCRIPTIONS = {
    'L01': 'Проверка echo, tickvi_status, чтение ID и дробной версии',
    'L02': 'Руки mode 1 + звёзды mode 1 без таймера (сценарий 1.4.1)',
    'L03': 'Руки mode 2 + звёзды mode 1 без таймера (1.4.2)',
    'L04': 'Руки mode 3 на 5 секунд, звёзды остаются включены (1.4.3)',
    'L05': 'Руки выключены, звёзды включены (1.4.4)',
    'L06': 'Вся подсветка выключена (1.4.5)',
    'L07': 'Руки mode 1/2/3 по отдельности, звёзды выключены',
    'L08': 'Звёзды mode 2: пульсация',
    'L09': 'Таймер звёзд не выключает руки',
    'L10': 'Раздельные таймеры рук и звёзд',
    'L11': 'Неверный режим включения и неверный device отклоняются',
    'L12': 'При action:false игнорируются mode и duration_sec',
    'L13': 'Немедленное выключение и замена текущего режима',
    'L14': 'Частичный zones-запрос сохраняет вторую группу',
    'P01': 'Возможности и статус модуля',
    'P02': 'Неверные настройки не уничтожают сессию; повтор arm; конфликт ID',
    'P03': 'Без электродов: ожидание и wait_timeout',
    'P04': 'Отмена во время ожидания',
    'P05': 'Полная запись 10 секунд, пульс и WAV/CRC',
    'P06': 'Полная запись 30 секунд, пульс и WAV/CRC',
    'P07': 'Полная запись 60 секунд, пульс и WAV/CRC',
    'P08': 'Снятие электродов: запись продолжается до конца, WAV содержит отметки отрыва',
    'P09': 'Отрыв и восстановление: события и карта качества',
    'P10': 'Отмена уже начатой записи',
    'P11': 'Новый arm прерывает начатую запись, старый файл недоступен',
    'P12': 'Автоматическая передача WAV/quality, неверные offset/CRC, повтор блока',
}


class Runner:
    def __init__(self, device, args, log):
        self.d, self.args, self.log = device, args, log
        self.results = []
        self.visual = 'NOT_CHECKED'
        self.visual_answers = []
        self.capabilities = None
        self.active_sid = None

    def prompt(self, instruction):
        if self.args.non_interactive:
            return False
        print('\n' + instruction, flush=True)
        answer = input('Enter — начать; s — пропустить: ').strip().lower()
        return answer != 's'

    def observe(self, instruction):
        self.log.write('VISUAL_INSTRUCTION', message=instruction)
        self.d.drain(self.args.hold)
        if self.args.non_interactive:
            self.visual = 'NOT_CHECKED'
            return
        answer = input('Наблюдение соответствует? [Enter/да — да, n — нет, s — не проверено]: ').strip().lower()
        if answer in ('n', 'no', 'нет'):
            self.visual = 'FAIL'
            raise TestFailure('Визуальная проверка не пройдена: ' + instruction)
        self.visual_answers.append(answer != 's')
        self.visual = 'PASS' if all(self.visual_answers) else 'NOT_CHECKED'

    def light(self, command):
        self.d.legacy(command, 'OK: applied')

    def arm(self, **fields):
        try:
            sid, command, start = self.d.arm(**fields)
        finally:
            self.active_sid = self.d.last_arm_sid
        return sid, command, start

    def cancel(self, sid):
        self.d.cancel(sid)
        self.active_sid = None

    def capabilities_test(self):
        cmd = dict(device='light_controller', request_id=uuid.uuid4().hex, get_capabilities=True)
        result = self.d.rpc(cmd, expected='capabilities')
        self.capabilities = result
        self.d.boot_id = result['boot_id']
        require(result.get('protocol_version') == 3, 'Нужна обновлённая прошивка с protocol_version 3')
        require(result['features'].get('ecg') and result['features'].get('wav'), 'Модуль не сообщает поддержку ЭКГ/WAV')
        require(result['ecg']['sample_rate_hz'] == 250, 'Неожиданная частота ЭКГ')
        self.d.rpc(request('status'), expected='ecg.status')

    def record(self, case, duration, mode='normal', transport_test=False):
        instructions = {
            'normal': f'Приложите электроды и не снимайте их до завершения записи {duration} с.',
            'loss': 'Приложите электроды. После сообщения «СНИМИТЕ» снимите их до окончания записи. WAV должен прийти в любом случае.',
            'recovery': 'Приложите электроды. После сообщения «СНИМИТЕ» снимите их примерно на 1 секунду и верните.',
            'cancel': 'Приложите электроды: после первого отчёта пульса программа отменит запись.',
            'replace': 'Приложите электроды: после первого отчёта пульса программа заменит сессию новой.',
        }
        if not self.prompt(instructions[mode]):
            return False
        warmup_seconds = getattr(self.args, 'warmup_seconds', 5)
        if warmup_seconds:
            self.log.write('WARMUP_START', seconds=warmup_seconds,
                           message='Держите электроды приложенными и сохраняйте неподвижность. USB-порт остаётся открытым.')
            warmup_end = time.monotonic() + warmup_seconds
            while time.monotonic() < warmup_end:
                status = self.d.rpc(request('status'), expected='ecg.status')
                self.log.write('WARMUP_STATUS', remaining_sec=round(max(0, warmup_end - time.monotonic()), 1),
                               contact=status.get('contact'), adc=status.get('diagnostics', {}).get('adc'))
                self.d.drain(min(5, max(0, warmup_end - time.monotonic())))
            self.log.write('WARMUP_FINISHED', message='Пауза закончена; отправляем arm. Качество проверит измерительная сессия.')
        sid, command, start = self.arm(duration=duration)
        if transport_test:
            self.d.retry_sessions.add(sid)
        cursor, pulse_reports, seen = start, [], set()
        recording_start_ms = None
        deadline = time.monotonic() + 120 + duration + 90
        instructed = False
        while True:
            m, cursor = self.d.wait(lambda x: isinstance(x, dict) and x.get('session_id') == sid,
                                    max(0.1, deadline - time.monotonic()), cursor)
            require(time.monotonic() < deadline, 'Превышено время сценария')
            if m.get('type') == 'ecg.error':
                if self.d.is_expected_error(m):
                    continue
                raise TestFailure(str(m))
            event = m.get('event')
            if event:
                seen.add(event)
            if event == 'recording_started':
                recording_start_ms = m['start_ms']
                require(m['sample_rate_hz'] == 250 and m['target_samples'] == duration * 250,
                        'Неверная частота/длительность recording_started')
            if m.get('type') == 'pulse':
                pulse_reports.append(m)
                if not instructed:
                    instructed = True
                    if mode in ('loss', 'recovery'):
                        self.log.write('OPERATOR_ACTION', message='СНИМИТЕ ЭЛЕКТРОДЫ СЕЙЧАС' +
                                       (' примерно на 1 секунду, затем верните' if mode == 'recovery' else ''))
                    elif mode == 'cancel':
                        self.cancel(sid)
                        status = self.d.rpc(request('status'), expected='ecg.status')
                        require(status['state'] == 'cancelled', 'Запись не отменена')
                        return True
                    elif mode == 'replace':
                        new_sid, _, replacement_mark = self.arm(duration=10)
                        old_event, _ = self.d.event(sid, 'cancelled', 5, replacement_mark)
                        require(old_event.get('reason') == 'replaced'
                                and old_event.get('replaced_by_session_id') == new_sid, 'Нет корректного события замены')
                        self.cancel(new_sid)
                        self.d.rpc(request('file_get', sid, file_id=sid + '-wav', offset=0),
                                   expected='ecg.error', code='session_mismatch')
                        return True
            if event in ('failed', 'cancelled', 'transfer_failed'):
                self.active_sid = None if event != 'transfer_failed' else sid
                if event == 'failed':
                    diagnostic_status = self.d.rpc(request('status', sid), expected='ecg.status')
                    self.log.write('FAILURE_DIAGNOSTICS', session_id=sid,
                                   diagnostics=m.get('diagnostics') or diagnostic_status.get('diagnostics'),
                                   failure_code=m.get('code'))
                raise TestFailure('Измерение не завершилось: ' + str(m))
            if transport_test and event == 'result_ready':
                items = {f['name']: f for f in m['files']}
                self.d.rpc(request('file_get', sid, file_id=items['ecg.wav']['file_id'], offset=1),
                           expected='ecg.error', code='invalid_parameter')
                bad_crc = f'{int(items["ecg.wav"]["crc32"], 16) ^ 0xffffffff:08x}'
                self.d.rpc(request('file_received', sid, file_id=items['ecg.wav']['file_id'],
                                   bytes=items['ecg.wav']['bytes'], crc32=bad_crc),
                           expected='ecg.error', code='file_crc_mismatch')
            if event == 'completed':
                require('recording_started' in seen and 'recording_finished' in seen and 'result_ready' in seen,
                        'Неполная последовательность событий')
                require(len(pulse_reports) == duration // 5, 'Не получен отчёт пульса за каждое 5-секундное окно')
                for i, report in enumerate(pulse_reports, 1):
                    require(report['seq'] == i and report['window_ms'] == 5000, 'Пропуск/неверное окно пульса')
                    require(recording_start_ms is not None and
                            (report['t_ms'] - recording_start_ms) % (2 ** 32) == i * 5000,
                            'Отчёты пульса не привязаны к 5-секундным окнам записи')
                    require(type(report['valid']) is bool, 'valid должен быть логическим значением')
                    if report['valid']:
                        bps, bpm = report['bps'], report['bpm']
                        require(type(bps) in (int, float) and type(bpm) in (int, float)
                                and math.isfinite(bps) and math.isfinite(bpm)
                                and 45 <= bpm <= 200 and abs(bpm - bps * 60) < 0.2, 'Неверные единицы BPM/BPS')
                    else:
                        require(report['bps'] is None and report['bpm'] is None, 'Недостоверный пульс должен быть null')
                maps = [f for f in self.d.files.values() if f['sid'] == sid and f['meta']['name'] == 'quality.json']
                require(len(maps) == 1 and maps[0]['verified'], 'Нет проверенной карты качества')
                quality = json.loads(maps[0]['data'])
                if mode == 'recovery':
                    require({'contact_lost', 'contact_restored', 'signal_resumed'} <= seen,
                            'Не зафиксированы отрыв и полное восстановление')
                    require(quality['invalid_samples'] > 0, 'Отрыв не отмечен в карте качества')
                if mode == 'loss':
                    require(instructed and 'contact_lost' in seen, 'Не зафиксирован запланированный отрыв')
                    require(any(mask & 1 for mask in quality_flags(quality)), 'Отрыв не сохранён в карте качества')
                if transport_test:
                    require(sid in self.d.retried, 'Повтор блока не проверен')
                self.d.save(sid, case, self.log.directory / 'recordings')
                self.active_sid = None
                self.log.write('PULSE_CHECK', reports=len(pulse_reports),
                               valid_reports=sum(bool(p['valid']) for p in pulse_reports),
                               detector_accuracy='NOT_VERIFIED', invalid_samples=quality['invalid_samples'])
                return True

    def execute(self, case):
        if case == 'L01':
            self.d.legacy({'echo': True}, 'OK: echo')
            for field, expected in [('tickvi_status', 'OK: tickvi_OK'), ('get_tickvi_id', 'OK: id:'),
                                    ('get_tickvi_version', 'OK: version:')]:
                result = self.d.legacy(dict(device='light_controller', **{field: True}), expected)
                if field == 'get_tickvi_version':
                    value = result[len(expected):]
                    require(math.isfinite(float(value)), 'Некорректная версия')
                    if not self.args.legacy:
                        require('.' in value, 'Новая ESP32 должна возвращать дробную версию; для старой платы используйте --legacy')
                    self.log.write('VERSION', value=value, fractional='.' in value)
        elif case in ('L02', 'L03'):
            mode = 1 if case == 'L02' else 2
            self.light(lights(True, mode, stars=True))
            self.observe(f'Руки работают в режиме {mode}; фиолетовый фон включён.')
        elif case == 'L04':
            self.light(lights(True, 3, self.args.scanner_seconds, stars=True))
            self.log.write('VISUAL_INSTRUCTION', message=f'Сканер рук работает {self.args.scanner_seconds} с, затем руки гаснут; звёзды остаются.')
            self.d.drain(self.args.scanner_seconds + 1)
            self.observe('Руки выключились по таймеру, звёзды продолжают гореть.')
        elif case == 'L05':
            self.light(lights(False, 2, stars=True))
            self.observe('Руки выключены, фиолетовый фон включён.')
        elif case == 'L06':
            self.light(lights(False, 2, stars=False))
            self.observe('Все светодиодные группы выключены.')
        elif case == 'L07':
            for mode in (1, 2, 3):
                self.light(lights(True, mode))
                self.observe(f'Только руки: режим {mode}; фон выключен.')
        elif case == 'L08':
            if self.args.legacy:
                return False
            self.light(lights(stars=True, star_mode=2))
            self.observe('Только звёзды: пульсация')
        elif case == 'L09':
            self.light(lights(True, 1, stars=True, star_seconds=3))
            self.d.drain(4)
            self.observe('Звёзды выключились по таймеру; руки продолжают пульсировать.')
        elif case == 'L10':
            self.log.write('VISUAL_INSTRUCTION', message='Наблюдайте: сначала обе группы; через 3 с гаснут руки; через 6 с гаснут звёзды.')
            self.light(lights(True, 3, seconds=3, stars=True, star_seconds=6))
            self.d.drain(3.5)
            self.log.write('VISUAL_INSTRUCTION', message='Сейчас руки должны быть выключены, звёзды ещё горят.')
            self.d.drain(3.5)
            self.observe('Руки погасли через 3 с, звёзды через 6 с; сейчас обе группы выключены.')
        elif case == 'L11':
            self.d.legacy(lights(True, 9), 'ERROR: hands mode must be 1, 2, or 3')
            command = lights(True, 1); command['device'] = 'wrong_device'
            self.d.legacy(command, 'ERROR: wrong device')
        elif case == 'L12':
            self.light(lights(False, 99, seconds=99, stars=False, star_mode=99, star_seconds=99))
            self.observe('Обе группы выключены, несмотря на mode=99 и duration_sec=99.')
        elif case == 'L13':
            self.log.write('NOTE', message='Согласованное поведение: новая команда сразу заменяет текущую; очереди нет.')
            self.light(lights(True, 1, 10, stars=True))
            self.d.drain(1)
            self.light(lights(True, 3, stars=True))
            self.observe('Сканер начался сразу, не дожидаясь завершения предыдущей команды.')
            self.light(lights())
            self.observe('Выключение прервало режим сразу.')
        elif case == 'L14':
            self.light(lights(True, 1, stars=True))
            self.light({'device': 'light_controller', 'zones': {'hands': {'action': False}}})
            self.observe('После частичного запроса выключились только руки, звёзды остались.')
            self.light({'device': 'light_controller', 'zones': {'stars': {'action': False}}})
            self.observe('После выключения звёзд обе группы выключены.')
        elif case == 'P01':
            self.capabilities_test()
        elif case == 'P02':
            sid, arm, _ = self.arm(duration=10)
            self.d.rpc(arm)
            conflict = json.loads(json.dumps(arm)); conflict['ecg']['duration_sec'] = 30
            self.d.rpc(conflict, expected='ecg.error', code='request_id_conflict')
            for fields, code in [({'duration_sec': 9}, 'invalid_parameter'),
                                 ({'sample_rate_hz': 500}, 'invalid_parameter'),
                                 ({'typo_setting': 1}, 'invalid_parameter')]:
                e = dict(action='arm', session_id='bad-' + uuid.uuid4().hex, duration_sec=10)
                e.update(fields)
                command = dict(device='light_controller', request_id=uuid.uuid4().hex, ecg=e)
                self.d.rpc(command, expected='ecg.error', code=code)
                status = self.d.rpc(request('status'), expected='ecg.status')
                require(status['session_id'] == sid, 'Ошибочный arm уничтожил текущую сессию')
            self.cancel(sid)
        elif case == 'P03':
            if not self.prompt('Снимите электроды и оставьте их снятыми: проверяем таймаут ожидания.'):
                return False
            # Build explicitly because this scenario overrides the usual 120 s wait.
            sid = 't-' + uuid.uuid4().hex
            self.active_sid = sid
            start = len(self.d.history)
            self.d.rpc(request('arm', sid, duration_sec=10, wait_timeout_sec=3))
            result, _ = self.d.event(sid, 'failed', 8, start)
            require(result.get('code') == 'wait_timeout', 'Ожидался wait_timeout без электродов')
            require(not any(isinstance(m, dict) and m.get('session_id') == sid and
                            m.get('event') == 'recording_started' for m in self.d.history[start:]), 'Запись началась без ожидаемого контакта')
            self.active_sid = None
        elif case == 'P04':
            if not self.prompt('Снимите электроды: проверяем отмену ожидания.'):
                return False
            sid, _, _ = self.arm(duration=10)
            status = self.d.rpc(request('status'), expected='ecg.status')
            require(status['state'] == 'waiting_contact', 'Ожидалось waiting_contact')
            self.cancel(sid)
        elif case in ('P05', 'P06', 'P07'):
            return self.record(case, {'P05': 10, 'P06': 30, 'P07': 60}[case])
        elif case in ('P08', 'P09', 'P10', 'P11'):
            return self.record(case, 30, {'P08': 'loss', 'P09': 'recovery', 'P10': 'cancel', 'P11': 'replace'}[case])
        elif case == 'P12':
            return self.record(case, 10, transport_test=True)
        return True

    def run_case(self, case):
        self.visual = 'NOT_APPLICABLE' if case.startswith('P') or case in ('L01', 'L11') else 'NOT_CHECKED'
        self.visual_answers = []
        self.log.write('TEST_START', case=case, description=DESCRIPTIONS[case])
        started = time.monotonic()
        result = dict(case=case, description=DESCRIPTIONS[case])
        try:
            if self.args.non_interactive and case in ('P03', 'P04', 'P05', 'P06', 'P07', 'P08', 'P09', 'P10', 'P11', 'P12'):
                result.update(status='SKIP', reason='Нужны действия с электродами; запустите без --non-interactive')
            else:
                result['status'] = 'PASS' if self.execute(case) else 'SKIP'
        except FramingError:
            result.update(status='FAIL', reason='Потеряна граница бинарных данных')
            raise
        except (TestFailure, TimeoutError, ValueError, KeyError, wave.Error) as exc:
            result.update(status='FAIL', reason=str(exc))
        finally:
            result.update(visual=self.visual, duration_sec=round(time.monotonic() - started, 3))
            self.results.append(result)
            self.log.write('TEST_RESULT', **result)
        if self.active_sid:
            self.cancel(self.active_sid)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--suite', choices=('all', 'lighting', 'pulse'), default='all')
    parser.add_argument('--cases', help='Только указанные сценарии, например L02,L03,P05')
    parser.add_argument('--list', action='store_true', help='Показать сценарии без подключения к порту')
    parser.add_argument('--hold', type=float, default=3, help='Время демонстрации светового режима, секунд')
    parser.add_argument('--scanner-seconds', type=int, choices=(5, 10), default=5)
    parser.add_argument('--warmup-seconds', type=int, default=5,
                        help='Ожидание с приложенными электродами перед каждой записью ЭКГ; по умолчанию 5 с')
    parser.add_argument('--non-interactive', action='store_true', help='Не спрашивать подтверждения; ручные проверки ЭКГ пропускаются')
    parser.add_argument('--legacy', action='store_true', help='Только старые световые команды, без расширений ESP32/ЭКГ')
    parser.add_argument('--output', type=Path, default=Path('test_logs'))
    args = parser.parse_args()
    if args.list:
        for suite, cases in CASES.items():
            print(suite + ':')
            for case in cases:
                print('  ' + case + ' — ' + DESCRIPTIONS[case])
        return 0
    if not math.isfinite(args.hold) or args.hold <= 0:
        parser.error('--hold должен быть положительным числом')
    if not 0 <= args.warmup_seconds <= 120:
        parser.error('--warmup-seconds должен быть в пределах 0..120')
    cases = CASES['lighting'] + CASES['pulse'] if args.suite == 'all' else CASES[args.suite]
    if args.cases:
        cases = [s.strip().upper() for s in args.cases.split(',')]
        if not cases or any(case not in DESCRIPTIONS for case in cases):
            parser.error('Неизвестный сценарий; используйте --list')
    if args.legacy:
        cases = [case for case in cases if case.startswith('L')]
    if not cases:
        parser.error('Не выбрано ни одного сценария')
    directory = args.output / (datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + uuid.uuid4().hex[:6])
    log = Log(directory)
    runner, device = None, None
    exit_code = 0
    try:
        log.write('RUN_START', port=args.port, baud=115200, cases=cases,
                  visual_check='MANUAL' if not args.non_interactive else 'NOT_CHECKED',
                  queue_supported=False, pulse_accuracy='NOT_VERIFIED')
        port = args.port
        if port == 'auto':
            ports = sorted(glob.glob('/dev/serial/by-id/*')) or sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))
            require(len(ports) == 1, 'Укажите --port: найдено несколько портов или ни одного: ' + ', '.join(ports))
            port = ports[0]
        with serial.Serial(port=None, baudrate=115200, bytesize=8, parity='N', stopbits=1,
                           timeout=0.1, write_timeout=3, exclusive=True) as uart:
            uart.dtr = uart.rts = False
            uart.port = port
            uart.open()
            device = Device(uart, log)
            runner = Runner(device, args, log)
            try:
                device.drain(2)
                device.legacy({'echo': True}, 'OK: echo', timeout=15)
                if any(case.startswith('P') for case in cases):
                    runner.capabilities_test()
                    status = device.rpc(request('status'), expected='ecg.status')
                    require(status['state'] in ('idle', 'cancelled', 'failed', 'completed'),
                            'На ESP32 уже есть незавершённая сессия; завершите её перед тестированием')
                for case in cases:
                    runner.run_case(case)
            finally:
                if device.framing_ok:
                    try:
                        if runner.active_sid:
                            runner.cancel(runner.active_sid)
                        device.legacy(lights(), 'OK: applied')
                    except (TestFailure, TimeoutError, OSError, ValueError, KeyError) as exc:
                        log.write('CLEANUP_ERROR', message=str(exc))
                        exit_code = 1
    except KeyboardInterrupt:
        log.write('INTERRUPTED', message='Тест остановлен пользователем')
        exit_code = 130
    except (TestFailure, TimeoutError, OSError, ValueError, KeyError, serial.SerialException) as exc:
        log.write('RUN_ERROR', message=str(exc))
        exit_code = 1
    finally:
        results = runner.results if runner else []
        counts = dict(Counter(r['status'] for r in results))
        summary = dict(results=results, counts=counts, selected=cases,
                       not_run=[case for case in cases if case not in {r['case'] for r in results}],
                       visual_not_checked=[r['case'] for r in results if r['visual'] == 'NOT_CHECKED'],
                       pulse_accuracy='NOT_VERIFIED', queue_supported=False)
        summary['exit_code'] = exit_code or (1 if any(r['status'] == 'FAIL' for r in results) else 0)
        summary['overall'] = ('INTERRUPTED' if summary['exit_code'] == 130 else 'FAIL'
                              if summary['exit_code'] else 'PASS_WITH_SKIPS'
                              if counts.get('SKIP') or summary['not_run'] else 'PASS')
        (directory / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
        log.write('RUN_SUMMARY', **summary)
        log.close()
    return exit_code or (1 if any(r['status'] == 'FAIL' for r in results) else 0)


if __name__ == '__main__':
    raise SystemExit(main())
