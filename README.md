# Auroscope

Контроллер подсветки и ЭКГ. Прошивка **2.1**.

## Прошивка

Поместите `bootloader.bin`, `partitions.bin`, `boot_app0.bin`, `firmware.bin` из одной сборки в `~/esp32_firmware` на Raspberry Pi. Остановите программы, использующие USB-порт.

```bash
cd ~/esp32_firmware
python3 -m venv .venv
.venv/bin/pip install esptool
.venv/bin/python -m esptool --chip esp32 --port /dev/ttyUSB0 --baud 460800 write-flash 0x1000 bootloader.bin 0x8000 partitions.bin 0xe000 boot_app0.bin 0x10000 firmware.bin
```

## Связь и системные команды

USB-UART, обычно `/dev/ttyUSB0`, **115200 бод, 8N1**. Каждый запрос — одна строка JSON с `\n` в конце. Порт использует один процесс; открытие порта может перезапустить ESP32.

| JSON-команда | Ответ |
| --- | --- |
| `{"device":"light_controller","tickvi_status":true}` | `OK: tickvi_OK` |
| `{"device":"light_controller","echo":true}` | `OK: echo` |
| `{"device":"light_controller","get_tickvi_id":true}` | `OK: id:<число>` |
| `{"device":"light_controller","set_tickvi_id":2}` | `OK: id_set:2` |
| `{"device":"light_controller","get_tickvi_version":true}` | `OK: version:2.1` |
| `{"device":"light_controller","tickvi_test":true}` | Тест света, затем `OK: tickvi_test_OK` |
| `{"device":"light_controller","get_capabilities":true}` | JSON `type:"capabilities"` с возможностями |

Версия доступна только для чтения. Ошибки старых команд: `ERROR: ...`; ошибки ЭКГ: JSON `type:"ecg.error"`.

## Подсветка

| Группа | Режимы `mode` |
| --- | --- |
| `hands` | 1 — оранжевая пульсация; 2 — смена семи цветов с пульсацией; 3 — зелёный сканер |
| `stars` | 1 — постоянный фиолетовый; 2 — фиолетовая пульсация |

`action` — включить/выключить; `duration_sec` — секунды, `0` — бессрочно. Таймеры независимы. Новая команда сразу заменяет режим своей группы; пропущенная группа не меняется.

Включить обе группы:
```json
{"device":"light_controller","zones":{"hands":{"action":true,"mode":1,"duration_sec":0},"stars":{"action":true,"mode":1,"duration_sec":0}}}
```

Сканер рук на 5 секунд:
```json
{"device":"light_controller","zones":{"hands":{"action":true,"mode":3,"duration_sec":5}}}
```

Выключить всё:
```json
{"device":"light_controller","zones":{"hands":{"action":false},"stars":{"action":false}}}
```

Ответ: `OK: applied`.

## ЭКГ

`arm` — подготовить измерение: устройство ждёт контакта электродов и только после его подтверждения начинает запись. Это не немедленный старт записи.

Начать ожидание контакта и запись:
```json
{"device":"light_controller","request_id":"arm-001","ecg":{"action":"arm","duration_sec":30,"wait_timeout_sec":120}}
```

`duration_sec` — 10–60 секунд; `wait_timeout_sec` — 0–600 секунд (`0` — ждать бессрочно). Необязательный `continuous_sec` сохраняется в настройках, но на запись не влияет. Новый `arm` отменяет старую сессию. Для новых запросов используйте новые `request_id`.

| Действие | JSON | Ответ |
| --- | --- | --- |
| Статус | `{"device":"light_controller","ecg":{"action":"status"}}` | `ecg.status`: состояние, контакт, прогресс, пульс |
| Отмена | `{"device":"light_controller","ecg":{"action":"cancel"}}` | `ecg.ack`, событие `cancelled` |
| Освободить результат после передачи | `{"device":"light_controller","ecg":{"action":"release"}}` | `ecg.ack` |

После `arm` приходит `ecg.ack` с `accepted:true` и `session_id`. После устойчивого контакта (500 мс + 2000 мс стабилизации) — `recording_started`. Каждые 5 секунд записи приходит JSON `type:"pulse"`. Потеря/возврат контакта: `contact_lost` / `contact_restored`.

Начатая запись сохраняет полную длительность при отрывах: плохие отсчёты заменяются нулями и отмечаются в карте качества. Отмена и новый `arm` удаляют старый результат. Если контакт не появился до таймаута, приходят `contact_timeout` и `failed`, без WAV.

Результат: **`ecg.wav` (PCM16 mono, 250 Гц)** и **`quality.json`**. События: `result_ready` → передача файлов → `completed`. Файлы идут бинарными блоками с CRC; клиент подтверждает блоки (`chunk_ack`) и файлы (`file_received`). Используйте готовый приёмник ниже.

Пульс передаётся с `rate_scale:0.5`, итоговый диапазон — 45–200 BPM. Вне диапазона: `valid:false`, `bpm:null`, `bps:null`. Точность по эталону не подтверждена.

## Запуск с Raspberry Pi

Файлы из каталога `raspberry_pi` должны находиться в `~/raspberry_pi`.

**Все файлы запускать не нужно.** Выберите одну программу под задачу; одновременно порт использует только одна программа.

| Файл в `raspberry_pi` | Для чего нужен |
| --- | --- |
| `esp32_uart.py` | Проверить связь или отправить команду подсветки; не принимает WAV |
| `ecg_capture.py` | Записать ЭКГ и сохранить WAV + карту качества; для обычного измерения достаточно этого файла |
| `test_device.py` | Дополнительные подробные тесты устройства |

Тесты не требуются для каждого измерения и не прошивают устройство.

```bash
cd ~/raspberry_pi
python3 -m venv .venv
.venv/bin/pip install pyserial

# Проверка связи
.venv/bin/python esp32_uart.py --port /dev/ttyUSB0 probe

# Отправка команды
.venv/bin/python esp32_uart.py --port /dev/ttyUSB0 send '{"device":"light_controller","zones":{"hands":{"action":true,"mode":1,"duration_sec":0}}}' --expect 'OK: applied'

# Запись и сохранение WAV + quality.json
.venv/bin/python ecg_capture.py --port /dev/ttyUSB0 --duration 30 --wait 120 --output recordings
```
