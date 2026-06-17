# Auroscope

## Light Controller Firmware

Custom ESP8266 firmware for driving addressable LED matrices via a structured JSON-over-UART protocol. Designed to manage independent "Hands" and "Stars"

### ✨ Features
* **Hands Zone (3 Animation Modes):**
  1. **Color Cycle:** 0.4 Hz breathing (min to 80% brightness), cycling through 7 specific RGB colors.
  2. **Constant Color:** 0.4 Hz breathing (min to 60% brightness) in Orange (`RGB: 255,128,0`).
  3. **Scanline:** 2-LED wide green line scanning up/down over a Magenta (`RGB: 204,0,204`) background.
* **Stars Zone:** Constant Purple (`RGB: 76,0,153`) at 60% brightness.
* **Auto-Timeouts:** Automatically turns off zones after a specified duration.

---

### 🔌 Hardware Pinout (Host ➔ ESP8266)
The automated testing environment utilizes a host board (e.g., Raspberry Pi) to handle UART communication and bootloader sequencing.

| Host GPIO | ESP8266 Pin | Wire Color | Function |
| :--- | :--- | :--- | :--- |
| **GPIO 14** (TX) | **RX** | 🟢 Green | UART Data |
| **GPIO 15** (RX) | **TX** | 🟠 Orange | UART Data |
| **GND** | **GND** | ⚪ White | Common Ground |
| **GPIO 23** | **RESET** | 🔴 Red | Auto-reset sequencing |
| **GPIO 24** | **GPIO0 (BOOT)** | Black | Bootloader mode selection |

---

### ⚡ Flashing & Automated Testing
The repository includes `flasher_tester.py`, which automates entering the bootloader, flashing the ESP8266 via `esptool.py`, and executing functional/stress tests over UART. 

You can use this script to verify the stock firmware, test your own custom builds, or take it as a foundation to build your own hardware QA pipeline.

> ⚠️ **Important:** The compiled firmware file from the GitHub releases (`firmware.bin`) **must** be placed in the exact same directory as `flasher_tester.py` before execution.

---

### 📡 UART Protocol (JSON API)
* **Baud Rate:** `115200` (`SERIAL_8N1`)
* **Format:** Newline-terminated JSON strings.
* **Base Requirement:** Every command must include `"device": "light_controller"`.
* **Responses:** Returns `OK: <payload>` on success or `ERROR: <reason>` on failure (accompanied by onboard LED visual feedback).

#### System Commands
// To download the bin file
sudo python3 flasher_tester.py firmware.bin --skip-test
```json
// Returns the board's ID
{"device":"light_controller", "get_tickvi_id":true}

// Returns the firmware version
{"device":"light_controller", "get_tickvi_version":true}

// Ping check (Returns "OK: tickvi_OK")
{"device":"light_controller", "tickvi_status":true}

// Assigns a new ID to the board (e.g., ID = 2)
{"device":"light_controller", "set_tickvi_id":2}

// Triggers the onboard LED matrix hardware test
{"device":"light_controller", "tickvi_test":true}
