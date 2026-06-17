#!/usr/bin/env python3
import sys
import subprocess
import time
import os
import json
import argparse
import serial
from gpiozero import DigitalOutputDevice

os.environ['GPIOZERO_PIN_FACTORY'] = 'lgpio'

RESET_PIN_NUM = 23
GPIO0_PIN_NUM = 24
UART_PORT = "/dev/ttyAMA0" 
FLASH_BAUDRATE = "460800"
TEST_BAUDRATE = 115200
ECHO_TIMEOUT_MS = 500
BASE_DELAY_SEC = 3.0
TIMER_TEST_DELAY_SEC = 12.0
RAPID_FIRE_DELAY_SEC = 0.1
STRESS_DELAY_SEC = 1.0

TESTS = [
    {"name": "1. H:mode1(inf) + S:ON(inf)", "h_act": True, "h_mode": 1, "h_dur": 0, "s_act": True, "s_mode": 1, "s_dur": 0, "expect_ok": True, "is_timer_test": False},
    {"name": "2. H:mode2(inf) + S:ON(inf)", "h_act": True, "h_mode": 2, "h_dur": 0, "s_act": True, "s_mode": 1, "s_dur": 0, "expect_ok": True, "is_timer_test": False},
    {"name": "3. H:mode3(inf) + S:ON(inf)", "h_act": True, "h_mode": 3, "h_dur": 0, "s_act": True, "s_mode": 1, "s_dur": 0, "expect_ok": True, "is_timer_test": False},
    {"name": "4. STARS only: H:OFF + S:ON(inf)", "h_act": False, "h_mode": 0, "h_dur": 0, "s_act": True, "s_mode": 1, "s_dur": 0, "expect_ok": True, "is_timer_test": False},
    {"name": "5. HANDS only: H:mode1(inf) + S:OFF", "h_act": True, "h_mode": 1, "h_dur": 0, "s_act": False, "s_mode": 0, "s_dur": 0, "expect_ok": True, "is_timer_test": False},
    {"name": "6. [TIMER] H:mode3(5s) + S:ON(inf)", "h_act": True, "h_mode": 3, "h_dur": 5, "s_act": True, "s_mode": 1, "s_dur": 0, "expect_ok": True, "is_timer_test": True},
    {"name": "7. [TIMER] H:mode1(3s) + S:OFF", "h_act": True, "h_mode": 1, "h_dur": 3, "s_act": False, "s_mode": 0, "s_dur": 0, "expect_ok": True, "is_timer_test": True},
    {"name": "8. [TIMER] H:OFF + S:ON(4s)", "h_act": False, "h_mode": 0, "h_dur": 0, "s_act": True, "s_mode": 1, "s_dur": 4, "expect_ok": True, "is_timer_test": True},
    {"name": "9. [TIMER] Both with timers", "h_act": True, "h_mode": 2, "h_dur": 3, "s_act": True, "s_mode": 1, "s_dur": 2, "expect_ok": True, "is_timer_test": True},
    {"name": "10. ALL OFF (Reset)", "h_act": False, "h_mode": 0, "h_dur": 0, "s_act": False, "s_mode": 0, "s_dur": 0, "expect_ok": True, "is_timer_test": False},
    {"name": "11. [ERROR] H:invalid_mode(9)", "h_act": True, "h_mode": 9, "h_dur": 0, "s_act": False, "s_mode": 0, "s_dur": 0, "expect_ok": False, "is_timer_test": False},
    {"name": "12. [IGNORE] H:OFF with garbage", "h_act": False, "h_mode": 5, "h_dur": 99, "s_act": False, "s_mode": 0, "s_dur": 0, "expect_ok": True, "is_timer_test": False},
    {"name": "13. [ERROR] Wrong device", "device": "wrong_device", "h_act": True, "h_mode": 1, "h_dur": 0, "s_act": True, "s_mode": 1, "s_dur": 0, "expect_ok": False, "is_timer_test": False},
    {"name": "14. PING (Quick Check)", "h_act": True, "h_mode": 1, "h_dur": 1, "s_act": True, "s_mode": 1, "s_dur": 1, "expect_ok": True, "is_timer_test": True}
]

STRESS_TESTS = [
    {"name": "S1. Rapid Fire #1", "h_act": True, "h_mode": 1, "h_dur": 0, "s_act": True, "s_mode": 1, "s_dur": 0, "expect_ok": True, "delay": RAPID_FIRE_DELAY_SEC},
    {"name": "S2. Rapid Fire #2", "h_act": True, "h_mode": 1, "h_dur": 0, "s_act": True, "s_mode": 1, "s_dur": 0, "expect_ok": True, "delay": RAPID_FIRE_DELAY_SEC},
    {"name": "S3. Rapid Fire #3", "h_act": True, "h_mode": 1, "h_dur": 0, "s_act": True, "s_mode": 1, "s_dur": 0, "expect_ok": True, "delay": RAPID_FIRE_DELAY_SEC},
    {"name": "S4. Rapid Fire #4", "h_act": True, "h_mode": 1, "h_dur": 0, "s_act": True, "s_mode": 1, "s_dur": 0, "expect_ok": True, "delay": RAPID_FIRE_DELAY_SEC},
    {"name": "S5. Rapid Fire #5", "h_act": True, "h_mode": 1, "h_dur": 0, "s_act": True, "s_mode": 1, "s_dur": 0, "expect_ok": True, "delay": STRESS_DELAY_SEC},
    {"name": "S6. H:mode1 -> H:mode2", "h_act": True, "h_mode": 1, "h_dur": 0, "s_act": False, "s_mode": 0, "s_dur": 0, "expect_ok": True, "delay": STRESS_DELAY_SEC},
    {"name": "S7. H:mode2 -> H:mode3", "h_act": True, "h_mode": 2, "h_dur": 0, "s_act": False, "s_mode": 0, "s_dur": 0, "expect_ok": True, "delay": STRESS_DELAY_SEC},
    {"name": "S8. H:mode3 -> H:mode1", "h_act": True, "h_mode": 3, "h_dur": 0, "s_act": False, "s_mode": 0, "s_dur": 0, "expect_ok": True, "delay": STRESS_DELAY_SEC},
    {"name": "S9. Timer 10s -> Override 1s", "h_act": True, "h_mode": 1, "h_dur": 10, "s_act": False, "s_mode": 0, "s_dur": 0, "expect_ok": True, "delay": STRESS_DELAY_SEC},
    {"name": "S10. Override with 1s timer", "h_act": True, "h_mode": 2, "h_dur": 1, "s_act": False, "s_mode": 0, "s_dur": 0, "expect_ok": True, "delay": 3.0},
    {"name": "S11. Set 2s timer", "h_act": True, "h_mode": 3, "h_dur": 2, "s_act": False, "s_mode": 0, "s_dur": 0, "expect_ok": True, "delay": 1.5},
    {"name": "S12. Override at 1.5s mark", "h_act": True, "h_mode": 1, "h_dur": 0, "s_act": False, "s_mode": 0, "s_dur": 0, "expect_ok": True, "delay": STRESS_DELAY_SEC},
    {"name": "S13. Toggle ON", "h_act": True, "h_mode": 1, "h_dur": 0, "s_act": True, "s_mode": 1, "s_dur": 0, "expect_ok": True, "delay": RAPID_FIRE_DELAY_SEC},
    {"name": "S14. Toggle OFF", "h_act": False, "h_mode": 0, "h_dur": 0, "s_act": False, "s_mode": 0, "s_dur": 0, "expect_ok": True, "delay": RAPID_FIRE_DELAY_SEC},
    {"name": "S15. Toggle ON again", "h_act": True, "h_mode": 2, "h_dur": 0, "s_act": True, "s_mode": 1, "s_dur": 0, "expect_ok": True, "delay": RAPID_FIRE_DELAY_SEC},
    {"name": "S16. Toggle OFF again", "h_act": False, "h_mode": 0, "h_dur": 0, "s_act": False, "s_mode": 0, "s_dur": 0, "expect_ok": True, "delay": STRESS_DELAY_SEC},
    {"name": "S17. Max duration (65535s)", "h_act": True, "h_mode": 1, "h_dur": 65535, "s_act": True, "s_mode": 1, "s_dur": 65535, "expect_ok": True, "delay": STRESS_DELAY_SEC},
    {"name": "S18. Reset max duration", "h_act": False, "h_mode": 0, "h_dur": 0, "s_act": False, "s_mode": 0, "s_dur": 0, "expect_ok": True, "delay": STRESS_DELAY_SEC},
    {"name": "S19. Short timer (1s)", "h_act": True, "h_mode": 3, "h_dur": 1, "s_act": False, "s_mode": 0, "s_dur": 0, "expect_ok": True, "delay": 2.0},
    {"name": "S20. Re-enable same mode", "h_act": True, "h_mode": 3, "h_dur": 0, "s_act": False, "s_mode": 0, "s_dur": 0, "expect_ok": True, "delay": STRESS_DELAY_SEC},
    {"name": "S21. ALL OFF #1", "h_act": False, "h_mode": 0, "h_dur": 0, "s_act": False, "s_mode": 0, "s_dur": 0, "expect_ok": True, "delay": RAPID_FIRE_DELAY_SEC},
    {"name": "S22. ALL OFF #2", "h_act": False, "h_mode": 0, "h_dur": 0, "s_act": False, "s_mode": 0, "s_dur": 0, "expect_ok": True, "delay": RAPID_FIRE_DELAY_SEC},
    {"name": "S23. ALL OFF #3", "h_act": False, "h_mode": 0, "h_dur": 0, "s_act": False, "s_mode": 0, "s_dur": 0, "expect_ok": True, "delay": RAPID_FIRE_DELAY_SEC},
    {"name": "S24. ALL OFF #4", "h_act": False, "h_mode": 0, "h_dur": 0, "s_act": False, "s_mode": 0, "s_dur": 0, "expect_ok": True, "delay": STRESS_DELAY_SEC},
    {"name": "S25. Valid command", "h_act": True, "h_mode": 1, "h_dur": 0, "s_act": False, "s_mode": 0, "s_dur": 0, "expect_ok": True, "delay": RAPID_FIRE_DELAY_SEC},
    {"name": "S26. Invalid mode (should fail)", "h_act": True, "h_mode": 99, "h_dur": 0, "s_act": False, "s_mode": 0, "s_dur": 0, "expect_ok": False, "delay": RAPID_FIRE_DELAY_SEC},
    {"name": "S27. Valid after invalid", "h_act": True, "h_mode": 2, "h_dur": 0, "s_act": False, "s_mode": 0, "s_dur": 0, "expect_ok": True, "delay": RAPID_FIRE_DELAY_SEC},
    {"name": "S28. Wrong device (should fail)", "device": "wrong_device", "h_act": True, "h_mode": 1, "h_dur": 0, "s_act": False, "s_mode": 0, "s_dur": 0, "expect_ok": False, "delay": RAPID_FIRE_DELAY_SEC},
    {"name": "S29. Valid after wrong device", "h_act": True, "h_mode": 3, "h_dur": 0, "s_act": False, "s_mode": 0, "s_dur": 0, "expect_ok": True, "delay": STRESS_DELAY_SEC},
    {"name": "S30. Final ALL OFF", "h_act": False, "h_mode": 0, "h_dur": 0, "s_act": False, "s_mode": 0, "s_dur": 0, "expect_ok": True, "delay": STRESS_DELAY_SEC}
]

def build_zone_command(test_case):
    device = test_case.get("device", "light_controller")
    return {
        "device": device,
        "zones": {
            "hands": {"action": test_case["h_act"], "mode": test_case["h_mode"], "duration_sec": test_case["h_dur"]},
            "stars": {"action": test_case["s_act"], "mode": test_case["s_mode"], "duration_sec": test_case["s_dur"]}
        }
    }

def send_and_wait(ser, test_case, timeout_ms=ECHO_TIMEOUT_MS):
    ser.reset_input_buffer()
    time.sleep(0.05)
    payload = build_zone_command(test_case)
    json_str = json.dumps(payload) + "\n"
    ser.write(json_str.encode('utf-8'))
    ser.flush()
    start_time = time.time()
    current_line = ""
    final_response = ""
    while (time.time() - start_time) * 1000 < timeout_ms:
        if ser.in_waiting > 0:
            char = ser.read(1).decode('utf-8', errors='ignore')
            if char in ('\n', '\r'):
                line = current_line.strip()
                if line:
                    if line.startswith("OK") or line.startswith("ERROR"):
                        final_response = line
                        break
                current_line = ""
            else:
                current_line += char
        else:
            time.sleep(0.01)
    if not final_response:
        final_response = current_line.strip()
    if test_case["expect_ok"]:
        return final_response.startswith("OK"), final_response
    else:
        return not final_response.startswith("OK"), final_response

def run_tests():
    time.sleep(2)
    try:
        ser = serial.Serial(UART_PORT, TEST_BAUDRATE, timeout=1)
        ser.reset_input_buffer()
    except serial.SerialException as e:
        print(f"Error opening port {UART_PORT}: {e}")
        return False
    passed = 0
    failed = 0
    for i, t in enumerate(TESTS):
        ack_received, response = send_and_wait(ser, t)
        if ack_received:
            passed += 1
        else:
            failed += 1
        resp_display = response[:15] + "..." if len(response) > 15 else response
        print(f"Test {i+1}: {t['name']} - {'PASS' if ack_received else 'FAIL'} ({resp_display})")
        time.sleep(TIMER_TEST_DELAY_SEC if t["is_timer_test"] else BASE_DELAY_SEC)
    ser.close()
    print(f"Result: Passed {passed}/{len(TESTS)}, Failed: {failed}")
    return failed == 0

def run_stress_tests():
    time.sleep(2)
    try:
        ser = serial.Serial(UART_PORT, TEST_BAUDRATE, timeout=1)
        ser.reset_input_buffer()
        time.sleep(0.5)
    except serial.SerialException as e:
        print(f"Error opening port {UART_PORT}: {e}")
        return False
    passed = 0
    failed = 0
    for i, t in enumerate(STRESS_TESTS):
        ack_received, response = send_and_wait(ser, t)
        if ack_received:
            passed += 1
        else:
            failed += 1
        resp_display = response[:15] + "..." if len(response) > 15 else response
        print(f"Stress {i+1}: {t['name']} - {'PASS' if ack_received else 'FAIL'} ({resp_display})")
        time.sleep(t.get("delay", STRESS_DELAY_SEC))
    ser.close()
    print(f"Result: Passed {passed}/{len(STRESS_TESTS)}, Failed: {failed}")
    return failed == 0

def enter_bootloader(reset_pin, gpio0_pin):
    gpio0_pin.off()
    reset_pin.off()
    time.sleep(0.1)
    reset_pin.on()
    time.sleep(0.5)

def exit_bootloader(reset_pin, gpio0_pin):
    gpio0_pin.on()
    reset_pin.off()
    time.sleep(0.1)
    reset_pin.on()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("firmware", nargs="?")
    parser.add_argument("--test-only", action="store_true")
    parser.add_argument("--skip-test", action="store_true")
    parser.add_argument("--stress", action="store_true")
    parser.add_argument("--stress-only", action="store_true")
    args = parser.parse_args()
    
    if not args.test_only and not args.stress_only and not args.firmware:
        print("Error: Provide firmware file or use --test-only flag")
        sys.exit(1)
        
    reset_pin = DigitalOutputDevice(RESET_PIN_NUM, active_high=True, initial_value=True)
    gpio0_pin = DigitalOutputDevice(GPIO0_PIN_NUM, active_high=True, initial_value=True)
    flash_success = True
    
    if not args.test_only and not args.stress_only:
        enter_bootloader(reset_pin, gpio0_pin)
        cmd = [sys.executable, "-m", "esptool", "--chip", "esp8266", "--port", UART_PORT, "--baud", FLASH_BAUDRATE, "--before", "no-reset", "--after", "no-reset", "write-flash", "-z", "--flash-size", "detect", "0x00000", args.firmware]
        try:
            result = subprocess.run(cmd)
            if result.returncode != 0:
                flash_success = False
        except Exception:
            flash_success = False
        finally:
            exit_bootloader(reset_pin, gpio0_pin)
            
    if not args.skip_test and flash_success:
        test_success = True
        if not args.stress_only:
            if not run_tests():
                test_success = False
        if args.stress or args.stress_only:
            if not run_stress_tests():
                test_success = False
                
    reset_pin.close()
    gpio0_pin.close()

if __name__ == "__main__":
    main()