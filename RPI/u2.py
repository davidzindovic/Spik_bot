#!/usr/bin/env python3
import serial
import time
import RPi.GPIO as GPIO
import os

def check_system():
    print("=== RPI4 UART Diagnostic ===")
    
    # Check if we're on RPI
    try:
        with open('/proc/device-tree/model', 'r') as f:
            model = f.read()
        print(f"Board: {model}")
    except:
        print("Not running on RPI or cannot detect model")
    
    # Check serial devices
    print("\n1. Checking serial devices:")
    os.system('ls -la /dev/tty* | grep -E "tty(AMA|S)"')
    
    # Check config
    print("\n2. Checking /boot/config.txt:")
    os.system('grep -E "(enable_uart|dtoverlay|uart)" /boot/config.txt || echo "No UART settings found"')
    
    # Check cmdline.txt
    print("\n3. Checking /boot/cmdline.txt:")
    os.system('cat /boot/cmdline.txt')
    
    # Check services
    print("\n4. Checking serial services:")
    os.system('systemctl list-units | grep -i serial')

def test_gpio_direct():
    print("\n5. Testing GPIO 14 (TX pin) directly:")
    GPIO.setmode(GPIO.BCM)
    TX_PIN = 14
    GPIO.setup(TX_PIN, GPIO.OUT)
    
    print("Toggling GPIO 14 - check scope now!")
    for i in range(10):
        GPIO.output(TX_PIN, GPIO.HIGH)
        time.sleep(0.1)  # 100ms high
        GPIO.output(TX_PIN, GPIO.LOW)
        time.sleep(0.1)  # 100ms low
        print(f"Toggle {i+1}")
    
    GPIO.cleanup()

def test_uart_raw():
    print("\n6. Testing UART with direct file access:")
    try:
        # Try direct file write (bypass Python serial library)
        with open('/dev/ttyS0', 'wb') as ser:
            ser.write(b'TEST')
            ser.flush()
            print("Written to /dev/ttyS0")
    except Exception as e:
        print(f"Error writing to /dev/ttyS0: {e}")
    
    try:
        with open('/dev/ttyAMA0', 'wb') as ser:
            ser.write(b'TEST') 
            ser.flush()
            print("Written to /dev/ttyAMA0")
    except Exception as e:
        print(f"Error writing to /dev/ttyAMA0: {e}")

if __name__ == "__main__":
    check_system()
    test_gpio_direct()
    test_uart_raw()
