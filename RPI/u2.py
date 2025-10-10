#!/usr/bin/env python3
import serial
import time
import subprocess
import os

def check_system():
    print("=== RPI4 UART Diagnostic ===")
    
    # Check board
    try:
        with open('/proc/device-tree/model', 'r') as f:
            model = f.read()
        print(f"Board: {model}")
    except:
        print("Cannot detect board model")
    
    # Check serial devices
    print("\n1. Checking serial devices:")
    os.system('ls -la /dev/tty* | grep -E "tty(AMA|S|USB)"')
    
    # Check config
    print("\n2. Checking /boot/config.txt:")
    os.system('grep -E "(enable_uart|dtoverlay|uart)" /boot/config.txt || echo "No UART settings found"')
    
    # Check services
    print("\n3. Checking serial services:")
    os.system('systemctl list-units | grep -i serial')

def test_gpio_with_gpiod():
    print("\n4. Testing GPIO 14 (TX pin) using gpiod:")
    
    try:
        # Install gpiod if not available
        subprocess.run(['sudo', 'apt', 'install', '-y', 'python3-libgpiod'], check=False)
        
        import gpiod
        
        # GPIO 14 is chip 0, line 14
        chip = gpiod.Chip('gpiochip0')
        line = chip.get_line(14)
        
        config = gpiod.line_request()
        config.consumer = "uart_test"
        config.request_type = gpiod.line_request.DIRECTION_OUTPUT
        
        line.request(config)
        
        print("Toggling GPIO 14 - check scope now!")
        for i in range(10):
            line.set_value(1)  # HIGH
            time.sleep(0.1)    # 100ms
            line.set_value(0)  # LOW  
            time.sleep(0.1)    # 100ms
            print(f"Toggle {i+1}")
            
        line.release()
        chip.close()
        
    except ImportError:
        print("gpiod not available, trying alternative...")
        test_gpio_with_sysfs()

def test_gpio_with_sysfs():
    print("\n5. Testing GPIO 14 using sysfs (fallback):")
    
    try:
        # Export GPIO
        with open('/sys/class/gpio/export', 'w') as f:
            f.write('14')
        time.sleep(0.1)
        
        # Set as output
        with open('/sys/class/gpio/gpio14/direction', 'w') as f:
            f.write('out')
        
        print("Toggling GPIO 14 via sysfs - check scope!")
        for i in range(10):
            with open('/sys/class/gpio/gpio14/value', 'w') as f:
                f.write('1')  # HIGH
            time.sleep(0.1)
            with open('/sys/class/gpio/gpio14/value', 'w') as f:
                f.write('0')  # LOW
            time.sleep(0.1)
            print(f"Toggle {i+1}")
            
        # Unexport
        with open('/sys/class/gpio/unexport', 'w') as f:
            f.write('14')
            
    except Exception as e:
        print(f"Sysfs error: {e}")

def test_uart_transmission():
    print("\n6. Testing UART transmission:")
    
    ports_to_try = ['/dev/ttyAMA0', '/dev/ttyS0', '/dev/serial0']
    
    for port in ports_to_try:
        print(f"\nTrying {port}:")
        try:
            ser = serial.Serial(
                port=port,
                baudrate=115200,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            
            print(f"✓ Port {port} opened")
            
            # Send test pattern
            test_data = b'U'  # 0x55 = alternating bits
            print(f"Sending test pattern on {port}...")
            
            for i in range(5):
                ser.write(test_data * 10)  # Send 10 bytes
                ser.flush()
                print(f"  Sent burst {i+1} - check scope!")
                time.sleep(0.5)
                
            ser.close()
            
        except Exception as e:
            print(f"✗ {port}: {e}")

if __name__ == "__main__":
    check_system()
    test_gpio_with_gpiod()
    test_uart_transmission()
