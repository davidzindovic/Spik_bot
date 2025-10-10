#!/usr/bin/env python3
import serial
import time
import sys

def debug_serial_communication(port='/dev/ttyS0', baudrate=115200):
    print(f"Testing communication on {port} at {baudrate} baud")
    print("=" * 50)
    
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=2
        )
        
        print("✓ Serial port opened successfully")
        
        # Flush buffers
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        print("✓ Buffers flushed")
        
        # Listen for any incoming data first (STM32 startup messages)
        print("Listening for STM32 startup messages...")
        time.sleep(2)
        
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            print(f"Received startup data: {data.decode('utf-8', errors='ignore')}")
        else:
            print("No startup messages received")
        
        # Test 1: Send simple text
        print("\n1. Testing simple text echo...")
        test_message = "Hello STM32!\r\n"
        print(f"Sending: {test_message.strip()}")
        ser.write(test_message.encode('utf-8'))
        ser.flush()
        
        time.sleep(1)
        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
            print(f"Response: {response}")
        else:
            print("No response to simple text")
        
        # Test 2: Send STATUS command
        print("\n2. Testing STATUS command...")
        ser.write(b"STATUS\r\n")
        ser.flush()
        
        time.sleep(1)
        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
            print(f"STATUS response: {response}")
        else:
            print("No response to STATUS command")
        
        # Test 3: Send single characters
        print("\n3. Testing character echo...")
        for char in "ABC":
            ser.write(char.encode('utf-8'))
            ser.flush()
            time.sleep(0.1)
            if ser.in_waiting > 0:
                echo = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                print(f"Sent '{char}', received: {echo}")
            else:
                print(f"Sent '{char}', no echo")
        
        ser.close()
        print("\n✓ Debug test completed")
        
    except Exception as e:
        print(f"✗ Error: {e}")
        print(f"Make sure:")
        print(f"- Port {port} exists")
        print(f"- STM32 is powered on")
        print(f"- Wiring is correct (TX->RX, RX->TX, GND->GND)")
        print(f"- No other program is using the serial port")

def check_serial_ports():
    print("Available serial ports:")
    try:
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        for port in ports:
            print(f"  {port.device} - {port.description}")
    except:
        print("  Could not list ports")

if __name__ == "__main__":
    check_serial_ports()
    print()
    
    # Try common ports
    ports_to_try = ['/dev/ttyS0']
    
    for port in ports_to_try:
        print(f"\nTrying {port}...")
        debug_serial_communication(port=port)
        time.sleep(1)
