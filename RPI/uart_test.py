#!/usr/bin/env python3
import serial
import time
import threading

class STM32Communicator:
    def __init__(self, port='/dev/ttyAMA0', baudrate=115200, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.running = False
        self.callback = None
        
    def connect(self):
        """Establish connection with STM32"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=self.timeout
            )
            print(f"Connected to {self.port} at {self.baudrate} baud")
            
            # Wait for startup message
            time.sleep(2)
            self.flush_buffer()
            
            # Test communication
            response = self.send_command("STATUS")
            print("Connection test:", response)
            
            return True
            
        except serial.SerialException as e:
            print(f"Error connecting to {self.port}: {e}")
            return False
    
    def flush_buffer(self):
        """Clear serial buffer"""
        if self.ser:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
    
    def send_command(self, command, wait_time=0.5):
        """Send command to STM32 and return response"""
        if not self.ser or not self.ser.is_open:
            print("Serial port not open")
            return None
        
        try:
            # Send command with newline
            full_command = command + '\r\n'
            self.ser.write(full_command.encode('utf-8'))
            self.ser.flush()
            
            # Wait for response
            time.sleep(wait_time)
            
            # Read available response
            response = ""
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    response += line + "\n"
            
            return response.strip() if response else "No response"
            
        except Exception as e:
            print(f"Error sending command: {e}")
            return None
    
    def start_listening(self, callback=None):
        """Start background thread to listen for incoming messages"""
        self.callback = callback
        self.running = True
        self.listener_thread = threading.Thread(target=self._listen_loop)
        self.listener_thread.daemon = True
        self.listener_thread.start()
        print("Started listening thread")
    
    def _listen_loop(self):
        """Background thread to listen for incoming data"""
        while self.running:
            if self.ser and self.ser.is_open:
                try:
                    if self.ser.in_waiting > 0:
                        data = self.ser.readline().decode('utf-8', errors='ignore').strip()
                        if data:
                            print(f"Received: {data}")
                            if self.callback:
                                self.callback(data)
                    time.sleep(0.1)
                except Exception as e:
                    print(f"Error in listen loop: {e}")
                    time.sleep(1)
            else:
                time.sleep(1)
    
    def disconnect(self):
        """Close serial connection"""
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        print("Disconnected from STM32")

def main():
    # Create communicator instance
    # Try different ports if needed: /dev/ttyAMA0, /dev/ttyS0, /dev/serial0
    stm32 = STM32Communicator(port='/dev/ttyAMA0', baudrate=115200)
    
    # Connect to STM32
    if not stm32.connect():
        return
    
    try:
        # Interactive command loop
        print("\nSTM32 UART Communication Test")
        print("Commands: STATUS, STOP, START [0-3], MOVE [motor] [position], RESET, QUIT")
        
        while True:
            command = input("\nEnter command: ").strip().upper()
            
            if command == 'QUIT':
                break
            elif command == 'HELP':
                print("Available commands:")
                print("STATUS - Get motor status")
                print("STOP - Stop all motors")
                print("START [0-3] - Start specific motor")
                print("MOVE [motor] [position] - Move motor to position")
                print("RESET - Reset all motors")
                print("QUIT - Exit program")
            else:
                response = stm32.send_command(command)
                print(f"Response: {response}")
                
    except KeyboardInterrupt:
        print("\nProgram interrupted")
    finally:
        stm32.disconnect()

def message_callback(data):
    """Callback for incoming messages from STM32"""
    print(f"Callback received: {data}")

if __name__ == "__main__":
    # For background listening, use this instead of main():
    # stm32 = STM32Communicator(port='/dev/ttyAMA0', baudrate=115200)
    # if stm32.connect():
    #     stm32.start_listening(callback=message_callback)
    #     # Your main application logic here
    #     time.sleep(60)  # Run for 60 seconds
    #     stm32.disconnect()
    
    main()
