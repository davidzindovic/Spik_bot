import serial
import time

ser=serial.Serial('/dev/ttyS0',115200,timeout=1.0)
time.sleep(3)
ser.reset_input_buffer()
print("Serial OK")

try:
  while True:
    time.sleep(1)
    print("Sending")
    ser.write("Hello\n".encode('utf-8'))
except KeyboardInterrupt:
  print("closing")
  ser.close()
