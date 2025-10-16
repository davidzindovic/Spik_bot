import serial
import time

ser=serial.Serial('/dev/ttyS0',115200,timeout=1.0)
time.sleep(3)
ser.reset_input_buffer()
print("Serial Receive ready")

data=[]

try:
  while len(data)<9:
    time.sleep(1)
    print("Receiving")
    data.append(pmd.read())
    print(data)
  pmd.reset_output_buffer()
  print("Buffer resset")
except KeyboardInterrupt:
  print("closing")
  ser.close()
