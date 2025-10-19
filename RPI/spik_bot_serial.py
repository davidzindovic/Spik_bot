import serial
import time

ser=serial.Serial('/dev/ttyS0',115200,timeout=1.0)

def serial_init():
  global ser
  #ser=serial.Serial('/dev/ttyS0',115200,timeout=1.0)
  time.sleep(3)
  ser.reset_input_buffer()
  print("Serial OK")

def serial_transmit(message):
  global ser
  print("Sending: "+message)
  ser.write(message.encode('utf-8'))

def serial_receive():
  global ser
  print("Receiving message...")
  x=ser.readline()
  if len(x)>2:
    print("Received: "+x+"\n-----")
  else:
    print("Received message is invalid \n-----")
  ser.reset_output_buffer()
