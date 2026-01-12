import serial
import time

#za test na hw odkomentiraj
ser=serial.Serial('/dev/ttyS0',115200,timeout=1.0)

def serial_init():
  global ser
  #za test na hw zakomentiraj
  #ser=serial.Serial('/dev/ttyS0',115200,timeout=1.0)
  time.sleep(3)
  ser.reset_input_buffer()
  print("Serial OK")

# spremembe, potestiraj
def serial_transmit(message):
  global ser
  print("Sending: "+message)
  ser.write((message+"\n").encode('utf-8'))
  sporocilo=None
  while(sporocilo is None):
    sporocilo=serial_receive()
  if sporocilo==message:
    print("Uspesno!")
    ser.write(("OK"+"\n").encode('utf-8'))
  else:
    print("Neuspesno!")
    ser.write(("NOT OK"+"\n").encode('utf-8'))

def serial_receive():
  global ser
  #print("Receiving message...")
  x=ser.readline()
  if len(x)>2:
    print("Received: "+x.decode("utf-8")+"\n-----")
    return x.decode("utf-8")
  else:
    pass
    return None
    #print("Received message is invalid \n-----")
  ser.reset_output_buffer()


def serial_close():
  global ser
  print("Closing serial communication.")
  ser.close()
