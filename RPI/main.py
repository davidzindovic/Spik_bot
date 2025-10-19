import lib_init
import spik_bot_serial

try:
  library_setup()
  serial_init()
  
  while(1):
    print(serial_receive())

except KeyboardInterrupt:
  print("closing")
  ser.close()
