import lib_init
import spik_bot_serial

try:
  lib_init.library_setup()
  spik_bot_serial.serial_init()
  
  while(1):
    print(spik_bot_serial.serial_receive())

except KeyboardInterrupt:
  print("closing")
  ser.close()
