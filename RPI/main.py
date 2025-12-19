import lib_init
import spik_bot_serial

try:
  lib_init.library_setup()
  spik_bot_serial.serial_init()
  
  while(1):
    poslano="X:35 Y:100"
    spik_bot_serial.serial_transmit(poslano)

except KeyboardInterrupt:
  spik_bot_serial.serial_close()
