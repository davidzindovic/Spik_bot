import lib_init
import spik_bot_serial

try:
  lib_init.library_setup()
  spik_bot_serial.serial_init()
  
  while(1):
    sporocilo=spik_bot_serial.serial_receive()
    if sporocilo is not None:
      spik_bot_serial.serial_transmit(sporocilo)

except KeyboardInterrupt:
  spik_bot_serial.serial_close()
