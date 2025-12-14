import lib_init
import spik_bot_serial

try:
  lib_init.library_setup()
  spik_bot_serial.serial_init()
  
  while(1):
    poslano="X:35 Y:100"
    spik_bot_serial.serial_transmit(poslano)
    sporocilo=None
    while(sporocilo is None):
      sporocilo=spik_bot_serial.serial_receive()
    print("Prejel sem sporočilo: ",sporocilo)
    if sporocilo==poslano:
      print("Uspesno!")
      spik_bot_serial.serial_transmit("OK")
      while(1):
        pass
    else:
      print("Neuspesno!")
      spik_bot_serial.serial_transmit("NOT OK")
      

except KeyboardInterrupt:
  spik_bot_serial.serial_close()
