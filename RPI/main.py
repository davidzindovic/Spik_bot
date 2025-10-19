import lib_init.py
import spik_bot_serial.py

try:
  library_setup()
  serial_init()
  
  while(1):
    print(serial_receive())
