import lib_init
import spik_bot_serial
import data_capture_and_process
import glob
import os

try:
  lib_init.library_setup()

  #odkomentiraj za test na hw:
  spik_bot_serial.serial_init()
  
  while(1):

    color_images = glob.glob("zajeti_podatki/*barva.png")
    
    for color_img_path in sorted(color_images):
        # Ločimo mapo in ime datoteke (npr. "zajeti_podatki/" in "1barva.png")
        directory, filename = os.path.split(color_img_path)
        
        # Zamenjamo del imena datoteke
        depth_filename = filename.replace("barva.png", "globina.npy")
        
        # Ponovno združimo v polno pot
        depth_img_path = os.path.join(directory, depth_filename)
        
        print(f"Barvna: {color_img_path} -> Globinska: {depth_img_path}")
    
        poslano=data_capture_and_process.exec(color_img_path,depth_img_path)
        spik_bot_serial.serial_transmit(poslano)

except KeyboardInterrupt:
  spik_bot_serial.serial_close()
