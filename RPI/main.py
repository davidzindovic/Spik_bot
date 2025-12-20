import lib_init
import spik_bot_serial

try:
  lib_init.library_setup()
  spik_bot_serial.serial_init()
  
  while(1):

    color_images = glob.glob("*barva.png")
    
    for color_img_path in sorted(color_images):
        img_number = color_img_path.replace("barva.png", "")
        depth_img_path = f"{img_number}globina.npy"
        if not os.path.exists(depth_img_path): continue
    
    poslano="X:35 Y:100"
    spik_bot_serial.serial_transmit(poslano)

except KeyboardInterrupt:
  spik_bot_serial.serial_close()
