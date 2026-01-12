import lib_init
import spik_bot_serial
import data_capture_and_process
import capture_and_save_images

import glob
import os
import cv2
import numpy as np

def pridobi_potrditev_vboda_cv2():
    # Ustvarimo črno sliko (višina 800, širina 1600)
    okno_y=800
    okno_x=1600
    img = np.zeros((okno_y, okno_x, 3), dtype=np.uint8)
    okno_ime = "Potrditev"
    
    # Nastavitve besedila
    font = cv2.FONT_HERSHEY_SIMPLEX
    barva = (255, 255, 255) # Bela
    
    # Izris besedila na sliko
    cv2.putText(img, "Tocka za vbod dosezena.", (int(okno_x*0.3), 300), font, 2, barva, 5)
    cv2.putText(img, "Ali izvedem vbod?", (int(okno_x*0.3)+50, 400), font, 2, barva, 5)
    cv2.putText(img, "1=DA , 2=NE", (int(okno_x*0.3)+100, 500), font, 2, (0, 255, 0), 5)
    
    rezultat = None
    
    while True:
        cv2.imshow(okno_ime, img)
        
        # Čakamo na pritisk tipke (0 pomeni neskončno čakanje)
        kljuc = cv2.waitKey(0) & 0xFF
        
        if kljuc == ord('1'):
            rezultat = 1
            break
        elif kljuc == ord('2'):
            rezultat = 2
            break
        # Če je pritisnjena katera koli druga tipka, zanka teče naprej
            
    cv2.destroyWindow(okno_ime)
    # Dodamo kratek zamik, da se okno zagotovo zapre v nekaterih okoljih
    cv2.waitKey(1)
    
    return rezultat

def za_vse():
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
        #spik_bot_serial.serial_transmit(poslano)

try:
  lib_init.library_setup()

  #odkomentiraj za test na hw:
  spik_bot_serial.serial_init()

  capture_and_save_images.exec()

  poslano=data_capture_and_process.exec("/home/lr/Desktop/Spik_bot-main/RPI/zajeti_podatki/TRENUTNA_barva.png","/home/lr/Desktop/Spik_bot-main/RPI/zajeti_podatki/TRENUTNA_globina_surova.npy")
  spik_bot_serial.serial_transmit(poslano)

  res=pridobi_potrditev_vboda_cv2()
  spik_bot_serial.serial_transmit(str(res))

except KeyboardInterrupt:
  pass
  spik_bot_serial.serial_close()

