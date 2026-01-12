import pyrealsense2 as rs
import numpy as np
import cv2
import os
import datetime

# --- Nastavitve ---
OUTPUT_DIR = "zajeti_podatki"
# Privzeto za D400 serijo, preverite specifikacije vaše kamere
WIDTH = 640 
HEIGHT = 480
FPS = 30
# ------------------

def initialize_camera():
    """Inicializira RealSense kamero in konfigurira pipelino."""
    pipeline = rs.pipeline()
    config = rs.config()

    # Omogočanje barvnega toka
    config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
    
    # Omogočanje globinskega toka
    # Če imate težave, poskusite z rs.format.z16
    config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS) 
    
    # Začetek pretakanja
    print("Zagon pretakanja RealSense...")
    profile = pipeline.start(config)

    # Pridobitev globinskega senzorja in njegovih parametrov
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print(f"Lestvica globine je: {depth_scale} metrov na enoto (z16).")

    # Poravnava globine na barvno sliko
    align_to = rs.stream.color
    align = rs.align(align_to)
    
    return pipeline, align, depth_scale

def save_frames(color_frame, depth_frame_data, depth_scale):
    """Shrani barvni okvir kot PNG in globinske podatke kot .npy."""
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # Imena datotek
    #color_filename = os.path.join(OUTPUT_DIR, f"{timestamp}_barva.png")
    #depth_raw_filename = os.path.join(OUTPUT_DIR, f"{timestamp}_globina_surova.npy")
    color_filename = os.path.join(OUTPUT_DIR, f"TRENUTNA_barva.png")
    depth_raw_filename = os.path.join(OUTPUT_DIR, f"TRENUTNA_globina_surova.npy")
    
    # depth_visual_filename = os.path.join(OUTPUT_DIR, f"{timestamp}_globina_vizualizacija.png") # Lahko dodate za vizualizacijo

    # Shranjevanje barvne slike
    cv2.imwrite(color_filename, color_frame)
    print(f"Barvna slika shranjena kot: {color_filename}")

    # Shranjevanje surovih podatkov globine (numpy array)
    # Shranite 16-bitne surove podatke, da ohranite natančnost.
    np.save(depth_raw_filename, depth_frame_data)
    print(f"Surovi globinski podatki shranjeni kot: {depth_raw_filename}")
    
    # Če želite globinsko sliko vidno (za pregled, ni natančen surov podatek)
    # depth_image = cv2.convertScaleAbs(depth_frame_data, alpha=0.03) # Vizualizacija
    # depth_colormap = cv2.applyColorMap(depth_image, cv2.COLORMAP_JET)
    # cv2.imwrite(depth_visual_filename, depth_colormap)
    # print(f"Vizualizacija globine shranjena kot: {depth_visual_filename}")


def exec():
    # Preverjanje/ustvarjanje izhodnega direktorija
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)
        print(f"Ustvarjen izhodni direktorij: {OUTPUT_DIR}")

    pipeline, align, depth_scale = initialize_camera()

    try:
        while True:
            # Počakaj na sklop usklajenih okvirjev (barva in globina)
            frames = pipeline.wait_for_frames()
            
            # Poravnava globine na barvni okvir
            aligned_frames = align.process(frames)
            
            # Pridobivanje posameznih okvirjev
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            # Pretvorba v numpy array
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Vizualizacija globine za prikaz
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            
            # Prikaz slike
            images = np.hstack((color_image, depth_colormap))
            cv2.imshow('RealSense Stream (Pritisni "s" za shranjevanje, "q" za izhod)', images)

            # Čakanje na pritisk tipke
            key = cv2.waitKey(1)
            
            if key == ord('s'):
                # SHRANJEVANJE PODATKOV
                print(">>> Zajem in shranjevanje podatkov...")
                save_frames(color_image, depth_image, depth_scale)
                print(">>> Shranjevanje končano.")
            
            elif key == ord('q'):
                break

    except Exception as e:
        print(f"Prišlo je do napake: {e}")
        
    finally:
        print("Ustavljanje pretakanja RealSense.")
        pipeline.stop()
        cv2.destroyAllWindows()
        print("Program končan.")

#if __name__ == "__main__":
#    main()
