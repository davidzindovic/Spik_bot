import os
import sys
import time

# --- Samodejna namestitev in uvoz knjižnic ---
try:
    import pyrealsense2 as rs
except ImportError:
    os.system('python -m pip install pyrealsense2')
finally:
    import pyrealsense2 as rs

try:
    import numpy as np
except ImportError:
    os.system('python -m pip install numpy')
finally:
    import numpy as np

try:
    import cv2
except ImportError:
    os.system('python -m pip install opencv-python')
finally:
    import cv2

try:
    import serial
except ImportError:
    os.system('python -m pip install pyserial')
finally:
    import serial

try:
    from ultralytics import YOLO
except ImportError:
    os.system('python -m pip install ultralytics')
finally:
    from ultralytics import YOLO

# ==============================================================================
# NASTAVITVE
# ==============================================================================
DEVELOPMENT = True  # Nastavi na False za produkcijski zagon brez vizualizacije
SERIAL_PORT = 'COM3' 
BAUD_RATE = 115200
# ==============================================================================

# --- Nastavitve serijske komunikacije ---
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Povezava uspešno vzpostavljena na {SERIAL_PORT}")
except Exception as e:
    print(f"Napaka pri povezovanju na serijska vrata: {e}")
    ser = None

# --- Nastavitve RealSense Kamere ---
pipeline = rs.pipeline()
config = rs.config()

# Nastavitev barvnega in globinskega toka (D455)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

# Začetek zajemanja
profile = pipeline.start(config)

# Poravnava globinske slike na barvno sliko (bistveno za ujemanje pikslov)
align_to = rs.stream.color
align = rs.align(align_to)

# Pridobitev intrinzičnih parametrov kamere (za pretvorbo pikslov v metre)
intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

# --- Nalaganje YOLO segmentacijskega modela ---
print("Nalagam YOLO11 segmentacijski model...")
# Uporabimo lahek segmentacijski model za izjemno hitro delovanje
model = YOLO("yolo11s-seg.pt") 

try:
    # Pustimo kameri 30 sličic, da se prilagodi svetlobi (Auto-Exposure)
    for _ in range(30):
        pipeline.wait_for_frames()

    print("Zajemam sliko...")
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    
    color_frame = aligned_frames.get_color_frame()
    depth_frame = aligned_frames.get_depth_frame()

    if not color_frame or not depth_frame:
        raise RuntimeError("Ni mogoče pridobiti podatkov iz kamere.")

    # Pretvorba v numpy polja
    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())

    # --- Predikcija z YOLO ---
    print("Poganjam YOLO segmentacijo...")
    results = model.predict(source=color_image, conf=0.25, verbose=False)
    
    has_mask = False
    if len(results) > 0 and results[0].masks is not None:
        masks_data = results[0].masks.data
        if len(masks_data) > 0:
            # Izberemo največjo zaznano masko na sliki (predvidevamo, da je to deblo)
            largest_mask_idx = 0
            max_area = 0
            for idx, m in enumerate(masks_data):
                area = m.sum().item()
                if area > max_area:
                    max_area = area
                    largest_mask_idx = idx

            # Masko pretvorimo v primerno ločljivost slike (1280x720) in bool format
            mask = masks_data[largest_mask_idx].cpu().numpy().astype(bool)
            mask = cv2.resize(mask.astype(np.uint8), (1280, 720)).astype(bool)
            has_mask = True

    if not has_mask:
        print("Na sliki ni bil zaznan noben primeren objekt za segmentacijo.")
    else:
        # Globinski podatki znotraj maske (izven maske nastavimo globino na 0)
        masked_depth = np.where(mask, depth_image, 0)

        # Poiščemo minimalno globino, ki je večja od 0 (najbližja točka debla)
        valid_depths = masked_depth[masked_depth > 0]

        if len(valid_depths) > 0:
            min_depth_raw = np.min(valid_depths)
            
            # Najdemo piksel koordinate (u, v) te najbližje točke
            y_indices, x_indices = np.where(masked_depth == min_depth_raw)
            u, v = x_indices[0], y_indices[0]  # Vzamemo prvo najdeno točko z min globino

            # Pretvorba globinske vrednosti v metre (upoštevamo merilo kamere)
            depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
            depth_in_meters = min_depth_raw * depth_scale

            # Deprojekcija 2D piksla v 3D prostor (referenčni sistem kamere)
            rs_coords = rs.rs2_deproject_pixel_to_point(intrinsics, [u, v], depth_in_meters)
            
            # --- Prilagoditev koordinatnega sistema po navodilih ---
            # Y je naravnost od kamere (RealSense Z_cam)
            # X+ je levo od kamere, X- je desno (negiramo RealSense X_cam)
            X_coord = -rs_coords[0]
            Y_coord = rs_coords[2]

            # --- Izračun orientacije (O) ---
            contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                rect = cv2.minAreaRect(largest_contour)
                raw_angle = rect[2]
                
                # Normalizacija kota, da dobimo odklon od navpičnice (0 stopinj)
                width, height = rect[1]
                if width < height:
                    calculated_angle = raw_angle
                else:
                    calculated_angle = raw_angle - 90 if raw_angle > 0 else raw_angle + 90
                
                # Omejitev kota na interval [-30, 30] stopinj
                orientation = float(np.clip(calculated_angle, -30.0, 30.0))
            else:
                orientation = 0.0

            # --- Vizualizacija (DEVELOPMENT flag) ---
            if DEVELOPMENT:
                # Ustvarimo kopijo barvne slike za risanje vizualnih elementov
                vis_img = color_image.copy()
                
                # Narišemo konturo zaznanega debla (zelena barva)
                cv2.drawContours(vis_img, contours, -1, (0, 255, 0), 2)
                
                # Označimo točko vboda (rdeč poln krog)
                cv2.circle(vis_img, (u, v), 7, (0, 0, 255), -1)
                
                # Izris izračunanih podatkov v zgornji levi kot slike
                text_x = f"X: {X_coord:.3f} m (L+/R-)"
                text_y = f"Y: {Y_coord:.3f} m (Naprej)"
                text_o = f"O: {orientation:.2f} deg ([-30, 30])"
                
                # Podlaga za besedilo, da bo lažje berljivo
                cv2.rectangle(vis_img, (10, 10), (320, 110), (0, 0, 0), -1)
                
                cv2.putText(vis_img, text_x, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(vis_img, text_y, (20, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(vis_img, text_o, (20, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                # Prikaz slike v oknu
                cv2.imshow("Development View - YOLO11 & Vbodna tocka", vis_img)
                print("\nPrikazujem sliko. Pritisni poljubno tipko na sliki za nadaljevanje...")
                cv2.waitKey(0)
                cv2.destroyAllWindows()

            # --- Pošiljanje podatkov preko serijske povezave ---
            if ser and ser.is_open:
                # Formatiranje podatkov na 3 oz. 2 decimalni mesti
                cmd_x = f"X={X_coord:.3f}\r\n"
                cmd_y = f"Y={Y_coord:.3f}\r\n"
                cmd_o = f"O={orientation:.2f}\r\n"

                print(f"\nPošiljam podatke na STM32:")
                print(f"Koda: {cmd_x.strip()}")
                ser.write(cmd_x.encode())
                time.sleep(0.1)  # Kratek premor med ukazi za STM32

                print(f"Koda: {cmd_y.strip()}")
                ser.write(cmd_y.encode())
                time.sleep(0.1)

                print(f"Koda: {cmd_o.strip()}")
                ser.write(cmd_o.encode())
                time.sleep(0.1)

                print("Koda: GO")
                ser.write("GO\r\n".encode())
            else:
                print("\nSerijska vrata niso odprta. Izpis izračunanih podatkov:")
                print(f"X (levo/desno): {X_coord:.3f} m")
                print(f"Y (naravnost): {Y_coord:.3f} m")
                print(f"Orientacija (kot): {orientation:.2f}°")

        else:
            print("Znotraj maske ni veljavnih globinskih podatkov (morda preblizu/izven območja senzorja).")

finally:
    # Varno zapiranje virov
    pipeline.stop()
    if ser and ser.is_open:
        ser.close()
    print("Program zaključen.")
