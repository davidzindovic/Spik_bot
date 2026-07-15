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
DEVELOPMENT = True  # Če je True, prikazuje živi stream in čaka na pritisk tipke 's'
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

# --- Nalaganje YOLO-World modela in prilagoditev za "tree trunk" ---
print("Nalagam YOLO-World model...")
# Naložimo model, ki podpira poljubne tekstovne poizvedbe v realnem času
model = YOLO("yolov8s-world.pt") 

# Nastavimo iskalni razred na "tree trunk" (drevesno deblo)
print("Prilagajam model za detekcijo: 'tree trunk'...")
model.set_classes(["tree trunk"])

def run_segmentation(color_image, depth_image):
    """Izvede YOLO-World segmentacijo za 'tree trunk' na podani sliki."""
    print("\nPoganjam YOLO-World segmentacijo (iskanje 'tree trunk')...")
    results = model.predict(source=color_image, conf=0.25, verbose=False)
    
    has_mask = False
    if len(results) > 0 and results[0].masks is not None:
        masks_data = results[0].masks.data
        if len(masks_data) > 0:
            # Vzamemo največjo zaznano masko debla
            largest_mask_idx = 0
            max_area = 0
            for idx, m in enumerate(masks_data):
                area = m.sum().item()
                if area > max_area:
                    max_area = area
                    largest_mask_idx = idx

            # Masko pretvorimo v ločljivost slike (1280x720) in bool format
            mask = masks_data[largest_mask_idx].cpu().numpy().astype(bool)
            mask = cv2.resize(mask.astype(np.uint8), (1280, 720)).astype(bool)
            has_mask = True

    if not has_mask:
        print("Deblo ('tree trunk') na sliki ni bilo zaznano.")
        return None

    # Globinski podatki znotraj maske debla
    masked_depth = np.where(mask, depth_image, 0)

    # Poiščemo minimalno globino (najbližjo točko debla)
    valid_depths = masked_depth[masked_depth > 0]

    if len(valid_depths) > 0:
        min_depth_raw = np.min(valid_depths)
        
        # Piksel koordinate (u, v) najbližje točke
        y_indices, x_indices = np.where(masked_depth == min_depth_raw)
        u, v = x_indices[0], y_indices[0]

        # Pretvorba globinske vrednosti v metre
        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        depth_in_meters = min_depth_raw * depth_scale

        # Deprojekcija 2D piksla v 3D prostor kamere
        rs_coords = rs.rs2_deproject_pixel_to_point(intrinsics, [u, v], depth_in_meters)
        
        # --- Prilagoditev koordinatnega sistema ---
        # Y je naravnost (Z_cam)
        # X+ je levo, X- je desno (-X_cam)
        X_coord = -rs_coords[0]
        Y_coord = rs_coords[2]

        # --- Izračun orientacije (O) ---
        contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            rect = cv2.minAreaRect(largest_contour)
            raw_angle = rect[2]
            
            # Normalizacija kota glede na vertikalno os debla
            width, height = rect[1]
            if width < height:
                calculated_angle = raw_angle
            else:
                calculated_angle = raw_angle - 90 if raw_angle > 0 else raw_angle + 90
            
            # Omejitev kota na interval [-30, 30] stopinj
            orientation = float(np.clip(calculated_angle, -30.0, 30.0))
        else:
            orientation = 0.0

        # --- Vizualizacija ---
        vis_img = color_image.copy()
        if contours:
            cv2.drawContours(vis_img, contours, -1, (0, 255, 0), 2)
        cv2.circle(vis_img, (u, v), 7, (0, 0, 255), -1)
        
        # Izpis izračunanih podatkov na zaslon
        text_x = f"X: {X_coord:.3f} m (L+/R-)"
        text_y = f"Y: {Y_coord:.3f} m (Naprej)"
        text_o = f"O: {orientation:.2f} deg ([-30, 30])"
        
        cv2.rectangle(vis_img, (10, 10), (320, 110), (0, 0, 0), -1)
        cv2.putText(vis_img, text_x, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(vis_img, text_y, (20, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(vis_img, text_o, (20, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # --- Pošiljanje podatkov na STM32 ---
        if ser and ser.is_open:
            cmd_x = f"X={X_coord:.3f}\r\n"
            cmd_y = f"Y={Y_coord:.3f}\r\n"
            cmd_o = f"O={orientation:.2f}\r\n"

            print(f"Pošiljam podatke na STM32:")
            print(f"  Koda: {cmd_x.strip()}")
            ser.write(cmd_x.encode())
            time.sleep(0.1)

            print(f"  Koda: {cmd_y.strip()}")
            ser.write(cmd_y.encode())
            time.sleep(0.1)

            print(f"  Koda: {cmd_o.strip()}")
            ser.write(cmd_o.encode())
            time.sleep(0.1)

            print("  Koda: GO")
            ser.write("GO\r\n".encode())
        else:
            print("Serijska vrata niso odprta. Izračunani podatki:")
            print(f"  X (levo/desno): {X_coord:.3f} m")
            print(f"  Y (naravnost): {Y_coord:.3f} m")
            print(f"  Orientacija (kot): {orientation:.2f}°")

        return vis_img
    else:
        print("Znotraj maske ni veljavnih globinskih podatkov.")
        return None

try:
    if DEVELOPMENT:
        print("\n=== DEVELOPMENT MODE ===")
        print("Tipka 's': Shrani trenutno sliko in zažene segmentacijo.")
        print("Tipka 'q': Prekini in zapri program.\n")

        cv2.namedWindow("RealSense - Live Stream", cv2.WINDOW_NORMAL)

        while True:
            # Zajemi sličice v realnem času
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            # Pretvorba v numpy polja
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            # Ustvari barvni prikaz globine (JET) za lažjo človeško orientacijo
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET
            )

            # Združimo sliki vodoravno (Live Color | Live Depth)
            live_view = np.hstack((color_image, depth_colormap))

            # Izris navodil na živi prenos
            cv2.putText(live_view, "LIVE COLOR", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(live_view, "LIVE DEPTH MAP (JET)", (1310, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(live_view, "Pritisni 's' za segmentacijo 'tree trunk' | 'q' za izhod", (30, 700), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            cv2.imshow("RealSense - Live Stream", live_view)

            # Čakamo na tipko
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break
            elif key == ord('s'):
                # Zaustavi live stream in sproži segmentacijo na trenutnem kadru
                vis_result = run_segmentation(color_image, depth_image)
                
                if vis_result is not None:
                    cv2.imshow("RealSense - Live Stream", vis_result)
                    print("\nPrikazan je rezultat modela. Pritisni poljubno tipko za vrnitev v live stream...")
                    cv2.waitKey(0)
                else:
                    print("\nSegmentacija ni uspela, vračam se v live stream...")

    else:
        # PRODUKCIJSKI NAČIN
        print("Zagon v produkcijskem načinu...")
        for _ in range(30):
            pipeline.wait_for_frames()

        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if color_frame and depth_frame:
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            run_segmentation(color_image, depth_image)

finally:
    # Varno zapiranje virov
    pipeline.stop()
    cv2.destroyAllWindows()
    if ser and ser.is_open:
        ser.close()
    print("Program zaključen.")
