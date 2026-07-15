import os
import sys
import time

# --- Samodejna namestitev in uvoz knjižnic ---
try:
    import pyrealsense2 as rs
except ImportError:
    print("[DEBUG] Nameščam pyrealsense2...")
    os.system('python -m pip install pyrealsense2')
finally:
    import pyrealsense2 as rs

try:
    import numpy as np
except ImportError:
    print("[DEBUG] Nameščam numpy...")
    os.system('python -m pip install numpy')
finally:
    import numpy as np

try:
    import cv2
except ImportError:
    print("[DEBUG] Nameščam opencv-python...")
    os.system('python -m pip install opencv-python')
finally:
    import cv2

try:
    import serial
except ImportError:
    print("[DEBUG] Nameščam pyserial...")
    os.system('python -m pip install pyserial')
finally:
    import serial

try:
    from ultralytics import YOLO
except ImportError:
    print("[DEBUG] Nameščam ultralytics...")
    os.system('python -m pip install ultralytics')
finally:
    from ultralytics import YOLO

# ==============================================================================
# NASTAVITVE
# ==============================================================================
DEVELOPMENT = True  # NUJNO: Nastavi na True za živi stream in prikaz okna!
SERIAL_PORT = 'COM3' 
BAUD_RATE = 115200
# ==============================================================================

print(f"[DEBUG] Zagon programa. Način DEVELOPMENT = {DEVELOPMENT}")

# --- Nastavitve serijske komunikacije ---
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"[DEBUG] Povezava uspešno vzpostavljena na {SERIAL_PORT}")
except Exception as e:
    print(f"[DEBUG] Opozorilo pri povezovanju na serijska vrata: {e}")
    ser = None

# --- Nastavitve RealSense Kamere ---
try:
    print("[DEBUG] Inicializacija RealSense kamere...")
    pipeline = rs.pipeline()
    config = rs.config()

    # Nastavitev barvnega in globinskega toka (D455)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

    # Začetek zajemanja
    profile = pipeline.start(config)
    align_to = rs.stream.color
    align = rs.align(align_to)
    intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    print("[DEBUG] Kamera uspešno inicializirana.")
except Exception as e:
    print(f"[DEBUG] NAPAKA pri inicializaciji kamere: {e}")
    sys.exit(1)

# --- Nalaganje YOLO-World modela ---
try:
    print("[DEBUG] Nalagam YOLO-World model (yolov8s-world.pt)...")
    model = YOLO("yolov8s-world.pt") 
    print("[DEBUG] Nastavljam razred na 'tree trunk'...")
    model.set_classes(["tree trunk"])
    print("[DEBUG] Model je pripravljen.")
except Exception as e:
    print(f"[DEBUG] NAPAKA pri nalaganju modela: {e}")
    sys.exit(1)

def run_segmentation(color_image, depth_image):
    """Izvede segmentacijo za 'tree trunk' na podani sliki."""
    print("\n[DEBUG] Poganjam segmentacijo (iskanje 'tree trunk')...")
    results = model.predict(source=color_image, conf=0.25, verbose=False)
    
    has_mask = False
    if len(results) > 0 and results[0].masks is not None:
        masks_data = results[0].masks.data
        if len(masks_data) > 0:
            largest_mask_idx = 0
            max_area = 0
            for idx, m in enumerate(masks_data):
                area = m.sum().item()
                if area > max_area:
                    max_area = area
                    largest_mask_idx = idx

            mask = masks_data[largest_mask_idx].cpu().numpy().astype(bool)
            mask = cv2.resize(mask.astype(np.uint8), (1280, 720)).astype(bool)
            has_mask = True

    if not has_mask:
        print("[DEBUG] Deblo ('tree trunk') na sliki ni bilo zaznano.")
        return None

    masked_depth = np.where(mask, depth_image, 0)
    valid_depths = masked_depth[masked_depth > 0]

    if len(valid_depths) > 0:
        min_depth_raw = np.min(valid_depths)
        y_indices, x_indices = np.where(masked_depth == min_depth_raw)
        u, v = x_indices[0], y_indices[0]

        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        depth_in_meters = min_depth_raw * depth_scale

        rs_coords = rs.rs2_deproject_pixel_to_point(intrinsics, [u, v], depth_in_meters)
        
        X_coord = -rs_coords[0]
        Y_coord = rs_coords[2]

        # Izračun orientacije
        contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            rect = cv2.minAreaRect(largest_contour)
            raw_angle = rect[2]
            
            width, height = rect[1]
            if width < height:
                calculated_angle = raw_angle
            else:
                calculated_angle = raw_angle - 90 if raw_angle > 0 else raw_angle + 90
            
            orientation = float(np.clip(calculated_angle, -30.0, 30.0))
        else:
            orientation = 0.0

        # Izris rezultatov na sliko
        vis_img = color_image.copy()
        if contours:
            cv2.drawContours(vis_img, contours, -1, (0, 255, 0), 2)
        cv2.circle(vis_img, (u, v), 7, (0, 0, 255), -1)
        
        text_x = f"X: {X_coord:.3f} m (L+/R-)"
        text_y = f"Y: {Y_coord:.3f} m (Naprej)"
        text_o = f"O: {orientation:.2f} deg"
        
        cv2.rectangle(vis_img, (10, 10), (320, 110), (0, 0, 0), -1)
        cv2.putText(vis_img, text_x, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(vis_img, text_y, (20, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(vis_img, text_o, (20, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # Serijsko pošiljanje
        if ser and ser.is_open:
            cmd_x = f"X={X_coord:.3f}\r\n"
            cmd_y = f"Y={Y_coord:.3f}\r\n"
            cmd_o = f"O={orientation:.2f}\r\n"

            print(f"[DEBUG] Pošiljam podatke na STM32:")
            ser.write(cmd_x.encode())
            time.sleep(0.1)
            ser.write(cmd_y.encode())
            time.sleep(0.1)
            ser.write(cmd_o.encode())
            time.sleep(0.1)
            ser.write("GO\r\n".encode())
            print("[DEBUG] Podatki poslani.")
        else:
            print(f"[DEBUG] Serijska vrata zaprta. Rezultat: X={X_coord:.3f}, Y={Y_coord:.3f}, O={orientation:.2f}")

        return vis_img
    else:
        print("[DEBUG] Ni veljavnih globinskih podatkov znotraj maske.")
        return None

try:
    if DEVELOPMENT:
        print("\n=== DEVELOPMENT MODE ACTIVATED ===")
        print("-> Ustvarjam OpenCV okno...")
        cv2.namedWindow("RealSense - Live Stream", cv2.WINDOW_NORMAL)
        cv2.startWindowThread() # Prisili sistem, da osveži okna v ozadju

        print("-> Vstopam v glavno zanko prenašanja videa...")
        print("-> Klikni na okno s sliko in pritisni 's' za zajem ali 'q' za izhod.\n")

        while True:
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            # Barvni prikaz globine
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET
            )

            # Združitev slik v eno veliko okno
            live_view = np.hstack((color_image, depth_colormap))

            cv2.putText(live_view, "LIVE COLOR", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(live_view, "LIVE DEPTH MAP", (1310, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(live_view, "Pritisni 's' za segmentacijo | 'q' za izhod", (30, 700), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            # Prikaži okno
            cv2.imshow("RealSense - Live Stream", live_view)

            # Pomembno: waitKey(1) mora biti vsaj 1 ms, da OpenCV sploh nariše okno na zaslon!
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                print("[DEBUG] Zaznan pritisk na 'q'. Zapiram program...")
                break
            elif key == ord('s'):
                print("[DEBUG] Zaznan pritisk na 's'. Izvajam zajem...")
                vis_result = run_segmentation(color_image, depth_image)
                
                if vis_result is not None:
                    cv2.imshow("RealSense - Live Stream", vis_result)
                    print("[DEBUG] Rezultat prikazan. Pritisni katerokoli tipko za vrnitev v live stream...")
                    cv2.waitKey(0) # Čaka na poljuben pritisk tipke za nadaljevanje
                else:
                    print("[DEBUG] Segmentacija ni vrnila rezultatov.")

    else:
        print("[DEBUG] Zagon v produkcijskem načinu (brez grafičnega vmesnika)...")
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
    print("[DEBUG] Čiščenje in zapiranje virov...")
    pipeline.stop()
    cv2.destroyAllWindows()
    if ser and ser.is_open:
        ser.close()
    print("[DEBUG] Program uspešno zaključen.")
