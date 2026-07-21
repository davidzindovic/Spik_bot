import os
import sys
import time
import math

# ==============================================================================
# OSNOVNE NASTAVITVE
# ==============================================================================
os.environ["CUDA_VISIBLE_DEVICES"] = ""
os.environ["HF_TOKEN"] = "VPISI_SVOJ_HF_TOKEN_TUKAJ"

try:
    sys.modules['torchaudio'] = None
except Exception:
    pass

# ==============================================================================
# PREVERJANJE IN UVOZ KNJIŽNIC
# ==============================================================================
try:
    import pyrealsense2 as rs
except ImportError:
    print("[INFO] Namestitev pyrealsense2...")
    os.system(f"{sys.executable} -m pip install pyrealsense2")
    import pyrealsense2 as rs

try:
    import numpy as np
except ImportError:
    print("[INFO] Namestitev numpy...")
    os.system(f"{sys.executable} -m pip install numpy")
    import numpy as np

try:
    import cv2
except ImportError:
    print("[INFO] Namestitev opencv-python...")
    os.system(f"{sys.executable} -m pip install opencv-python")
    import cv2

try:
    import serial
except ImportError:
    print("[INFO] Namestitev pyserial...")
    os.system(f"{sys.executable} -m pip install pyserial")
    import serial

try:
    import torch
    from transformers import CLIPSegProcessor, CLIPSegForImageSegmentation
except ImportError as e:
    print(f"\n[NAPAKA] Težava pri uvozu PyTorch ali Transformers: {e}")
    print("[INFO] Poskušam namestiti ustrezne različice...")
    os.system(f"{sys.executable} -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu")
    os.system(f"{sys.executable} -m pip install transformers")
    import torch
    from transformers import CLIPSegProcessor, CLIPSegForImageSegmentation

# ==============================================================================
# NASTAVITVE PROGRAMA IN KALIBRACIJA
# ==============================================================================
DEVELOPMENT = True  
SERIAL_PORT = 'COM3'  # Windows COM vrata ali /dev/ttyACM0 na Linuxu
BAUD_RATE = 115200

# --- KALIBRACIJSKI PARAMETRI (Nastavi po potrebi) ---
X_SCALE = 1.0       # Množilnik za X (npr. 1.05 poveča X za 5%)
X_OFFSET_MM = -25     # Konstanten zamik za X v mm (npr. +15 ali -10)

Y_SCALE = 1.0       # Množilnik za Y (globina)
Y_OFFSET_MM = -230     # Konstanten zamik za Y v mm
# ==============================================================================

# --- Nastavitve serijske komunikacije ---
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"[DEBUG] Povezava uspešno vzpostavljena na {SERIAL_PORT}")
except Exception as e:
    print(f"[DEBUG] Opozorilo pri povezovanju na {SERIAL_PORT}: {e}")
    print("[DEBUG] Program bo tekel brez pošiljanja na STM32.")
    ser = None

# --- Nastavitve RealSense Kamere ---
try:
    print("[DEBUG] Inicializacija RealSense kamere...")
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

    profile = pipeline.start(config)
    align_to = rs.stream.color
    align = rs.align(align_to)
    intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    print("[DEBUG] Kamera uspešno pripravljena.")
except Exception as e:
    print(f"[DEBUG] NAPAKA pri inicializaciji kamere: {e}")
    sys.exit(1)

# --- Nalaganje CLIPSeg modela ---
try:
    device = "cpu"  
    print(f"[DEBUG] Izbrana naprava za poganjanje CLIPSeg: {device.upper()}")
    
    print("[DEBUG] Nalagam CLIPSeg model...")
    processor = CLIPSegProcessor.from_pretrained("CIDAS/clipseg-rd64-refined")
    model = CLIPSegForImageSegmentation.from_pretrained("CIDAS/clipseg-rd64-refined")
    model.to(device)
    print("[DEBUG] CLIPSeg model uspešno naložen.")
except Exception as e:
    print(f"[DEBUG] NAPAKA pri nalaganju modela: {e}")
    sys.exit(1)


def send_to_stm32(x, y, z, o):
    """Pomožna funkcija za pošiljanje usklajenih koordinat na STM32."""
    if ser and ser.is_open:
        cmd_x = f"X={x}\r\n"
        cmd_y = f"Y={y}\r\n"
        cmd_z = f"Z={z}\r\n"
        cmd_o = f"O={o:.2f}\r\n"

        print(f"\n[SERIJSKI] Pošiljam podatke na STM32:")
        print(f"  -> {cmd_x.strip()}")
        print(f"  -> {cmd_y.strip()}")
        print(f"  -> {cmd_z.strip()}")
        print(f"  -> {cmd_o.strip()}")
        
        ser.write(cmd_x.encode())
        time.sleep(0.1)
        ser.write(cmd_y.encode())
        time.sleep(0.1)
        ser.write(cmd_z.encode())
        time.sleep(0.1)
        ser.write(cmd_o.encode())
        time.sleep(0.1)
        ser.write("GO\r\n".encode())
        print("[SERIJSKI] Ukazi GO uspešno poslani.")
    else:
        print(f"\n[DEBUG] Serijska vrata niso povezana. Bi pa poslal: X={x}, Y={y}, Z={z}, O={o:.2f}")


def run_segmentation(color_image, depth_image):
    """Izvede CLIPSeg segmentacijo, določi središče debla in vrne izračunane podatke ter sliko."""
    print("\n[DEBUG] Poganjam CLIPSeg segmentacijo (iskanje 'tree trunk')...")
    
    # Pretvori BGR v RGB
    rgb_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
    
    # Priprava vhodnih podatkov za CLIPSeg
    inputs = processor(text=["tree trunk"], images=[rgb_image], padding="max_length", return_tensors="pt")
    inputs = {k: v.to(device) for k, v in inputs.items()}
    
    with torch.no_grad():
        outputs = model(**inputs)
    
    # Prilagoditev ločljivosti na 1280x720
    logits = outputs.logits.unsqueeze(1)
    preds = torch.nn.functional.interpolate(
        logits,
        size=(720, 1280),
        mode="bilinear"
    )
    
    probs = torch.sigmoid(preds[0][0])
    mask = (probs > 0.35).cpu().numpy().astype(bool)

    # Poiščemo konture debla
    contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        
        # Izračun težišča konture (v piklih: u = kolona/X, v = vrstica/Y na sliki)
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            u = int(M["m10"] / M["m00"])
            v = int(M["m01"] / M["m00"])
        else:
            rect = cv2.minAreaRect(largest_contour)
            u, v = int(rect[0][0]), int(rect[0][1])

        # Globina v težišču (mediana 5x5 piklov)
        half_w = 2
        depth_roi = depth_image[max(0, v-half_w):min(720, v+half_w+1), max(0, u-half_w):min(1280, u+half_w+1)]
        valid_roi_depths = depth_roi[depth_roi > 0]

        if len(valid_roi_depths) > 0:
            depth_raw = np.median(valid_roi_depths)
        else:
            masked_depth = np.where(mask, depth_image, 0)
            valid_depths = masked_depth[masked_depth > 0]
            if len(valid_depths) > 0:
                depth_raw = np.min(valid_depths)
                y_indices, x_indices = np.where(masked_depth == depth_raw)
                u, v = x_indices[0], y_indices[0]
            else:
                print("[DEBUG] Ni veljavnih globinskih podatkov za deblo.")
                return None

        # Pretvorba raw globine v metre
        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        depth_in_meters = depth_raw * depth_scale

        # Standardna RealSense 3D deprojekcija:
        # rs_coords[0] = X (desno +, levo -)
        # rs_coords[1] = Y (dol +, gor -)
        # rs_coords[2] = Z (naprej/globina +)
        rs_coords = rs.rs2_deproject_pixel_to_point(intrinsics, [u, v], depth_in_meters)
        
        # --- PRENOS V TVOJ KOORDINATNI SISTEM (v mm) ---
        
        # 1. Y KOORDINATA (GLOBINA NAPREJ):
        # Direktna razdalja naprej od kamere
        raw_y_mm = rs_coords[2] * 1000  
        
        # 2. X KOORDINATA (LEVO / DESNO):
        # Obrnemo predznak, da je Levo (+), Desno (-)
        raw_x_mm = -rs_coords[0] * 1000 

        # --- APLICIRANJE SKALIRANJA IN OFFSETI ---
        X_coord_mm = int(round((raw_x_mm * X_SCALE) + X_OFFSET_MM))
        Y_coord_mm = int(round((raw_y_mm * Y_SCALE) + Y_OFFSET_MM))
        Z_coord_mm = 100  # Konstanten vbodni offset: +100 mm nad kamero

        # --- RADIALNI KOT (O) ---
        # Računa se kot odmik od središča glede na globino Y in zamik X
        radial_angle_rad = math.atan2(X_coord_mm, Y_coord_mm)
        radial_angle_deg = math.degrees(radial_angle_rad)
        orientation = float(np.clip(radial_angle_deg, -30.0, 30.0))

        # --- Vizualizacija ---
        vis_img = color_image.copy()
        cv2.drawContours(vis_img, [largest_contour], -1, (0, 255, 0), 2)
        cv2.circle(vis_img, (u, v), 7, (0, 0, 255), -1)
        
        text_x = f"X: {X_coord_mm} mm (L+/R-)"
        text_y = f"Y: {Y_coord_mm} mm (Globina naprej)"
        text_z = f"Z: {Z_coord_mm} mm (Offset)"
        text_o = f"O: {orientation:.2f} deg (Radial)"
        
        cv2.rectangle(vis_img, (10, 10), (410, 135), (0, 0, 0), -1)
        cv2.putText(vis_img, text_x, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        cv2.putText(vis_img, text_y, (20, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        cv2.putText(vis_img, text_z, (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        cv2.putText(vis_img, text_o, (20, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        
        if DEVELOPMENT:
            cv2.putText(vis_img, "[p] Poslji koordinate | [katerakoli druga] Preklici", (20, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

        return vis_img, (X_coord_mm, Y_coord_mm, Z_coord_mm, orientation)
    else:
        print("[DEBUG] Deblo ni bilo zaznano na sliki.")
        return None

try:
    if DEVELOPMENT:
        print("\n=== DEVELOPMENT MODE ACTIVATED ===")
        cv2.namedWindow("RealSense - Live Stream", cv2.WINDOW_NORMAL)
        cv2.startWindowThread()

        print("-> Pritisni 's' za segmentacijo | 'q' za izhod\n")

        while True:
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET
            )

            live_view = np.hstack((color_image, depth_colormap))

            cv2.putText(live_view, "LIVE COLOR", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(live_view, "LIVE DEPTH MAP (JET)", (1310, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(live_view, "Pritisni 's' za segmentacijo | 'q' za izhod", (30, 700), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            cv2.imshow("RealSense - Live Stream", live_view)

            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break
            elif key == ord('s'):
                res = run_segmentation(color_image, depth_image)
                
                if res is not None:
                    vis_result, coords = res
                    cv2.imshow("RealSense - Live Stream", vis_result)
                    
                    print("\nPrikazan je rezultat segmentacije.")
                    print("-> Pritisni 'p' za POSILJANJE koordinat na STM32.")
                    print("-> Pritisni katerokoli drugo tipko za VRNITEV v prenos brez pošiljanja.")
                    
                    decision_key = cv2.waitKey(0) & 0xFF
                    
                    if decision_key == ord('p'):
                        x, y, z, o = coords
                        send_to_stm32(x, y, z, o)
                    else:
                        print("[DEBUG] Pošiljanje preklicano s strani uporabnika.")
                else:
                    print("\nSegmentacija ni uspela.")

    else:
        print("[DEBUG] Zagon v produkcijskem načinu...")
        for _ in range(30):
            pipeline.wait_for_frames()

        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if color_frame and depth_frame:
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            res = run_segmentation(color_image, depth_image)
            if res is not None:
                _, coords = res
                x, y, z, o = coords
                send_to_stm32(x, y, z, o)

finally:
    print("[DEBUG] Zapiram vire...")
    pipeline.stop()
    cv2.destroyAllWindows()
    if ser and ser.is_open:
        ser.close()
    print("Program zaključen.")
