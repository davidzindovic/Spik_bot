import os
import sys
import time
import math

# ==============================================================================
# VARNOSTNI POPRAVKI ZA LINUX (CUDA & AUDIO BYPASS)
# ==============================================================================
os.environ["CUDA_VISIBLE_DEVICES"] = ""
os.environ["HF_TOKEN"] = "VPISI_SVOJ_HF_TOKEN_TUKAJ"

try:
    import sys
    sys.modules['torchaudio'] = None
except Exception:
    pass

# ==============================================================================
# PREVERJANJE IN UVOZ KNJIŽNIC
# ==============================================================================
try:
    import pyrealsense2 as rs
except ImportError:
    print("[NAPAKA] Manjka knjižnica 'pyrealsense2'. Namesti jo z: pip install pyrealsense2")
    sys.exit(1)

try:
    import numpy as np
except ImportError:
    print("[NAPAKA] Manjka knjižnica 'numpy'. Namesti jo z: pip install numpy")
    sys.exit(1)

try:
    import cv2
except ImportError:
    print("[NAPAKA] Manjka knjižnica 'opencv-python'. Namesti jo z: pip install opencv-python")
    sys.exit(1)

try:
    import serial
except ImportError:
    print("[NAPAKA] Manjka knjižnica 'pyserial'. Namesti jo z: pip install pyserial")
    sys.exit(1)

try:
    import torch
    from transformers import CLIPSegProcessor, CLIPSegForImageSegmentation
except ImportError as e:
    print("\n" + "="*80)
    print(f"[NAPAKA] Težava pri uvozu PyTorch/Transformers: {e}")
    sys.exit(1)

# ==============================================================================
# NASTAVITVE PROGRAMA
# ==============================================================================
DEVELOPMENT = True  
SERIAL_PORT = '/dev/ttyACM0'  # Na Linuxu je običajno /dev/ttyACM0 ali /dev/ttyUSB0
BAUD_RATE = 115200
# ==============================================================================

# --- Nastavitve serijske komunikacije ---
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"[DEBUG] Povezava uspešno vzpostavljena na {SERIAL_PORT}")
except Exception as e:
    print(f"[DEBUG] Opozorilo pri povezovanju na serijska vrata: {e}")
    print("[DEBUG] Poskušam se povezati na alternativna vrata /dev/ttyUSB0...")
    try:
        ser = serial.Serial('/dev/ttyUSB0', BAUD_RATE, timeout=1)
        print("[DEBUG] Uspešno povezan na /dev/ttyUSB0")
    except:
        print("[DEBUG] Serijska vrata niso na voljo. Program bo tekel brez pošiljanja na STM32.")
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


def run_segmentation(color_image, depth_image):
    """Izvede CLIPSeg segmentacijo, določi središče debla in izračuna radialni kot."""
    print("\n[DEBUG] Poganjam CLIPSeg segmentacijo (iskanje 'tree trunk')...")
    
    # Pretvori BGR v RGB
    rgb_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
    
    # Priprava vhodnih podatkov
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
        # Vzamemo največjo zaznano konturo (deblo)
        largest_contour = max(contours, key=cv2.contourArea)
        
        # Izračun težišča (momenti) konture za določitev robustnega središča debla
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            u = int(M["m10"] / M["m00"])
            v = int(M["m01"] / M["m00"])
        else:
            # Varnostni mehanizem, če momenti vrnejo 0
            rect = cv2.minAreaRect(largest_contour)
            u, v = int(rect[0][0]), int(rect[0][1])

        # Preverimo globino v okolici težišča (da se izognemo morebitnemu šumu ali luknjam)
        # Vzamemo majhno regijo 5x5 piklov okoli središča in izračunamo mediano veljavnih globin
        half_w = 2
        depth_roi = depth_image[max(0, v-half_w):min(720, v+half_w+1), max(0, u-half_w):min(1280, u+half_w+1)]
        valid_roi_depths = depth_roi[depth_roi > 0]

        if len(valid_roi_depths) > 0:
            depth_raw = np.median(valid_roi_depths)
        else:
            # Če v središču ni podatka, poskusimo poiskati najbližjo veljavno točko znotraj celotne maske debla
            masked_depth = np.where(mask, depth_image, 0)
            valid_depths = masked_depth[masked_depth > 0]
            if len(valid_depths) > 0:
                depth_raw = np.min(valid_depths)
                # Posodobimo točko na tisto, ki ima to globino
                y_indices, x_indices = np.where(masked_depth == depth_raw)
                u, v = x_indices[0], y_indices[0]
            else:
                print("[DEBUG] Ni veljavnih globinskih podatkov za deblo.")
                return None

        # Pretvorba globine v metre
        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        depth_in_meters = depth_raw * depth_scale

        # Deprojekcija slikovne točke (u, v) v realne 3D koordinate (v metrih)
        rs_coords = rs.rs2_deproject_pixel_to_point(intrinsics, [u, v], depth_in_meters)
        
        # --- Prilagoditev koordinatnega sistema v milimetre ---
        # Y naravnost, X+ levo, X- desno
        X_coord_mm = int(round(-rs_coords[0] * 1000))
        Y_coord_mm = int(round(rs_coords[2] * 1000))

        # --- Izračun RADIALNEGA kota (O) v stopinjah ---
        # atan2(X, Y) vrne kot v radianih. Ker je Y naprej in X levo/desno, dobimo radialno smer.
        # Pomnožimo z -1 ali prilagodimo predznak glede na želeno smer (tukaj: X+ levo je pozitivni kot)
        radial_angle_rad = math.atan2(X_coord_mm, Y_coord_mm)
        radial_angle_deg = math.degrees(radial_angle_rad)
        
        # Omejimo kot na interval [-30.0, 30.0] stopinj
        orientation = float(np.clip(radial_angle_deg, -30.0, 30.0))

        # --- Vizualizacija (prikaz v mm in stopinjah) ---
        vis_img = color_image.copy()
        cv2.drawContours(vis_img, [largest_contour], -1, (0, 255, 0), 2)
        
        # Označimo zaznano središče debla z rdečim krogcem
        cv2.circle(vis_img, (u, v), 7, (0, 0, 255), -1)
        
        text_x = f"X: {X_coord_mm} mm (L+/R-)"
        text_y = f"Y: {Y_coord_mm} mm (Naprej)"
        text_o = f"O: {orientation:.2f} deg (Radial)"
        
        cv2.rectangle(vis_img, (10, 10), (320, 110), (0, 0, 0), -1)
        cv2.putText(vis_img, text_x, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(vis_img, text_y, (20, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(vis_img, text_o, (20, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # --- Pošiljanje na STM32 (vrednosti poslane v mm in radialni stopinjah) ---
        if ser and ser.is_open:
            cmd_x = f"X={X_coord_mm}\r\n"
            cmd_y = f"Y={Y_coord_mm}\r\n"
            cmd_o = f"O={orientation:.2f}\r\n"

            print(f"[DEBUG] Pošiljam podatke na STM32:")
            ser.write(cmd_x.encode())
            time.sleep(0.1)
            ser.write(cmd_y.encode())
            time.sleep(0.1)
            ser.write(cmd_o.encode())
            time.sleep(0.1)
            ser.write("GO\r\n".encode())
            print("[DEBUG] Ukazi uspešno poslani.")
        else:
            print("[DEBUG] Serijska vrata niso povezana. Rezultati:")
            print(f"  X: {X_coord_mm} mm | Y: {Y_coord_mm} mm | O: {orientation:.2f}° (Radial)")

        return vis_img
    else:
        print("[DEBUG] Deblo ni bilo zaznano na sliki.")
        return None


try:
    if DEVELOPMENT:
        print("\n=== DEVELOPMENT MODE ACTIVATED ===")
        cv2.namedWindow("RealSense - Live Stream", cv2.WINDOW_NORMAL)
        cv2.startWindowThread()

        print("-> Pritisni 's' za segmentacijo 'tree trunk' | 'q' za izhod\n")

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

            # Združitev slik v eno okno
            live_view = np.hstack((color_image, depth_colormap))

            cv2.putText(live_view, "LIVE COLOR", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(live_view, "LIVE DEPTH MAP (JET)", (1310, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(live_view, "Pritisni 's' za segmentacijo | 'q' za izhod", (30, 700), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            cv2.imshow("RealSense - Live Stream", live_view)

            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break
            elif key == ord('s'):
                vis_result = run_segmentation(color_image, depth_image)
                
                if vis_result is not None:
                    cv2.imshow("RealSense - Live Stream", vis_result)
                    print("\nPrikazan je rezultat. Pritisni poljubno tipko za vrnitev v prenos...")
                    cv2.waitKey(0)
                else:
                    print("\nSegmentacija ni uspela.")

    else:
        # PRODUKCIJSKI NAČIN
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
            run_segmentation(color_image, depth_image)

finally:
    print("[DEBUG] Zapiram vire...")
    pipeline.stop()
    cv2.destroyAllWindows()
    if ser and ser.is_open:
        ser.close()
    print("Program zaključen.")
