import os
import sys
import time

# ==============================================================================
# 1. NASTAVITEV HUGGING FACE ŽETONA (TOKEN)
# ==============================================================================
os.environ["HF_TOKEN"] = "VPISI_SVOJ_HF_TOKEN_TUKAJ"

# ==============================================================================
# 2. SAMODEJNA NAMESTITEV IN UVOZ KNJIŽNIC
# ==============================================================================
try:
    import pyrealsense2 as rs
except:
    os.system('python -m pip install pyrealsense2')
finally:
    import pyrealsense2 as rs

try:
    import numpy as np
except:
    os.system('python -m pip install numpy')
finally:
    import numpy as np

try:
    import cv2
except:
    os.system('python -m pip install opencv-python')
finally:
    import cv2

try:
    import serial
except:
    os.system('python -m pip install pyserial')
finally:
    import serial

try:
    import torch
    from transformers import CLIPSegProcessor, CLIPSegForImageSegmentation
except:
    print("[DEBUG] Nameščam transformers in torch...")
    os.system('python -m pip install transformers torch torchvision')
finally:
    import torch
    from transformers import CLIPSegProcessor, CLIPSegForImageSegmentation

# ==============================================================================
# 3. NASTAVITVE PROGRAMA
# ==============================================================================
DEVELOPMENT = True  # True: živi stream s tipko 's' za segmentacijo | False: enojni zajem (produkcija)
SERIAL_PORT = '/dev/ttyACM0'  # OPOMBA: Na Linuxu je COM3 običajno '/dev/ttyACM0' ali '/dev/ttyUSB0'
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

# --- Varna detekcija CUDA in nalaganje CLIPSeg modela ---
try:
    print("[DEBUG] Analiziram napravo za poganjanje modela...")
    device = "cpu"
    
    if torch.cuda.is_available():
        try:
            # Poskusimo inicializirati CUDA s kratkim testom
            # Če gonilniki na Linuxu niso usklajeni, se bo tukaj sprožila napaka
            test_tensor = torch.zeros(1).cuda()
            device = "cuda"
            print("[DEBUG] CUDA je na voljo in deluje pravilno!")
        except Exception as cuda_err:
            print(f"[DEBUG] Opozorilo: CUDA javlja težave ({cuda_err}).")
            print("[DEBUG] Prisilno preklapljam na CPU za stabilno delovanje.")
            device = "cpu"
    else:
        print("[DEBUG] CUDA grafična kartica ni zaznana. Uporabljam CPU.")
        
    print(f"[DEBUG] Izbrana naprava: {device.upper()}")
    
    print("[DEBUG] Nalagam CLIPSeg model...")
    processor = CLIPSegProcessor.from_pretrained("CIDAS/clipseg-rd64-refined")
    model = CLIPSegForImageSegmentation.from_pretrained("CIDAS/clipseg-rd64-refined")
    model.to(device)
    print("[DEBUG] CLIPSeg model uspešno naložen.")
except Exception as e:
    print(f"[DEBUG] NAPAKA pri nalaganju modela: {e}")
    sys.exit(1)


def run_segmentation(color_image, depth_image):
    """Izvede CLIPSeg segmentacijo za 'tree trunk' na podani sliki."""
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

    # Globinski podatki znotraj maske
    masked_depth = np.where(mask, depth_image, 0)
    valid_depths = masked_depth[masked_depth > 0]

    if len(valid_depths) > 0:
        min_depth_raw = np.min(valid_depths)
        
        y_indices, x_indices = np.where(masked_depth == min_depth_raw)
        u, v = x_indices[0], y_indices[0]

        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        depth_in_meters = min_depth_raw * depth_scale

        rs_coords = rs.rs2_deproject_pixel_to_point(intrinsics, [u, v], depth_in_meters)
        
        # --- Prilagoditev koordinatnega sistema ---
        # Y naravnost, X+ levo, X- desno
        X_coord = -rs_coords[0]
        Y_coord = rs_coords[2]

        # --- Izračun orientacije (O) ---
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

        # --- Vizualizacija ---
        vis_img = color_image.copy()
        if contours:
            cv2.drawContours(vis_img, contours, -1, (0, 255, 0), 2)
        cv2.circle(vis_img, (u, v), 7, (0, 0, 255), -1)
        
        text_x = f"X: {X_coord:.3f} m (L+/R-)"
        text_y = f"Y: {Y_coord:.3f} m (Naprej)"
        text_o = f"O: {orientation:.2f} deg ([-30, 30])"
        
        cv2.rectangle(vis_img, (10, 10), (320, 110), (0, 0, 0), -1)
        cv2.putText(vis_img, text_x, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(vis_img, text_y, (20, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(vis_img, text_o, (20, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # --- Pošiljanje na STM32 ---
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
            print("[DEBUG] Ukazi uspešno poslani.")
        else:
            print("[DEBUG] Serijska vrata niso povezana. Rezultati:")
            print(f"  X: {X_coord:.3f} m | Y: {Y_coord:.3f} m | O: {orientation:.2f}°")

        return vis_img
    else:
        print("[DEBUG] Znotraj maske ni veljavnih globinskih podatkov.")
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
