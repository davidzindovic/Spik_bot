import os
import sys
import time
import math

# ==============================================================================
# OSNOVNE NASTAVITVE
# ==============================================================================
os.environ["CUDA_VISIBLE_DEVICES"] = ""
os.environ["HF_TOKEN"] = ""

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
    print("[NAPAKA] Manjka knjižnica 'pyrealsense2'. Nameščam jo z: pip install pyrealsense2")
    os.system('python -m pip install pyrealsense2')
finally:
    import pyrealsense2 as rs

try:
    import numpy as np
except ImportError:
    print("[NAPAKA] Manjka knjižnica 'numpy'. Nameščam jo z: pip install numpy")
    os.system('python -m pip install pip install numpy')
finally:
    import numpy as np

try:
    import cv2
except ImportError:
    print("[NAPAKA] Manjka knjižnica 'opencv-python'. Nameščam jo z: pip install opencv-python")
    os.system('python -m pip install pip install opencv-python')
finally:
    import cv2

try:
    import serial
except ImportError:
    print("[NAPAKA] Manjka knjižnica 'pyserial'. Nameščam jo z: pip install pyserial")
    os.system('python -m pip install pip install pyserial')
finally:
    import serial

try:
    import torch
    from transformers import CLIPSegProcessor, CLIPSegForImageSegmentation
except ImportError as e:
    print("\n" + "="*80)
    print(f"[NAPAKA] Težava pri uvozu PyTorch/Transformers: {e}")
    os.system('python -m pip install pip uninstall -y torch torchvision torchaudio')
    os.system('python -m pip install pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu')
    os.system('python -m pip install pip install transformers')
finally:
    import torch
    from transformers import CLIPSegProcessor, CLIPSegForImageSegmentation

# ==============================================================================
# NASTAVITVE PROGRAMA IN KALIBRACIJA
# ==============================================================================
DEVELOPMENT = True  
SERIAL_PORT = 'COM3'  # Ali /dev/ttyUSB0 / COM3
BAUD_RATE = 115200

# --- CILJNI DOSEG (200 mm - 400 mm) ---
MIN_DISTANCE_MM = 200    # Minimalna razdalja (20 cm)
MAX_DISTANCE_MM = 400    # Maksimalna razdalja (40 cm)

# --- ZASTAVICA ZA POVPREČENJE ---
ENABLE_AVERAGING = False  # True = zajame več slik in povpreči | False = izvede meritev na 1 sliki
NUM_SAMPLES = 7          # Število slik za povprečenje (če je ENABLE_AVERAGING = True)

# --- KALIBRACIJSKI PARAMETRI ---
X_SCALE = 1.0       
X_OFFSET_MM = 0     

Y_SCALE = 1.0       
Y_OFFSET_MM = 0     

Z_HEIGHT_MM = -100   # Fiksna višina nad sredino kamere (+100 mm)
# ==============================================================================

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"[DEBUG] Povezava uspešno vzpostavljena na {SERIAL_PORT}")
except Exception as e:
    print(f"[DEBUG] Opozorilo pri povezovanju na {SERIAL_PORT}: {e}")
    ser = None

try:
    print("[DEBUG] Inicializacija RealSense kamere...")
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)

    profile = pipeline.start(config)
    dev = profile.get_device()
    depth_sensor = dev.first_depth_sensor()

    # Nastavitev Disparity Shift za optimalno delovanje v območju 200–400 mm
    try:
        if dev.supports(rs.camera_info.advanced_mode):
            adv_mode = rs.rs400_advanced_mode(dev)
            if adv_mode.is_enabled():
                depth_table = adv_mode.get_depth_table()
                depth_table.disparityShift = 55  
                adv_mode.set_depth_table(depth_table)
                print("[DEBUG] Disparity Shift (55) uveljavljen za doseg 200-400 mm.")
    except Exception as err:
        print(f"[DEBUG] Napredne nastavitve niso bile uveljavljene: {err}")

    if depth_sensor.supports(rs.option.visual_preset):
        try:
            depth_sensor.set_option(rs.option.visual_preset, 3) # High Accuracy
        except Exception:
            pass

    # RealSense filtri za zmanjšanje šuma
    spatial_filter = rs.spatial_filter()
    hole_filling = rs.hole_filling_filter()
    hole_filling.set_option(rs.option.holes_fill, 1)

    align_to = rs.stream.color
    align = rs.align(align_to)
    intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    
    depth_scale = depth_sensor.get_depth_scale()
    
    print(f"[DEBUG] Kamera pripravljena za območje: {MIN_DISTANCE_MM} - {MAX_DISTANCE_MM} mm.")
except Exception as e:
    print(f"[DEBUG] NAPAKA pri inicializaciji kamere: {e}")
    sys.exit(1)

try:
    device = "cpu"  
    print("[DEBUG] Nalagam CLIPSeg model...")
    processor = CLIPSegProcessor.from_pretrained("CIDAS/clipseg-rd64-refined")
    model = CLIPSegForImageSegmentation.from_pretrained("CIDAS/clipseg-rd64-refined")
    model.to(device)
    print("[DEBUG] CLIPSeg model uspešno naložen.")
except Exception as e:
    print(f"[DEBUG] NAPAKA pri nalaganju modela: {e}")
    sys.exit(1)


def filter_by_distance(color_image, depth_image):
    """
    Na barvni in globinski sliki ohrani SAMO območja, ki imajo potrjeno globino
    med MIN_DISTANCE_MM in MAX_DISTANCE_MM. Vse ostalo bo ČRNO.
    """
    depth_in_mm = depth_image.astype(np.float32) * depth_scale * 1000.0
    valid_mask = (depth_in_mm >= MIN_DISTANCE_MM) & (depth_in_mm <= MAX_DISTANCE_MM)

    filtered_color = color_image.copy()
    filtered_depth = depth_image.copy()

    filtered_color[~valid_mask] = 0
    filtered_depth[~valid_mask] = 0

    return filtered_color, filtered_depth


def save_full_color_image(raw_color_img, contour, u, v, text_lines):
    """
    Shrani polno barvno sliko (brez črnih pik) z narisano konturo, točko in meritvami.
    """
    folder_name = "slike_segmentacija"
    if not os.path.exists(folder_name):
        os.makedirs(folder_name)

    # Najdemo naslednje prosto ime slike
    i = 1
    while True:
        file_path = os.path.join(folder_name, f"segmentacija_{i:03d}.png")
        if not os.path.exists(file_path):
            break
        i += 1

    # Narišemo na popolnoma barvno sliko
    save_img = raw_color_img.copy()
    if contour is not None:
        cv2.drawContours(save_img, [contour], -1, (0, 255, 0), 2)
    
    cv2.circle(save_img, (u, v), 7, (0, 0, 255), -1)

    # Prikaz besedila
    cv2.rectangle(save_img, (10, 10), (430, 115), (0, 0, 0), -1)
    y_pos = 30
    for line in text_lines:
        cv2.putText(save_img, line, (20, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2)
        y_pos += 25

    cv2.imwrite(file_path, save_img)
    print(f"[INFO] Polno barvna slika uspešno shranjena v: {file_path}")


def send_to_stm32(x, y, z, o):
    """Pošiljanje usklajenih koordinat na STM32."""
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


def process_single_frame(color_image, depth_frame_raw):
    """Filtrira sliko po razdalji, jo segmentira in vrne izračunano točko."""
    filtered_dframe = spatial_filter.process(depth_frame_raw)
    filtered_dframe = hole_filling.process(filtered_dframe)
    depth_image = np.asanyarray(filtered_dframe.get_data())

    color_filtered, depth_filtered = filter_by_distance(color_image, depth_image)
    
    rgb_image = cv2.cvtColor(color_filtered, cv2.COLOR_BGR2RGB)
    inputs = processor(text=["tree trunk"], images=[rgb_image], padding="max_length", return_tensors="pt")
    inputs = {k: v.to(device) for k, v in inputs.items()}
    
    with torch.no_grad():
        outputs = model(**inputs)
    
    logits = outputs.logits.unsqueeze(1)
    preds = torch.nn.functional.interpolate(logits, size=(720, 1280), mode="bilinear")
    probs = torch.sigmoid(preds[0][0])
    mask = (probs > 0.35).cpu().numpy().astype(bool)

    contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        
        masked_depth = np.where(mask, depth_filtered, 0)
        valid_depths = masked_depth[masked_depth > 0]
        
        if len(valid_depths) == 0:
            return None
            
        approx_depth_m = np.median(valid_depths) * depth_scale
        
        cy, fy = intrinsics.ppy, intrinsics.fy
        target_v = int(round(cy - ((Z_HEIGHT_MM / 1000.0) * fy / approx_depth_m)))
        v = int(np.clip(target_v, 0, 719))

        row_mask = mask[v, :]
        x_indices = np.where(row_mask)[0]
        
        if len(x_indices) > 0:
            u = int(np.mean(x_indices))
        else:
            M = cv2.moments(largest_contour)
            u = int(M["m10"] / M["m00"]) if M["m00"] != 0 else 640

        half_w = 3
        depth_roi = depth_filtered[max(0, v-half_w):min(720, v+half_w+1), max(0, u-half_w):min(1280, u+half_w+1)]
        valid_roi_depths = depth_roi[depth_roi > 0]

        depth_raw = np.median(valid_roi_depths) if len(valid_roi_depths) > 0 else np.median(valid_depths)
        depth_in_meters = depth_raw * depth_scale

        rs_coords = rs.rs2_deproject_pixel_to_point(intrinsics, [u, v], depth_in_meters)
        
        raw_y_mm = rs_coords[2] * 1000  
        raw_x_mm = -rs_coords[0] * 1000 

        return u, v, raw_x_mm, raw_y_mm, largest_contour, color_filtered, color_image
    return None


def run_measurement(color_frame=None, depth_frame=None):
    """Izvede meritev na 1 sliki ali pa povpreči več slik (glede na ENABLE_AVERAGING)."""
    if ENABLE_AVERAGING:
        print(f"\n[DEBUG] Povprečenje JE VKLOPLJENO. Zajema se {NUM_SAMPLES} slik ({MIN_DISTANCE_MM}-{MAX_DISTANCE_MM} mm)...")
        x_list, y_list = [], []
        last_vis_img = None
        last_raw_color = None
        last_contour = None
        last_u, last_v = 640, 360

        for i in range(NUM_SAMPLES):
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            c_frame = aligned_frames.get_color_frame()
            d_frame = aligned_frames.get_depth_frame()

            if not c_frame or not d_frame:
                continue

            c_img = np.asanyarray(c_frame.get_data())

            res = process_single_frame(c_img, d_frame)
            if res is not None:
                u, v, raw_x, raw_y, contour, filtered_c_img, raw_color_img = res
                x_list.append(raw_x)
                y_list.append(raw_y)
                last_vis_img = filtered_c_img.copy()
                last_raw_color = raw_color_img.copy()
                last_contour = contour
                last_u, last_v = u, v

        if len(x_list) == 0:
            print(f"[DEBUG] Deblo ni bilo zaznano v območju {MIN_DISTANCE_MM}-{MAX_DISTANCE_MM} mm.")
            return None

        avg_raw_x = np.median(x_list)
        avg_raw_y = np.median(y_list)
        
        u, v = last_u, last_v
        vis_img = last_vis_img
        raw_color_img = last_raw_color
        contour = last_contour
        raw_x, raw_y = avg_raw_x, avg_raw_y
        mode_label = f"POVPREČJE ({len(x_list)} slik)"

    else:
        print(f"\n[DEBUG] Povprečenje JE IZKLOPLJENO. Procesiram 1 sliko ({MIN_DISTANCE_MM}-{MAX_DISTANCE_MM} mm)...")
        c_img = np.asanyarray(color_frame.get_data())
        res = process_single_frame(c_img, depth_frame)
        if res is None:
            print(f"[DEBUG] Deblo ni bilo zaznano v območju {MIN_DISTANCE_MM}-{MAX_DISTANCE_MM} mm.")
            return None
            
        u, v, raw_x, raw_y, contour, filtered_c_img, raw_color_img = res
        vis_img = filtered_c_img.copy()
        mode_label = "POSAMEZNA SLIKA"

    # Narišemo obrobo na filtrirano sliko za ekran
    if contour is not None:
        cv2.drawContours(vis_img, [contour], -1, (0, 255, 0), 2)

    # Kalibracija
    X_coord_mm = int(round((raw_x * X_SCALE) + X_OFFSET_MM))
    Y_coord_mm = int(round((raw_y * Y_SCALE) + Y_OFFSET_MM))
    Z_coord_mm = int(Z_HEIGHT_MM)

    # Radialni kot O
    radial_angle_rad = math.atan2(X_coord_mm, Y_coord_mm)
    radial_angle_deg = math.degrees(radial_angle_rad)
    orientation = float(np.clip(radial_angle_deg, -30.0, 30.0))

    # Prikaz na ekranu (brez modre črte!)
    cv2.circle(vis_img, (u, v), 7, (0, 0, 255), -1)

    text_x = f"X ({mode_label}): {X_coord_mm} mm"
    text_y = f"Y: {Y_coord_mm} mm"
    text_z = f"Z: {Z_coord_mm} mm"
    text_o = f"O: {orientation:.2f} deg"

    cv2.rectangle(vis_img, (10, 10), (550, 135), (0, 0, 0), -1)
    cv2.putText(vis_img, text_x, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2)
    cv2.putText(vis_img, text_y, (20, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2)
    cv2.putText(vis_img, text_z, (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 255), 2)
    cv2.putText(vis_img, text_o, (20, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2)

    if DEVELOPMENT:
        cv2.putText(vis_img, "[i] Shrani barvno sliko | [p] Poslji | [katerakoli] Preklici", (20, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 0), 1)

    text_lines = [text_x, text_y, text_z, text_o]

    return vis_img, (X_coord_mm, Y_coord_mm, Z_coord_mm, orientation), raw_color_img, contour, u, v, text_lines


try:
    if DEVELOPMENT:
        print("\n=== DEVELOPMENT MODE ACTIVATED ===")
        print(f" Območje merjenja: {MIN_DISTANCE_MM} mm do {MAX_DISTANCE_MM} mm")
        cv2.namedWindow("RealSense - Live Stream", cv2.WINDOW_NORMAL)
        cv2.startWindowThread()

        print("-> Pritisni 's' za segmentacijo/meritev | 'q' za izhod\n")

        while True:
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            live_color_filt, live_depth_filt = filter_by_distance(color_image, depth_image)

            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(live_depth_filt, alpha=0.08), cv2.COLORMAP_JET
            )

            live_view = np.hstack((live_color_filt, depth_colormap))

            cv2.putText(live_view, f"COLOR ({MIN_DISTANCE_MM}-{MAX_DISTANCE_MM}mm)", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(live_view, f"DEPTH ({MIN_DISTANCE_MM}-{MAX_DISTANCE_MM}mm)", (1310, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(live_view, "Pritisni 's' za meritev | 'q' za izhod", (30, 700), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            cv2.imshow("RealSense - Live Stream", live_view)

            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break
            elif key == ord('s'):
                res = run_measurement(color_frame, depth_frame)
                
                if res is not None:
                    vis_result, coords, raw_color_img, contour, u, v, text_lines = res
                    cv2.imshow("RealSense - Live Stream", vis_result)
                    
                    print("\n-> Pritisni 'i' za SHRANJEVANJE polno barvne slike.")
                    print("-> Pritisni 'p' za POSILJANJE koordinat na STM32.")
                    print("-> Pritisni katerokoli drugo tipko za VRNITEV v prenos.")
                    
                    decision_key = cv2.waitKey(0) & 0xFF
                    
                    if decision_key == ord('i'):
                        save_full_color_image(raw_color_img, contour, u, v, text_lines)
                    elif decision_key == ord('p'):
                        x, y, z, o = coords
                        send_to_stm32(x, y, z, o)
                    else:
                        print("[DEBUG] Preklicano s strani uporabnika.")
                else:
                    print(f"\nSegmentacija/meritev ni uspela (ni zaznanih objektov v zoni {MIN_DISTANCE_MM}-{MAX_DISTANCE_MM} mm).")

    else:
        print("[DEBUG] Zagon v produkcijskem načinu...")
        for _ in range(10):
            pipeline.wait_for_frames()

        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        c_frame = aligned_frames.get_color_frame()
        d_frame = aligned_frames.get_depth_frame()

        if c_frame and d_frame:
            res = run_measurement(c_frame, d_frame)
            if res is not None:
                _, coords, _, _, _, _, _ = res
                x, y, z, o = coords
                send_to_stm32(x, y, z, o)

finally:
    print("[DEBUG] Zapiram vire...")
    pipeline.stop()
    cv2.destroyAllWindows()
    if ser and ser.is_open:
        ser.close()
    print("Program zaključen.")
