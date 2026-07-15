#import pyrealsense2 as rs
#import numpy as np
#import cv2
#import serial
import time
#from lang_sam import LangSAM

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
      from lang_sam import LangSAM
  except:
      os.system('python -m pip install lang-sam')
  finally:
      from lang_sam import LangSAM

# --- Nastavitve serijske komunikacije ---
# Prilagodi vrata (COM port) glede na tvojo STM32 napravo (npr. 'COM3' na Windows ali '/dev/ttyACM0' na Linuxu)
SERIAL_PORT = 'COM3' 
BAUD_RATE = 115200

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

# --- Nalaganje SAM modela ---
print("Nalagam SAM model...")
model = LangSAM()

try:
    # Pustimo kameri nekaj sličic, da se prilagodi osvetlitvi (Auto-Exposure)
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

    # Pretvori BGR v RGB za SAM model
    rgb_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

    # --- Segmentacija s SAM modelom ---
    print("Iščem 'tree trunk' na sliki...")
    masks, boxes, phrases, logits = model.predict(rgb_image, "tree trunk")

    if len(masks) == 0:
        print("Ključna beseda 'tree trunk' ni bila zaznana.")
    else:
        # Vzamemo prvo zaznano masko (najbolj zanesljivo)
        mask = masks[0].numpy().astype(bool)

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

            # Deprojekcija 2D piksla v 3D prostor (X, Y, Z v metrih glede na kamero)
            point_3d = rs.rs2_deproject_pixel_to_point(intrinsics, [u, v], depth_in_meters)
            X_coord, Y_coord, Z_coord = point_3d

            # --- Izračun orientacije (O) ---
            # Kot orientacije debla bomo določili s pomočjo nagnjenosti zaznane maske (glavna os)
            contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                # minAreaRect vrne ((center_x, center_y), (width, height), angle)
                rect = cv2.minAreaRect(largest_contour)
                orientation = rect[2]  # Kot orientacije v stopinjah
            else:
                orientation = 0.0

            # --- Pošiljanje podatkov preko serijske povezave ---
            if ser and ser.is_open:
                # Formatiranje podatkov na 3 decimalna mesta
                cmd_x = f"X={X_coord:.3f}\r\n"
                cmd_y = f"Y={Y_coord:.3f}\r\n"
                cmd_o = f"O={orientation:.2f}\r\n"

                print(f"\nPošiljam podatke na STM32:")
                print(f"Koda: {cmd_x.strip()}")
                ser.write(cmd_x.encode())
                time.sleep(0.1) # Kratek premor med ukazi, da STM32 lažje procesira

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
                print(f"X (metri): {X_coord:.3f}")
                print(f"Y (metri): {Y_coord:.3f}")
                print(f"Z (globina v metrih): {Z_coord:.3f}")
                print(f"Orientacija (kot): {orientation:.2f}°")

        else:
            print("Znotraj maske ni veljavnih globinskih podatkov (morda preblizu/izven območja senzorja).")

finally:
    # Varno zapiranje virov
    pipeline.stop()
    if ser and ser.is_open:
        ser.close()
    print("Program zaključen.")
