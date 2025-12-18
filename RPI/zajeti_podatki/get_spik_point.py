import cv2
import numpy as np
import os
import glob

# osnova je delujoče iskanje objektov
# potrebno naštimati parametre kamere in transformacijo kamera-robot

# --- NASTAVITVE PARAMETROV ---
# 1. Notranji parametri kamere (Vnesi dejanske podatke za svojo kamero!)
# Če uporabljaš npr. RealSense ali Kinect, te podatke dobiš iz SDK-ja.
FX = 600.0  # goriščna razdalja x
FY = 600.0  # goriščna razdalja y
CX = 320.0  # optično središče x (običajno širina/2)
CY = 240.0  # optično središče y (običajno višina/2)

# 2. Transformacija Kamera -> Aktuator (v mm)
# Zamik središča aktuatorja glede na središče kamere
OFF_X = 50.0   # Aktuator je 50mm desno od kamere
OFF_Y = -100.0 # Aktuator je 100mm nad kamero
OFF_Z = 20.0   # Aktuator je 20mm za kamero

# 3. Višina kamere nad tlemi (v mm)
CAMERA_HEIGHT = 1200.0 

def rgb_to_hsv_threshold(rgb_val):
    pixel = np.uint8([[ [rgb_val[2], rgb_val[1], rgb_val[0]] ]])
    hsv_pixel = cv2.cvtColor(pixel, cv2.COLOR_BGR2HSV)[0][0]
    return hsv_pixel

def get_3d_coordinates(u, v, z):
    """Izračun realnih X, Y koordinat iz pikslov in globine"""
    if z == 0: return 0, 0, 0
    x = (u - CX) * z / FX
    y = (v - CY) * z / FY
    return x, y, z

# --- BARVNI RAZPON ---
rgb_low = [32, 38, 26]
rgb_high = [52, 54, 43]
hsv_low_base = rgb_to_hsv_threshold(rgb_low)
hsv_high_base = rgb_to_hsv_threshold(rgb_high)

lower_hsv = np.array([max(0, hsv_low_base[0] - 15), 
                      max(0, hsv_low_base[1] - 30), 
                      max(0, hsv_low_base[2] - 40)])
upper_hsv = np.array([min(180, hsv_high_base[0] + 15), 
                      min(255, hsv_high_base[1] + 120), 
                      min(255, hsv_high_base[2] + 180)])

color_images = glob.glob("*barva.png")

for color_img_path in sorted(color_images):
    base_name = os.path.basename(color_img_path)
    img_number = base_name.replace("barva.png", "")
    depth_img_path = f"{img_number}globina.npy"
    
    if not os.path.exists(depth_img_path): continue
    
    try:
        color_img = cv2.imread(color_img_path)
        depth_map = np.load(depth_img_path)
        display_img = color_img.copy()
        
        # Procesiranje
        color_img_blurred = cv2.GaussianBlur(color_img, (9, 9), 0)
        hsv = cv2.cvtColor(color_img_blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        best_cnt = None
        max_ratio = 0
        best_coords_cam = (0, 0, 0)

        for cnt in contours:
            if cv2.contourArea(cnt) < 500: continue
            
            # Težišče konture (u, v piksel)
            M = cv2.moments(cnt)
            if M["m00"] == 0: continue
            u_center = int(M["m10"] / M["m00"])
            v_center = int(M["m01"] / M["m00"])
            
            # Globina na težišču
            avg_dist = depth_map[v_center, u_center]
            
            # Če je piksel v težišču neveljaven (0), vzemi povprečje okolice
            if avg_dist == 0:
                mask_temp = np.zeros(mask.shape, np.uint8)
                cv2.drawContours(mask_temp, [cnt], -1, 255, -1)
                depths = depth_map[(mask_temp > 0) & (depth_map > 0)]
                if depths.size > 0: avg_dist = np.percentile(depths, 15)
                else: continue

            if avg_dist < 2000:
                rect = cv2.minAreaRect(cnt)
                length = max(rect[1])
                width = min(rect[1]) if min(rect[1]) > 0 else 1
                ratio = length / width
                
                if ratio > 2.0 and ratio > max_ratio:
                    max_ratio = ratio
                    best_cnt = cnt
                    # Izračun 3D koordinat glede na KAMERO
                    best_coords_cam = get_3d_coordinates(u_center, v_center, avg_dist)

        if best_cnt is not None:
            # 1. Koordinate glede na KAMERO
            cam_x, cam_y, cam_z = best_coords_cam
            
            # 2. Koordinate glede na AKTUATOR (Linearna preslikava/offset)
            act_x = cam_x - OFF_X
            act_y = cam_y - OFF_Y
            act_z = cam_z - OFF_Z
            
            # 3. Dejanska višina glede na tla (ob upoštevanju višine kamere)
            # V OpenCV Y os kaže navzdol, zato odštevamo cam_y
            real_height_ground = CAMERA_HEIGHT - cam_y

            # Izpis v konzolo
            print(f"\n--- REZULTATI ZA {base_name} ---")
            print(f"KAMERA [mm]:   X: {cam_x:>6.1f}, Y: {cam_y:>6.1f}, Z: {cam_z:>6.1f}")
            print(f"AKTUATOR [mm]: X: {act_x:>6.1f}, Y: {act_y:>6.1f}, Z: {act_z:>6.1f}")
            print(f"Višina od tal: {real_height_ground:.1f} mm")

            # Vizualizacija
            cv2.drawContours(display_img, [best_cnt], -1, (0, 0, 255), 3)
            info_text = f"X_act: {act_x:.0f} Y_act: {act_y:.0f} Z: {act_z:.0f}"
            cv2.putText(display_img, info_text, (20, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow("Rezultat", display_img)
        if cv2.waitKey(0) == ord('q'): break
            
    except Exception as e:
        print(f"Napaka: {e}")

cv2.destroyAllWindows()
