import cv2
import numpy as np
import os
import glob
import math

# --- NASTAVITVE PARAMETROV ---
FX, FY = 600.0, 600.0  
CX, CY = 320.0, 240.0  

# Transformacija Kamera -> Aktuator (v mm)
OFF_X, OFF_Y, OFF_Z = 50.0, -100.0, 20.0  
RADIUS_OFFSET = 5.0  

def rgb_to_hsv_threshold(rgb_val):
    pixel = np.uint8([[ [rgb_val[2], rgb_val[1], rgb_val[0]] ]])
    hsv_pixel = cv2.cvtColor(pixel, cv2.COLOR_BGR2HSV)[0][0]
    return hsv_pixel.astype(np.int16)

# --- BARVNE MEJE ---
rgb_low, rgb_high = [32, 38, 26], [52, 54, 43]
hsv_low_base = rgb_to_hsv_threshold(rgb_low)
hsv_high_base = rgb_to_hsv_threshold(rgb_high)

lower_hsv = np.array([np.clip(hsv_low_base[0]-15, 0, 180), 
                      np.clip(hsv_low_base[1]-30, 0, 255), 
                      np.clip(hsv_low_base[2]-40, 0, 255)], dtype=np.uint8)
upper_hsv = np.array([np.clip(hsv_high_base[0]+15, 0, 180), 
                      np.clip(hsv_high_base[1]+120, 0, 255), 
                      np.clip(hsv_high_base[2]+180, 0, 255)], dtype=np.uint8)

color_images = glob.glob("*barva.png")

for color_img_path in sorted(color_images):
    img_number = color_img_path.replace("barva.png", "")
    depth_img_path = f"{img_number}globina.npy"
    if not os.path.exists(depth_img_path): continue
    
    try:
        color_img = cv2.imread(color_img_path)
        depth_map = np.load(depth_img_path)
        display_img = color_img.copy()
        
        # Maska
        blurred = cv2.GaussianBlur(color_img, (7, 7), 0)
        hsv_img = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_img, lower_hsv, upper_hsv)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7,7), np.uint8))
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_cnt, final_data = None, {}
        max_ratio = 0

        for cnt in contours:
            if cv2.contourArea(cnt) < 800: continue
            c_mask = np.zeros(mask.shape, np.uint8)
            cv2.drawContours(c_mask, [cnt], -1, 255, -1)
            depth_vals = depth_map[(c_mask > 0) & (depth_map > 0)]
            if depth_vals.size == 0: continue
            z_front = np.percentile(depth_vals, 15) 
            
            if z_front < 2000:
                rect = cv2.minAreaRect(cnt)
                (ux, uy), (w, h), angle = rect
                pixel_width = min(w, h)
                ratio = max(w, h) / (pixel_width if pixel_width > 0 else 1)
                
                if ratio > 1.2 and ratio > max_ratio:
                    max_ratio = ratio
                    best_cnt = cnt
                    r_mm = ((pixel_width * z_front) / (2 * FX)) + RADIUS_OFFSET
                    c_x_cam = (ux - CX) * z_front / FX
                    c_z_cam = z_front + r_mm
                    c_y_cam = (uy - CY) * z_front / FY
                    final_data = {'center_cam': (c_x_cam, c_y_cam, c_z_cam), 'radius': r_mm}

        if best_cnt is not None:
            cx, cy, cz = final_data['center_cam']
            r = final_data['radius']

            # --- IZRAČUN TOČK (Matematično pravilno) ---
            angle_to_cam = math.atan2(-cz, -cx)
            peripheral_points = []
            # Razmik 30 stopinj med točkami
            offsets = [0, math.radians(30), math.radians(-30)]
            
            for offset in offsets:
                ang = angle_to_cam + offset
                px = cx + r * math.cos(ang)
                pz = cz + r * math.sin(ang)
                peripheral_points.append(((px, cy, pz), math.degrees(offset), ang))

            # --- PREGLEDNA SKICA (Povečana vizualizacija) ---
            top_down = np.zeros((1000, 1000, 3), dtype=np.uint8)
            scale = 0.5
            orig_x, orig_z = 500, 950

            # Vizualni parametri (ločeni od realnosti za boljšo čitljivost)
            VIS_RADIUS = 120  # Fiksno povečan radij za skico v pikslih
            obj_px_x = int(orig_x + (cx * scale))
            obj_px_z = int(orig_z - (cz * scale))

            # 1. Naris povečanega objekta
            cv2.circle(top_down, (obj_px_x, obj_px_z), VIS_RADIUS, (35, 35, 35), -1)
            cv2.circle(top_down, (obj_px_x, obj_px_z), VIS_RADIUS, (150, 150, 150), 2)
            
            # 2. Pobarvan del oboda
            start_angle_cv = -math.degrees(peripheral_points[1][2])
            end_angle_cv = -math.degrees(peripheral_points[2][2])
            cv2.ellipse(top_down, (obj_px_x, obj_px_z), (VIS_RADIUS, VIS_RADIUS), 0, 
                       start_angle_cv, end_angle_cv, (0, 255, 0), 3)

            # 3. Točke in številke
            colors = [(0, 0, 255), (255, 100, 0), (0, 200, 255)]
            for i, (pt, deg, raw_ang) in enumerate(peripheral_points):
                # Izračun položaja pike na vizualnem (povečanem) obodu
                v_ppx = int(obj_px_x + VIS_RADIUS * math.cos(raw_ang))
                v_ppz = int(obj_px_z - VIS_RADIUS * math.sin(raw_ang))
                
                # Majhna pika na obodu
                cv2.circle(top_down, (v_ppx, v_ppz), 4, colors[i], -1)
                
                # Številka je odmaknjena še dlje stran od oboda
                label_x = int(obj_px_x + (VIS_RADIUS + 30) * math.cos(raw_ang))
                label_z = int(obj_px_z - (VIS_RADIUS + 30) * math.sin(raw_ang))
                
                cv2.putText(top_down, str(i+1), (label_x - 5, label_z + 5), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

            # 4. Kamera
            cv2.drawMarker(top_down, (orig_x, orig_z), (255, 255, 255), cv2.MARKER_TRIANGLE_UP, 15, 2)

            cv2.imshow("Detection", display_img)
            cv2.imshow("TLORIS - Povecano za vnos", top_down)
            
            print(f"\n--- TOČKE (Kot: 30°) ---")
            vnos = input("Izbira (1-3) ali 'q': ")
            if vnos in ['1', '2', '3']:
                idx = int(vnos)-1
                p, ang, _ = peripheral_points[idx]
                print(f"Točka {vnos}: X={p[0]:.1f}, Z={p[2]:.1f}, Kot={ang:.1f}°")
            elif vnos == 'q': break

        if cv2.waitKey(1) == ord('q'): break
            
    except Exception as e:
        print(f"Napaka: {e}")

cv2.destroyAllWindows()
