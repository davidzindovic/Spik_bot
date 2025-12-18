import cv2
import numpy as np
import os
import glob

# --- NASTAVITVE PARAMETROV ---
FX, FY = 600.0, 600.0  
CX, CY = 320.0, 240.0  

# Transformacija Kamera -> Aktuator (v mm)
OFF_X, OFF_Y, OFF_Z = 50.0, -100.0, 20.0  
CAMERA_HEIGHT = 1200.0 

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
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5,5), np.uint8))
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        best_cnt = None
        max_ratio = 0
        final_data = {}

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
                
                if ratio > 1.5 and ratio > max_ratio:
                    max_ratio = ratio
                    best_cnt = cnt
                    r_mm = (pixel_width * z_front) / (2 * FX)
                    c_x_cam = (ux - CX) * z_front / FX
                    c_z_cam = z_front + r_mm
                    
                    final_data = {
                        'cam_xyz': (c_x_cam, (uy-CY)*z_front/FY, c_z_cam),
                        'radius': r_mm,
                        'rect': rect
                    }

        # --- RISANJE TLORISA ---
        top_down = np.zeros((600, 600, 3), dtype=np.uint8)
        scale = 0.25  # 1 piksel = 4 mm (da vidimo 2400mm globine)
        origin_x = 300 # Sredina slike (X=0 za kamero)
        origin_z = 550 # Spodnji del slike (Kamera)

        # 1. Nariši kamero (Trikotnik)
        cam_pts = np.array([[origin_x-10, origin_z], [origin_x+10, origin_z], [origin_x, origin_z-20]], np.int32)
        cv2.polylines(top_down, [cam_pts], True, (255, 255, 255), 2)
        cv2.putText(top_down, "KAMERA", (origin_x-30, origin_z+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # 2. Nariši izhodišče aktuatorja (Križec)
        act_orig_x = int(origin_x + (OFF_X * scale))
        act_orig_z = int(origin_z - (OFF_Z * scale))
        cv2.drawMarker(top_down, (act_orig_x, act_orig_z), (0, 255, 255), cv2.MARKER_CROSS, 15, 2)
        cv2.putText(top_down, "AKTUATOR", (act_orig_x+10, act_orig_z), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

        if best_cnt is not None:
            cx, cy, cz = final_data['cam_xyz']
            r = final_data['radius']
            
            # Izračun pikslov za tloris
            obj_x = int(origin_x + (cx * scale))
            obj_z = int(origin_z - (cz * scale))
            obj_r = int(r * scale)

            # Nariši cilinder (krog iz tlorisa)
            cv2.circle(top_down, (obj_x, obj_z), obj_r, (0, 255, 0), -1) # Poln krog
            cv2.circle(top_down, (obj_x, obj_z), obj_r, (255, 255, 255), 2) # Obroba
            
            # Linija od aktuatorja do središča objekta
            cv2.line(top_down, (act_orig_x, act_orig_z), (obj_x, obj_z), (0, 0, 255), 1)

            # Izpis razdalje na tloris
            dist_act = np.sqrt((cx-OFF_X)**2 + (cz-OFF_Z)**2)
            cv2.putText(top_down, f"Dist: {dist_act:.0f}mm", (obj_x + obj_r + 5, obj_z), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            # Osveži glavno sliko
            box = np.intp(cv2.boxPoints(final_data['rect']))
            cv2.drawContours(display_img, [box], 0, (0, 255, 0), 2)

        cv2.imshow("Detection", display_img)
        cv2.imshow("TLORIS (Top-Down View)", top_down)
        
        if cv2.waitKey(0) == ord('q'): break
            
    except Exception as e:
        print(f"Napaka: {e}")

cv2.destroyAllWindows()
