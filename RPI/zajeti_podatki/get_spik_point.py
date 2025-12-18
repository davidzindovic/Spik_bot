import cv2
import numpy as np
import os
import glob

# --- NASTAVITVE PARAMETROV ---
FX, FY = 600.0, 600.0  # Goriščna razdalja
CX, CY = 320.0, 240.0  # Optično središče
OFF_X, OFF_Y, OFF_Z = 50.0, -100.0, 20.0  # Zamik aktuatorja (mm)
CAMERA_HEIGHT = 1200.0 # Višina kamere od tal (mm)

def rgb_to_hsv_threshold(rgb_val):
    pixel = np.uint8([[ [rgb_val[2], rgb_val[1], rgb_val[0]] ]])
    hsv_pixel = cv2.cvtColor(pixel, cv2.COLOR_BGR2HSV)[0][0]
    return hsv_pixel.astype(np.int16) # Pretvorba v int16 prepreči overflow pri odštevanju

def get_3d_coordinates(u, v, z):
    if z <= 0: return 0, 0, 0
    x = (u - CX) * z / FX
    y = (v - CY) * z / FY
    return x, y, z

# 1. Tvoji RGB podatki in izračun mej s preprečevanjem overflow-a
rgb_low = [32, 38, 26]
rgb_high = [52, 54, 43]

hsv_low_base = rgb_to_hsv_threshold(rgb_low)
hsv_high_base = rgb_to_hsv_threshold(rgb_high)

# Uporabimo np.clip za varno omejitev vrednosti med 0 in 255/180
lower_hsv = np.array([
    np.clip(hsv_low_base[0] - 15, 0, 180),
    np.clip(hsv_low_base[1] - 30, 0, 255),
    np.clip(hsv_low_base[2] - 40, 0, 255)
], dtype=np.uint8)

upper_hsv = np.array([
    np.clip(hsv_high_base[0] + 15, 0, 180),
    np.clip(hsv_high_base[1] + 120, 0, 255),
    np.clip(hsv_high_base[2] + 180, 0, 255)
], dtype=np.uint8)

color_images = glob.glob("*barva.png")
print(f"Najdenih slik: {len(color_images)}")

for color_img_path in sorted(color_images):
    img_number = color_img_path.replace("barva.png", "")
    depth_img_path = f"{img_number}globina.npy"
    
    if not os.path.exists(depth_img_path): continue
    
    try:
        color_img = cv2.imread(color_img_path)
        depth_map = np.load(depth_img_path)
        display_img = color_img.copy()
        
        # Procesiranje maske
        blurred = cv2.GaussianBlur(color_img, (9, 9), 0)
        hsv_img = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_img, lower_hsv, upper_hsv)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        best_cnt = None
        max_ratio = 0
        final_coords = None

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 500: continue
            
            # Pridobivanje globine iz celotne površine konture (bolj robustno kot en sam piksel)
            c_mask = np.zeros(mask.shape, np.uint8)
            cv2.drawContours(c_mask, [cnt], -1, 255, -1)
            depth_values = depth_map[(c_mask > 0) & (depth_map > 0)]
            
            if depth_values.size == 0: continue
            
            avg_dist = np.percentile(depth_values, 20) # Vzamemo sprednji del objekta
            
            if avg_dist < 2000:
                rect = cv2.minAreaRect(cnt)
                w, h = rect[1]
                ratio = max(w, h) / (min(w, h) if min(w, h) > 0 else 1)
                
                if ratio > 2.0 and ratio > max_ratio:
                    max_ratio = ratio
                    best_cnt = cnt
                    
                    # Izračun težišča za X, Y pozicijo
                    M = cv2.moments(cnt)
                    u = int(M["m10"] / M["m00"])
                    v = int(M["m01"] / M["m00"])
                    final_coords = get_3d_coordinates(u, v, avg_dist)

        if best_cnt is not None and final_coords:
            cx, cy, cz = final_coords
            
            # Transformacija do aktuatorja
            ax, ay, az = cx - OFF_X, cy - OFF_Y, cz - OFF_Z
            real_h = CAMERA_HEIGHT - cy

            print(f"\n--- OBJEKT NAJDEN ({img_number}) ---")
            print(f"Kamera   -> X: {cx:6.1f}, Y: {cy:6.1f}, Z: {cz:6.1f}")
            print(f"Aktuator -> X: {ax:6.1f}, Y: {ay:6.1f}, Z: {az:6.1f}")
            print(f"Višina od tal: {real_h:.1f} mm")

            cv2.drawContours(display_img, [best_cnt], -1, (0, 0, 255), 3)
            cv2.putText(display_img, f"X_act:{ax:.0f} Y_act:{ay:.0f}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            print(f"Slika {img_number}: Ni ustreznega objekta.")

        cv2.imshow("Rezultat", display_img)
        if cv2.waitKey(0) == ord('q'): break
            
    except Exception as e:
        print(f"Napaka pri {color_img_path}: {e}")

cv2.destroyAllWindows()
