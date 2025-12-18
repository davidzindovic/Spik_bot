import cv2
import numpy as np
import os
import glob

# --- NASTAVITVE PARAMETROV ---
# Notranji parametri kamere (Zamenjaj s svojimi realnimi podatki!)
FX, FY = 600.0, 600.0  
CX, CY = 320.0, 240.0  

# Zamik aktuatorja glede na kamero (v mm)
OFF_X, OFF_Y, OFF_Z = 50.0, -100.0, 20.0  
CAMERA_HEIGHT = 1200.0 

def rgb_to_hsv_threshold(rgb_val):
    pixel = np.uint8([[ [rgb_val[2], rgb_val[1], rgb_val[0]] ]])
    hsv_pixel = cv2.cvtColor(pixel, cv2.COLOR_BGR2HSV)[0][0]
    return hsv_pixel.astype(np.int16)

def get_3d_coordinates(u, v, z):
    """Pretvorba pikslov v 3D mm (središče)"""
    if z <= 0: return 0, 0, 0
    x = (u - CX) * z / FX
    y = (v - CY) * z / FY
    return x, y, z

def get_radius_mm(radius_px, z):
    """Pretvorba radija iz pikslov v milimetre na določeni razdalji"""
    if z <= 0: return 0
    return (radius_px * z) / FX

# --- BARVNI RAZPON (z zaščito pred overflow) ---
rgb_low, rgb_high = [32, 38, 26], [52, 54, 43]
hsv_low_base = rgb_to_hsv_threshold(rgb_low)
hsv_high_base = rgb_to_hsv_threshold(rgb_high)

lower_hsv = np.array([np.clip(hsv_low_base[0]-15, 0, 180), np.clip(hsv_low_base[1]-30, 0, 255), np.clip(hsv_low_base[2]-40, 0, 255)], dtype=np.uint8)
upper_hsv = np.array([np.clip(hsv_high_base[0]+15, 0, 180), np.clip(hsv_high_base[1]+120, 0, 255), np.clip(hsv_high_base[2]+180, 0, 255)], dtype=np.uint8)

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
        hsv_img = cv2.cvtColor(cv2.GaussianBlur(color_img, (9, 9), 0), cv2.COLOR_BGR2HSV)
        mask = cv2.morphologyEx(cv2.inRange(hsv_img, lower_hsv, upper_hsv), cv2.MORPH_CLOSE, np.ones((5,5), np.uint8))
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        best_target = None # Shranimo (center_3d, radius_mm, contour)
        max_ratio = 0

        for cnt in contours:
            if cv2.contourArea(cnt) < 500: continue
            
            # Izračun globine (20. percentil za rob debla)
            c_mask = np.zeros(mask.shape, np.uint8)
            cv2.drawContours(c_mask, [cnt], -1, 255, -1)
            depth_vals = depth_map[(c_mask > 0) & (depth_map > 0)]
            if depth_vals.size == 0: continue
            avg_z = np.percentile(depth_vals, 20)
            
            if avg_z < 2000:
                # Preverjanje vitkosti (cilinder)
                rect = cv2.minAreaRect(cnt)
                w, h = rect[1]
                ratio = max(w, h) / (min(w, h) if min(w, h) > 0 else 1)
                
                if ratio > 2.0 and ratio > max_ratio:
                    max_ratio = ratio
                    # Izračun kroga, ki najbolje opisuje širino cilindra
                    (u_px, v_px), r_px = cv2.minEnclosingCircle(cnt)
                    
                    # 3D koordinate središča v mm
                    center_3d_cam = get_3d_coordinates(u_px, v_px, avg_z)
                    # Polmer v mm
                    radius_mm = get_radius_mm(r_px, avg_z)
                    
                    best_target = (center_3d_cam, radius_mm, cnt, (int(u_px), int(v_px)), int(r_px))

        if best_target:
            (cx, cy, cz), r_mm, cnt, (upx, vpx), rpx = best_target
            
            # Transformacija do aktuatorja
            ax, ay, az = cx - OFF_X, cy - OFF_Y, cz - OFF_Z
            
            print(f"\n--- CILINDER NAJDEN ({img_number}) ---")
            print(f"AKTUATOR Središče -> X: {ax:.1f}mm, Y: {ay:.1f}mm, Z: {az:.1f}mm")
            print(f"RADDIJ OBJEKTA    -> R: {r_mm:.1f}mm")
            print(f"DOSTOPNOST        -> Obod je pri X: {ax:.1f} ± {r_mm:.1f}mm")

            # Vizualizacija kroga in središča
            cv2.circle(display_img, (upx, vpx), rpx, (255, 0, 0), 2)
            cv2.circle(display_img, (upx, vpx), 5, (0, 0, 255), -1)
            cv2.putText(display_img, f"R: {r_mm:.0f}mm", (upx + 10, vpx), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        cv2.imshow("Cilinder Detection", display_img)
        if cv2.waitKey(0) == ord('q'): break
            
    except Exception as e:
        print(f"Napaka: {e}")

cv2.destroyAllWindows()
