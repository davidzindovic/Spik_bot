import cv2
import numpy as np
import os
import glob

# --- NASTAVITVE PARAMETROV ---
# Notranji parametri kamere (Zamenjaj s svojimi dejanskimi vrednostmi)
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
            
            # Pridobivanje globine (15. percentil - sprednji rob)
            c_mask = np.zeros(mask.shape, np.uint8)
            cv2.drawContours(c_mask, [cnt], -1, 255, -1)
            depth_vals = depth_map[(c_mask > 0) & (depth_map > 0)]
            if depth_vals.size == 0: continue
            z_front = np.percentile(depth_vals, 15) 
            
            if z_front < 2000:
                rect = cv2.minAreaRect(cnt)
                (ux, uy), (w, h), angle = rect
                
                pixel_width = min(w, h)
                pixel_height = max(w, h)
                ratio = pixel_height / pixel_width
                
                if ratio > 1.5 and ratio > max_ratio:
                    max_ratio = ratio
                    best_cnt = cnt
                    
                    # IZRAČUN GEOMETRIJE VALJA
                    # Radij iz pikslov pretvorimo v mm (širina / 2)
                    r_mm = (pixel_width * z_front) / (2 * FX)
                    
                    # Središče (X, Y) na sprednji ploskvi
                    x_front_mm = (ux - CX) * z_front / FX
                    y_front_mm = (uy - CY) * z_front / FY
                    
                    # Središče VALJA (X, Y ostajata, Z se zamakne za radij v globino)
                    # To velja, če kamera gleda približno pravokotno na deblo
                    c_x_cam = x_front_mm
                    c_y_cam = y_front_mm
                    c_z_cam = z_front + r_mm
                    
                    final_data = {
                        'cam_xyz': (c_x_cam, c_y_cam, c_z_cam),
                        'radius': r_mm,
                        'rect': rect
                    }

        if best_cnt is not None:
            cx, cy, cz = final_data['cam_xyz']
            r = final_data['radius']
            
            # Koordinatni sistem aktuatorja
            ax, ay, az = cx - OFF_X, cy - OFF_Y, cz - OFF_Z
            
            print(f"\n--- REZULTAT: {img_number} ---")
            print(f"AKTUATOR SREDIŠČE: X={ax:6.1f}, Y={ay:6.1f}, Z={az:6.1f} [mm]")
            print(f"RADIJ OBJEKTA:    R={r:6.1f} [mm]")
            print(f"ROB OBJEKTA (Z):  Najbližja točka: {az - r:.1f} mm")

            # Izris
            # POPRAVEK: np.int0 -> np.intp ali .astype(int)
            box = cv2.boxPoints(final_data['rect'])
            box = np.intp(box) 
            cv2.drawContours(display_img, [box], 0, (0, 255, 0), 2)
            
            # Prikaz središčne točke
            cv2.circle(display_img, (int(final_data['rect'][0][0]), int(final_data['rect'][0][1])), 5, (0,0,255), -1)
            cv2.putText(display_img, f"R: {r:.1f}mm", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        cv2.imshow("Cylinder Detection", display_img)
        if cv2.waitKey(0) == ord('q'): break
            
    except Exception as e:
        print(f"Napaka pri obdelavi: {e}")

cv2.destroyAllWindows()
