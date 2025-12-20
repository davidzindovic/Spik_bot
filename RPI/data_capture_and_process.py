import cv2
import numpy as np
import os
import glob
import math

#ta koda zanemarja radij pri izrisu

# --- NASTAVITVE PARAMETROV KAMERE ---
FX, FY = 600.0, 600.0  
CX, CY = 320.0, 240.0  

# --- TRANSFORMACIJA KAMERA -> AKTUATOR (v mm) ---
OFF_X, OFF_Y, OFF_Z = 50.0, -100.0, 20.0  
RADIUS_OFFSET = 5.0  

# --- DEFINICIJA DELOVNEGA OBMOČJA AKTUATORJA (v mm glede na kamero) ---
WORKSPACE_RECT = np.array([
    [OFF_X - 400, OFF_Z + 100],
    [OFF_X + 400, OFF_Z + 100],
    [OFF_X + 500, OFF_Z + 900],
    [OFF_X - 500, OFF_Z + 900]
], dtype=np.float32)

def rgb_to_hsv_threshold(rgb_val):
    pixel = np.uint8([[ [rgb_val[2], rgb_val[1], rgb_val[0]] ]])
    return cv2.cvtColor(pixel, cv2.COLOR_BGR2HSV)[0][0].astype(np.int16)

def is_in_workspace(px, pz):
    res = cv2.pointPolygonTest(WORKSPACE_RECT, (float(px), float(pz)), False)
    return res >= 0

def exec(color_path, depth_path):
    # --- BARVNE MEJE ---
    rgb_low, rgb_high = [32, 38, 26], [52, 54, 43]
    hsv_low_base, hsv_high_base = rgb_to_hsv_threshold(rgb_low), rgb_to_hsv_threshold(rgb_high)
    lower_hsv = np.array([np.clip(hsv_low_base[0]-15, 0, 180), np.clip(hsv_low_base[1]-30, 0, 255), np.clip(hsv_low_base[2]-40, 0, 255)], dtype=np.uint8)
    upper_hsv = np.array([np.clip(hsv_high_base[0]+15, 0, 180), np.clip(hsv_high_base[1]+120, 0, 255), np.clip(hsv_high_base[2]+180, 0, 255)], dtype=np.uint8)
    
    #color_images = glob.glob("*barva.png")
    
    #for color_img_path in sorted(color_images):
    #img_number = color_img_path.replace("barva.png", "")
    #depth_img_path = f"{img_number}globina.npy"

    color_img_path=color_path
    depth_img_path=depth_path
    
    #if not os.path.exists(depth_img_path): continue
    
    try:
        color_img = cv2.imread(color_img_path)
        depth_map = np.load(depth_img_path)
        display_img = color_img.copy()
        
        blurred = cv2.GaussianBlur(color_img, (7, 7), 0)
        mask = cv2.inRange(cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV), lower_hsv, upper_hsv)
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
                (ux, uy), (w, h), _ = rect
                pixel_width = min(w, h)
                ratio = max(w, h) / (pixel_width if pixel_width > 0 else 1)
                
                if ratio > 1.2 and ratio > max_ratio:
                    max_ratio = ratio
                    best_cnt = cnt
                    r_mm = ((pixel_width * z_front) / (2 * FX)) + RADIUS_OFFSET
                    final_data = {'center': ((ux-CX)*z_front/FX, (uy-CY)*z_front/FY, z_front+r_mm), 'radius': r_mm}

        if best_cnt is not None:
            # --- OZNAČEVANJE NA IZVORNI SLIKI ---
            # 1. Nariši konturo (zelena barva, debelina 2)
            cv2.drawContours(display_img, [best_cnt], -1, (0, 255, 0), 2)
            
            cx, cy, cz = final_data['center']
            r = final_data['radius']
            angle_to_cam = math.atan2(-cz, -cx)
            points = []
            
            for offset_deg in [0, 30, -30]:
                off_rad = math.radians(offset_deg)
                ang = angle_to_cam + off_rad
                px, pz = cx + r * math.cos(ang), cz + r * math.sin(ang)
                safe = is_in_workspace(px, pz)
                points.append(((px, cy, pz), offset_deg, ang, safe))
                
                # 2. Nariši točke še na izvorno sliko (projekcija iz 3D v 2D)
                # Formula: u = (x * FX / z) + CX
                u_img = int((px * FX / pz) + CX)
                v_img = int((cy * FY / pz) + CY)
                cv2.circle(display_img, (u_img, v_img), 5, (0, 0, 255) if not safe else (0, 255, 255), -1)
                cv2.putText(display_img, str(len(points)), (u_img+10, v_img), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # --- TLORIS ---
            top_down = np.zeros((1000, 1000, 3), dtype=np.uint8)
            scale, ox, oz = 0.6, 500, 950
            ws_pts_px = np.array([[int(ox + p[0]*scale), int(oz - p[1]*scale)] for p in WORKSPACE_RECT], np.int32)
            cv2.polylines(top_down, [ws_pts_px], True, (80, 80, 80), 2)
            
            vis_r = 120
            bx, bz = int(ox + cx * scale), int(oz - cz * scale)
            cv2.circle(top_down, (bx, bz), vis_r, (35, 35, 35), -1)

            for i, (p, deg, raw_ang, safe) in enumerate(points):
                vpx, vpz = int(bx + vis_r * math.cos(raw_ang)), int(bz - vis_r * math.sin(raw_ang))
                cv2.circle(top_down, (vpx, vpz), 5, (0, 255, 0) if safe else (0, 0, 255), -1)
                cv2.putText(top_down, str(i+1), (int(bx+(vis_r+35)*math.cos(raw_ang))-5, int(bz-(vis_r+35)*math.sin(raw_ang))+5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

            cv2.imshow("Detection", display_img)
            cv2.imshow("TLORIS", top_down)
            cv2.waitKey(1)

            print(f"\nIzberi 1-3 (n=naslednja, q=konec):")

            vnos = input("> ").lower()
            
            #if vnos == 'q': break
            #if vnos == 'n': continue
            if vnos in ['1', '2', '3']:
                idx = int(vnos)-1
                if points[idx][3]:
                    p, d = points[idx][0], points[idx][1]
                    #print(f"Točka {vnos}: X={p[0]:.1f}, Z={p[2]:.1f}, Rot={d}°")

        #if cv2.waitKey(1) == ord('q'): break
    except Exception as e: print(f"Napaka: {e}")
    
    cv2.destroyAllWindows()
    
    message="X:"+str(math.floor(p[0]))+" Y:"+str(math.floor(p[1]))+" Z:"+str(math.floor(p[2]))+" ROT:"+str(math.floor(d))
    print(message)
    return message
