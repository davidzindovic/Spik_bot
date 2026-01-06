import cv2
import numpy as np
import os
import glob
import math

# --- PARAMETERS ---
FX, FY = 600.0, 600.0  
CX, CY = 320.0, 240.0  
OFF_X, OFF_Y, OFF_Z = 50.0, -100.0, 20.0  
RADIUS_OFFSET = 5.0  

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
    rgb_low, rgb_high = [32, 38, 26], [52, 54, 43]
    hsv_low_base, hsv_high_base = rgb_to_hsv_threshold(rgb_low), rgb_to_hsv_threshold(rgb_high)
    lower_hsv = np.array([np.clip(hsv_low_base[0]-15, 0, 180), np.clip(hsv_low_base[1]-30, 0, 255), np.clip(hsv_low_base[2]-40, 0, 255)], dtype=np.uint8)
    upper_hsv = np.array([np.clip(hsv_high_base[0]+15, 0, 180), np.clip(hsv_high_base[1]+120, 0, 255), np.clip(hsv_high_base[2]+180, 0, 255)], dtype=np.uint8)
    
    color_img_path = color_path
    depth_img_path = depth_path
    
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
            cv2.drawContours(display_img, [best_cnt], -1, (0, 255, 0), 2)
            cx, cy, cz = final_data['center']
            r = final_data['radius']
            angle_to_cam = math.atan2(-cz, -cx)
            points = []
            
            # --- IZRAČUN TOČK (Vrstni red popravljen na -30, 0, 30 za skladnost) ---
            for offset_deg in [-30, 0, 30]:
                off_rad = math.radians(offset_deg)
                ang = angle_to_cam + off_rad
                px, pz = cx + r * math.cos(ang), cz + r * math.sin(ang)
                safe = is_in_workspace(px, pz)
                points.append(((px, cy, pz), offset_deg, ang, safe))

            # --- DETECTION: VRSTICA Z NAPISI IN POVEZOVALNIMI ČRTAMI ---
            y_offset = 30  # Vaš parameter
            x_spread = 60  # Vaš parameter

            for i, p_data in enumerate(points):
                p, deg, ang, safe = p_data
                # Projekcija 3D točke v 2D koordinatni sistem slike
                u_img = int((p[0] * FX / p[2]) + CX)
                v_img = int((p[1] * FY / p[2]) + CY)
                
                # Narišemo piko na obodu
                cv2.circle(display_img, (u_img, v_img), 4, (0, 0, 255), -1)
                
                # Izračun pozicije napisa glede na indeks
                if i == 0:     # Točka 1 (Leva)
                    label_x = u_img - x_spread
                elif i == 1:   # Točka 2 (Sredinska)
                    label_x = u_img
                else:          # Točka 3 (Desna)
                    label_x = u_img + x_spread
                
                text_y = v_img + y_offset
                label_pos = (label_x - 10, text_y + 15) # +15 da je tekst pod koncem črte
                
                # Povezovalna črta v barvi teksta (bela z obrobo)
                # Rišemo od pike (u_img, v_img) do začetka teksta
                cv2.line(display_img, (u_img, v_img), (label_x, text_y), (0, 0, 0), 2)       # Črna obroba črte
                cv2.line(display_img, (u_img, v_img), (label_x, text_y), (255, 255, 255), 1) # Bela črta
                
                # Izpis številke s kontrastno obrobo
                cv2.putText(display_img, str(i+1), label_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3)
                cv2.putText(display_img, str(i+1), label_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            # --- TLORIS: Minimalistično ---
            top_down = np.ones((1000, 1000, 3), dtype=np.uint8) * 255
            scale, ox, oz = 0.5, 500, 850 
            vis_r_px = int(r * scale)

            # Kamera X in Napis
            cv2.line(top_down, (ox-12, oz-12), (ox+12, oz+12), (0, 0, 0), 2)
            cv2.line(top_down, (ox-12, oz+12), (ox+12, oz-12), (0, 0, 0), 2)
            cv2.putText(top_down, "KAMERA", (ox-45, oz+55), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (40, 40, 40), 2)

            # Workspace in Deblo
            ws_pts_px = np.array([[int(ox + p[0]*scale), int(oz - p[1]*scale)] for p in WORKSPACE_RECT], np.int32)
            cv2.polylines(top_down, [ws_pts_px], True, (150, 150, 150), 2)
            
            bx, bz = int(ox + cx * scale), int(oz - cz * scale)
            cv2.circle(top_down, (bx, bz), vis_r_px, (240, 240, 240), -1)
            cv2.circle(top_down, (bx, bz), vis_r_px, (100, 100, 100), 2)

            for i, (p, deg, raw_ang, safe) in enumerate(points):
                vpx, vpz = int(bx + vis_r_px * math.cos(raw_ang)), int(bz - vis_r_px * math.sin(raw_ang))
                color = (0, 160, 0) if safe else (0, 0, 220)
                cv2.circle(top_down, (vpx, vpz), 4, color, -1)
                
                # Črta in napis na tlorisu
                text_dist = vis_r_px + 70
                tx, tz = int(bx + text_dist * math.cos(raw_ang)), int(bz - text_dist * math.sin(raw_ang))
                cv2.line(top_down, (vpx, vpz), (tx, tz), (200, 200, 200), 1)
                cv2.putText(top_down, str(i+1), (tx-10, tz+10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

            cv2.imshow("Detection", display_img)
            cv2.imshow("TLORIS", top_down)
            
            # --- NON-BLOCKING INPUT ---
            print("Pritisni tipko 1, 2, ali 3 na tipkovnici (ali 'n' za naslednjo)...")
            while True:
                key = cv2.waitKey(1) & 0xFF
                if key == ord('n'):
                    return "Next"
                if key in [ord('1'), ord('2'), ord('3')]:
                    idx = int(chr(key)) - 1
                    if points[idx][3]:
                        p, d = points[idx][0], points[idx][1]
                        message = f"X:{math.floor(p[0])} Y:{math.floor(p[1])} Z:{math.floor(p[2])} ROT:{math.floor(d)}"
                        print(f"Izbrano: {message}")
                        cv2.destroyAllWindows()
                        return message
                    else:
                        print("Izbrana točka je izven delovnega območja!")

    except Exception as e: 
        print(f"Napaka: {e}")
    
    cv2.destroyAllWindows()
    return "No selection"
