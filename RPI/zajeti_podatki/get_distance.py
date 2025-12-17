import cv2
import numpy as np

def rgb_to_hsv_threshold(rgb_val):
    pixel = np.uint8([[ [rgb_val[2], rgb_val[1], rgb_val[0]] ]])
    hsv_pixel = cv2.cvtColor(pixel, cv2.COLOR_BGR2HSV)[0][0]
    return hsv_pixel

# 1. Tvoji RGB podatki
rgb_low = [32, 38, 26]
rgb_high = [52, 54, 43]

# 2. Pretvorba in razširitev tolerance
hsv_low_base = rgb_to_hsv_threshold(rgb_low)
hsv_high_base = rgb_to_hsv_threshold(rgb_high)

# Razširimo razpon, da zajamemo teksturo in sence debla
lower_hsv = np.array([max(0, hsv_low_base[0] - 15), 
                      max(0, hsv_low_base[1] - 30), 
                      max(0, hsv_low_base[2] - 40)])
upper_hsv = np.array([min(180, hsv_high_base[0] + 15), 
                      min(255, hsv_high_base[1] + 120), 
                      min(255, hsv_high_base[2] + 180)])

# 3. Procesiranje slike
color_img = cv2.imread('2barva.png')
depth_map = np.load('2globina.npy')
display_img = color_img.copy()

color_img= cv2.GaussianBlur(color_img,(9,9),0)

hsv = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)
mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

# Zapolnimo luknje v S-obliki
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))

contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

best_cnt = None
max_ratio = 0

# 4. Iskanje oblike z največjim razmerjem MED KONTURAMI BLIŽJE KOT 2000 mm
valid_contours = []

for cnt in contours:
    if cv2.contourArea(cnt) < 500: 
        continue
    
    # Izračunaj razdaljo za to konturo
    mask_temp = np.zeros(mask.shape, np.uint8)
    cv2.drawContours(mask_temp, [cnt], -1, 255, -1)
    depths = depth_map[(mask_temp > 0) & (depth_map > 0)]
    
    if depths.size == 0:
        continue
    
    # Izračunaj povprečno razdaljo za konturo
    avg_distance = np.mean(depths)
    
    # Preveri, če je kontura bližje kot 2000 mm
    if avg_distance < 2000:
        valid_contours.append((cnt, avg_distance))
        
    # Debug: prikaz vseh kontur (zelena obroba)
    cv2.drawContours(display_img, [cnt], -1, (0, 255, 0), 4)

# 5. Zdaj iščemo najboljšo konturo samo med tistimi, ki so bližje kot 2000 mm
print(f"Število kontur bližjih kot 2000 mm: {len(valid_contours)}")

for cnt, distance in valid_contours:
    rect = cv2.minAreaRect(cnt)
    (x, y), (w, h), angle = rect
    length = max(w, h)
    width = min(w, h) if min(w, h) > 0 else 1
    current_ratio = length / width
    
    # Preverimo, če je to trenutno najboljša (najbolj vitka) oblika
    if current_ratio > 2.0 and current_ratio > max_ratio:
        max_ratio = current_ratio
        best_cnt = cnt
        
    print(f"Razmerje: {current_ratio:.1f}, Oddaljenost: {distance:.0f} mm")

# 6. Označevanje zmagovalne oblike
if best_cnt is not None:
    # Izračunaj natančno razdaljo za zmagovalno konturo
    mask_best = np.zeros(mask.shape, np.uint8)
    cv2.drawContours(mask_best, [best_cnt], -1, 255, -1)
    depths = depth_map[(mask_best > 0) & (depth_map > 0)]
    
    if depths.size > 0:
        # 15. percentil za sprednji del debla
        final_dist = np.percentile(depths, 15)
        
        # Narišemo debelo rdečo obrobo za zmagovalno konturo
        cv2.drawContours(display_img, [best_cnt], -1, (0, 0, 255), 4)
        
        # Izpis podatkov na sliko
        bx, by, bw, bh = cv2.boundingRect(best_cnt)
        cv2.rectangle(display_img, (bx, by-40), (bx+250, by), (0, 255, 0), -1) # Ozadje za tekst
        cv2.putText(display_img, f"NAJVECJE R: {max_ratio:.1f} | D: {final_dist:.0f}mm", 
                    (bx + 5, by - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        
        print(f"Najdeno deblo z razmerjem {max_ratio:.2f} na razdalji {final_dist:.1f} mm.")
else:
    print("Nobena oblika ne ustreza pogojem (razmerje > 2.0 in oddaljenost < 2000 mm).")

# Prikaz
cv2.imshow("Zmagovalna oblika (Max Ratio) - oddaljenost < 2000 mm", display_img)
cv2.imshow("maska", mask)

cv2.waitKey(0)
cv2.destroyAllWindows()