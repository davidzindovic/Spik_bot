#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Združena koda: mobilna platforma + segmentacija z RealSense in CLIPSeg.
- Gamepad krmiljenje (hitrost, zavijanje z D-pad, načini, varnost)
- PLC komunikacija prek ADS
- RB (gumb 5) odpira/zapira okno s preview (živi prikaz)
- A (gumb 0) sproži segmentacijo na trenutnem okvirju
- X (gumb 2) sprejme rezultat (pošlje na STM32)
- B (gumb 1) zavrne rezultat (vrne v preview)
- Med odprtim oknom je vožnja zaustavljena
"""

import os
import sys
import time
import math
import threading
import numpy as np
import cv2
import torch
import pygame
import pyads
import serial
import pyrealsense2 as rs
from transformers import CLIPSegProcessor, CLIPSegForImageSegmentation

# ==============================================================================
# NASTAVITVE OKOLJA
# ==============================================================================
os.environ["CUDA_VISIBLE_DEVICES"] = ""
os.environ["HF_TOKEN"] = ""
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"

try:
    sys.modules['torchaudio'] = None
except Exception:
    pass

# ==============================================================================
# KONFIGURACIJA
# ==============================================================================
# --- Serijska povezava za STM32 ---
SERIAL_PORT = 'COM3'          # prilagodi (npr. /dev/ttyUSB0)
BAUD_RATE = 115200

# --- RealSense in segmentacija ---
MIN_DISTANCE_MM = 200
MAX_DISTANCE_MM = 400
ENABLE_AVERAGING = False      # True = povpreči več slik, False = ena slika
NUM_SAMPLES = 7               # velja le, če je ENABLE_AVERAGING = True

X_SCALE = 1.0
X_OFFSET_MM = 0
Y_SCALE = 1.0
Y_OFFSET_MM = 0
Z_HEIGHT_MM = -100            # fiksna višina

# --- PLC ---
LOKALNI_AMS_ID = "192.168.65.121.1.1"
PLC_IP = "192.168.64.200"
PLC_AMS_ID = "169.254.220.1.1.1"

# --- Globalne spremenljivke za krmiljenje (iz mobilna_drive_main.py) ---
set_speed = 50
current_angle = 0.0
old_current_angle = 0.0
delta_angle = 0.0
max_angle = 30.0
angle_step = 10
mode_change_requested = False
timestamp_temp = 0
cycle_timestamp = 0

# --- Globalne za vision ---
pipeline = None
align = None
depth_scale = None
intrinsics = None
device = "cpu"
processor = None
model = None
spatial_filter = None
hole_filling = None

# Stanja za preview/segmentacijo
vision_window_open = False     # ali je okno s preview odprto
vision_preview_mode = True     # True = prikazujemo živi prikaz, False = prikazujemo rezultat segmentacije
vision_segmentation_result = None  # (vis_img, coords, raw_color_img, contour, u, v, text_lines) ali None
vision_lock = threading.Lock()
last_color_frame = None        # zadnji barvni okvir za segmentacijo
last_depth_frame = None        # zadnji globinski okvir

# ==============================================================================
# RAZRED ZA GAMEPAD
# ==============================================================================
class SimpleGamepadPygame:
    def __init__(self):
        pygame.init()
        pygame.joystick.init()
        
        self.joystick = None
        self.x_axis = 0.0      
        self.y_axis = 0.0      # Hitrost (naprej / nazaj)
        self.e_stop = 0.0      # Varnostni gumb (E-Stop)
        self.drive_mode = 1.0  # Način vožnje (1.0 = Drive, 0.0 = Stop)
        self.plc_state = 0     # 0 = Stop, 1 = Start
        self.dead_man_pressed = False
        
        self.cruise_control_active = False
        self.selected_mode = 1.0
        self.mode_changed = False
        
        self.prev_y_axis = 0.0
        self.prev_e_stop = 0.0
        self.prev_drive_mode = 1.0
        
        self.has_changed = False
        
        # Atributi za D-pad
        self.hat_x = 0
        self.hat_y = 0
        self.prev_hat_x = 0
        self.prev_hat_y = 0
        
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            print(f"[INFO] Uspešno zaznan plošček: {self.joystick.get_name()}")
        else:
            print("[OPOZORILO] Noben igralni plošček ni zaznan!")

    def print_instructions(self):
        print("\n" + "="*70)
        print("                NAVODILA ZA UPORABO IGRALNEGA PLOŠČKA                 ")
        print("="*70)
        print(" * DESNI ANALOG (Y os)          : Hitrost NAPREJ (+) / NAZAJ (-)")
        print(" * D-PAD (Levo/Desno)          : Nastavljanje kota koles (zaklenjeno stanje)")
        print(" * GUMB LB (4)                 : DEAD MAN SWITCH (drži za delovanje)")
        print(" * GUMB A (0)                  : Vklop/izklop TEMPOMATA")
        print(" * GUMBI Y (3), X (2), B (1)   : Nastavitev načina (Y->1, X->2, B->5)")
        print(" * GUMB START (7)              : Preklop ZAGON / STOP")
        print(" * GUMB RB (5)                 : Odpre/zapre okno s preview")
        print(" * Ko je okno odprto (preview):")
        print("     A (0)    : Sproži segmentacijo")
        print("     X (2)    : Sprejmi rezultat (pošlji koordinate na STM32)")
        print("     B (1)    : Zavrni rezultat (vrni se v preview)")
        print("     RB (5)   : Zapri okno")
        print("="*70 + "\n")

    def update_and_log(self):
        global current_angle, old_current_angle, delta_angle, mode_change_requested, cycle_timestamp, timestamp_temp
        self.has_changed = False
        self.mode_changed = False
        
        pygame.event.pump()
        
        if not self.joystick:
            if pygame.joystick.get_count() > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                print(f"[INFO] Plošček ponovno povezan: {self.joystick.get_name()}")
            return

        # --- 1. Branje analogne Y osi (hitrost) ---
        if self.selected_mode != 5.0:
            if not self.cruise_control_active:
                raw_y = -self.joystick.get_axis(3)
                y_val = round(raw_y, 3) if abs(raw_y) > 0.05 else 0.0
                if y_val != self.y_axis:
                    self.y_axis = y_val
                    self.has_changed = True

        # --- Dead Man Switch ---
        self.dead_man_pressed = bool(self.joystick.get_button(4))
        if not self.dead_man_pressed:
            if self.e_stop != 1.0:
                self.e_stop = 1.0
                self.cruise_control_active = False
                self.has_changed = True
        else:
            if self.e_stop == 1.0 and not self.joystick.get_button(0):
                self.e_stop = 0.0
                self.has_changed = True

        # --- Branje D-pad ---
        hat = self.joystick.get_hat(0)
        self.prev_hat_x, self.prev_hat_y = self.hat_x, self.hat_y
        self.hat_x, self.hat_y = hat

        # --- Gumbi in D-pad za zavijanje (samo če ni mode_change_requested in ni odprto okno) ---
        if not mode_change_requested and not vision_window_open:
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == 7: # Start
                        self.drive_mode = 1.0 if self.drive_mode == 0.0 else 0.0
                        self.has_changed = True
                    elif event.button == 0 and self.dead_man_pressed: # A - tempomat
                        self.cruise_control_active = not self.cruise_control_active
                        self.has_changed = True
                        if self.cruise_control_active:
                            print(f"[TEMPOMAT] VKLOPLJEN pri hitrosti: {self.y_axis:.2f} m/s")
                        else:
                            print("[TEMPOMAT] IZKLOPLJEN")
                    elif event.button == 3: # Y -> Mode 1
                        self.selected_mode = 1.0
                        self.mode_changed = True
                        mode_change_requested = True
                        timestamp_temp = cycle_timestamp
                        print("[NAČIN] Izbran Mode 1 (Y)")
                    elif event.button == 2: # X -> Mode 2
                        self.selected_mode = 2.0
                        self.mode_changed = True
                        mode_change_requested = True
                        timestamp_temp = cycle_timestamp
                        print("[NAČIN] Izbran Mode 2 (X)")
                    elif event.button == 1: # B -> Mode 5
                        self.selected_mode = 5.0
                        self.mode_changed = True
                        mode_change_requested = True
                        timestamp_temp = cycle_timestamp
                        print("[NAČIN] Izbran Mode 5 (B)")

                # D-PAD (hat) za zavijanje (samo v načinu 1 in če ni e-stop in ni odprto okno)
                elif event.type == pygame.JOYHATMOTION and self.selected_mode == 1.0:
                    hat_x, hat_y = event.value
                    if hat_x != 0 and not self.e_stop and not vision_window_open:
                        new_angle = current_angle + (hat_x * angle_step)
                        if -max_angle <= new_angle <= max_angle:
                            current_angle = new_angle
                            delta_angle = current_angle - old_current_angle
                            self.has_changed = True
                            print(f"[D-PAD] Kot: {current_angle}° (Δ={delta_angle}°)")

        # --- Izpisi ob spremembah ---
        if self.has_changed:
            if self.e_stop != self.prev_e_stop:
                if self.e_stop == 1.0:
                    if not self.dead_man_pressed:
                        print("\n[VARNOST] DEAD MAN SWITCH SPROŠČEN! E-STOP!")
                    else:
                        print("\n[VARNOST] E-STOP AKTIVIRAN!")
                    self.plc_state = 0
                else:
                    print("\n[VARNOST] E-STOP SPROŠČEN.")
                self.prev_e_stop = self.e_stop

            if self.drive_mode != self.prev_drive_mode:
                if self.drive_mode == 1.0:
                    print("\n[VLOGA] ZAGON")
                    self.plc_state = 1
                else:
                    print("\n[VLOGA] STOP")
                    self.plc_state = 0
                self.prev_drive_mode = self.drive_mode

            if self.drive_mode == 1.0 and self.e_stop == 0.0:
                diff_y = abs(self.y_axis - self.prev_y_axis)
                if diff_y > 0.15:
                    smer = "NAPREJ" if self.y_axis > 0 else "NAZAJ" if self.y_axis < 0 else "0"
                    print(f"[JOYSTICK] Hitrost: {self.y_axis:+.2f} m/s ({smer})")
                    self.prev_y_axis = self.y_axis

# ==============================================================================
# FUNKCIJE ZA VISION (iz vision_sam3.py)
# ==============================================================================
def init_vision():
    global pipeline, align, depth_scale, intrinsics, processor, model
    global spatial_filter, hole_filling

    print("[DEBUG] Inicializacija RealSense kamere ...")
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)

    profile = pipeline.start(config)
    dev = profile.get_device()
    depth_sensor = dev.first_depth_sensor()

    # Nastavitev disparity shift za območje 200–400 mm
    try:
        if dev.supports(rs.camera_info.advanced_mode):
            adv_mode = rs.rs400_advanced_mode(dev)
            if adv_mode.is_enabled():
                depth_table = adv_mode.get_depth_table()
                depth_table.disparityShift = 55
                adv_mode.set_depth_table(depth_table)
                print("[DEBUG] Disparity Shift nastavljen na 55.")
    except Exception as e:
        print(f"[DEBUG] Napredne nastavitve niso uspele: {e}")

    if depth_sensor.supports(rs.option.visual_preset):
        try:
            depth_sensor.set_option(rs.option.visual_preset, 3)  # High Accuracy
        except:
            pass

    spatial_filter = rs.spatial_filter()
    hole_filling = rs.hole_filling_filter()
    hole_filling.set_option(rs.option.holes_fill, 1)

    align_to = rs.stream.color
    align = rs.align(align_to)
    intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    depth_scale = depth_sensor.get_depth_scale()
    print(f"[DEBUG] Kamera pripravljena (območje {MIN_DISTANCE_MM}–{MAX_DISTANCE_MM} mm).")

    # CLIPSeg model
    print("[DEBUG] Nalagam CLIPSeg model ...")
    processor = CLIPSegProcessor.from_pretrained("CIDAS/clipseg-rd64-refined")
    model = CLIPSegForImageSegmentation.from_pretrained("CIDAS/clipseg-rd64-refined")
    model.to(device)
    print("[DEBUG] CLIPSeg model uspešno naložen.")

def filter_by_distance(color_image, depth_image):
    depth_in_mm = depth_image.astype(np.float32) * depth_scale * 1000.0
    valid_mask = (depth_in_mm >= MIN_DISTANCE_MM) & (depth_in_mm <= MAX_DISTANCE_MM)
    filtered_color = color_image.copy()
    filtered_depth = depth_image.copy()
    filtered_color[~valid_mask] = 0
    filtered_depth[~valid_mask] = 0
    return filtered_color, filtered_depth

def save_full_color_image(raw_color_img, contour, u, v, text_lines):
    folder = "slike_segmentacija"
    if not os.path.exists(folder):
        os.makedirs(folder)
    i = 1
    while True:
        fpath = os.path.join(folder, f"segmentacija_{i:03d}.png")
        if not os.path.exists(fpath):
            break
        i += 1

    save_img = raw_color_img.copy()
    if contour is not None:
        cv2.drawContours(save_img, [contour], -1, (0, 255, 0), 2)
    cv2.circle(save_img, (u, v), 7, (0, 0, 255), -1)

    cv2.rectangle(save_img, (10, 10), (430, 115), (0, 0, 0), -1)
    y_pos = 30
    for line in text_lines:
        cv2.putText(save_img, line, (20, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2)
        y_pos += 25

    cv2.imwrite(fpath, save_img)
    print(f"[INFO] Slika shranjena: {fpath}")

def send_to_stm32(x, y, z, o):
    """Pošlje koordinate preko serijske povezave na STM32."""
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"[DEBUG] Serijska povezava vzpostavljena na {SERIAL_PORT}")
        cmd_x = f"X={x}\r\n"
        cmd_y = f"Y={y}\r\n"
        cmd_z = f"Z={z}\r\n"
        cmd_o = f"O={o:.2f}\r\n"
        print(f"\n[SERIJSKI] Pošiljam:")
        print(f"  {cmd_x.strip()}")
        print(f"  {cmd_y.strip()}")
        print(f"  {cmd_z.strip()}")
        print(f"  {cmd_o.strip()}")
        ser.write(cmd_x.encode()); time.sleep(0.05)
        ser.write(cmd_y.encode()); time.sleep(0.05)
        ser.write(cmd_z.encode()); time.sleep(0.05)
        ser.write(cmd_o.encode()); time.sleep(0.05)
        ser.write("GO\r\n".encode())
        print("[SERIJSKI] Ukaz GO poslan.")
        ser.close()
    except Exception as e:
        print(f"[SERIJSKI] Napaka pri pošiljanju: {e}")

def process_single_frame(color_image, depth_frame_raw):
    """Obdelava ene slike – segmentacija in izračun koordinat."""
    filtered_dframe = spatial_filter.process(depth_frame_raw)
    filtered_dframe = hole_filling.process(filtered_dframe)
    depth_image = np.asanyarray(filtered_dframe.get_data())

    color_filtered, depth_filtered = filter_by_distance(color_image, depth_image)

    rgb_image = cv2.cvtColor(color_filtered, cv2.COLOR_BGR2RGB)
    inputs = processor(text=["tree trunk"], images=[rgb_image], padding="max_length", return_tensors="pt")
    inputs = {k: v.to(device) for k, v in inputs.items()}

    with torch.no_grad():
        outputs = model(**inputs)

    logits = outputs.logits.unsqueeze(1)
    preds = torch.nn.functional.interpolate(logits, size=(720, 1280), mode="bilinear")
    probs = torch.sigmoid(preds[0][0])
    mask = (probs > 0.25).cpu().numpy().astype(bool)  # znižan prag

    contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return None

    largest_contour = max(contours, key=cv2.contourArea)
    masked_depth = np.where(mask, depth_filtered, 0)
    valid_depths = masked_depth[masked_depth > 0]

    if len(valid_depths) == 0:
        return None

    approx_depth_m = np.median(valid_depths) * depth_scale
    cy, fy = intrinsics.ppy, intrinsics.fy
    target_v = int(round(cy - ((Z_HEIGHT_MM / 1000.0) * fy / approx_depth_m)))
    v = int(np.clip(target_v, 0, 719))

    row_mask = mask[v, :]
    x_indices = np.where(row_mask)[0]
    if len(x_indices) > 0:
        u = int(np.mean(x_indices))
    else:
        M = cv2.moments(largest_contour)
        u = int(M["m10"] / M["m00"]) if M["m00"] != 0 else 640

    half_w = 3
    depth_roi = depth_filtered[max(0, v-half_w):min(720, v+half_w+1),
                               max(0, u-half_w):min(1280, u+half_w+1)]
    valid_roi = depth_roi[depth_roi > 0]
    depth_raw = np.median(valid_roi) if len(valid_roi) > 0 else np.median(valid_depths)
    depth_in_meters = depth_raw * depth_scale

    rs_coords = rs.rs2_deproject_pixel_to_point(intrinsics, [u, v], depth_in_meters)
    raw_y_mm = rs_coords[2] * 1000
    raw_x_mm = -rs_coords[0] * 1000

    return u, v, raw_x_mm, raw_y_mm, largest_contour, color_filtered, color_image

def run_measurement(color_frame, depth_frame):
    """Izvede meritev na enem okvirju (brez povprečenja)."""
    c_img = np.asanyarray(color_frame.get_data())
    res = process_single_frame(c_img, depth_frame)
    if res is None:
        return None
    u, v, raw_x, raw_y, contour, filtered_c_img, raw_color_img = res
    vis_img = filtered_c_img.copy()
    mode_label = "POSAMEZNA SLIKA"

    # Narišemo konturo in točko na sliko za prikaz
    if contour is not None:
        cv2.drawContours(vis_img, [contour], -1, (0, 255, 0), 2)
    cv2.circle(vis_img, (u, v), 7, (0, 0, 255), -1)

    # Kalibracija in izračun kota
    X_coord_mm = int(round((raw_x * X_SCALE) + X_OFFSET_MM))
    Y_coord_mm = int(round((raw_y * Y_SCALE) + Y_OFFSET_MM))
    Z_coord_mm = int(Z_HEIGHT_MM)

    radial_angle_rad = math.atan2(X_coord_mm, Y_coord_mm)
    radial_angle_deg = math.degrees(radial_angle_rad)
    orientation = float(np.clip(radial_angle_deg, -30.0, 30.0))

    text_x = f"X ({mode_label}): {X_coord_mm} mm"
    text_y = f"Y: {Y_coord_mm} mm"
    text_z = f"Z: {Z_coord_mm} mm"
    text_o = f"O: {orientation:.2f} deg"

    # Narišemo informacije na sliko
    cv2.rectangle(vis_img, (10, 10), (550, 135), (0, 0, 0), -1)
    cv2.putText(vis_img, text_x, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2)
    cv2.putText(vis_img, text_y, (20, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2)
    cv2.putText(vis_img, text_z, (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 255), 2)
    cv2.putText(vis_img, text_o, (20, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2)

    text_lines = [text_x, text_y, text_z, text_o]
    coords = (X_coord_mm, Y_coord_mm, Z_coord_mm, orientation)

    return vis_img, coords, raw_color_img, contour, u, v, text_lines

def create_preview_frame(color_image, depth_image):
    """Ustvari sliko za preview: levo barva, desno globina (filtrirano po razdalji)."""
    color_filt, depth_filt = filter_by_distance(color_image, depth_image)
    depth_colormap = cv2.applyColorMap(
        cv2.convertScaleAbs(depth_filt, alpha=0.08), cv2.COLORMAP_JET
    )
    preview = np.hstack((color_filt, depth_colormap))
    # Dodamo napise
    cv2.putText(preview, f"COLOR ({MIN_DISTANCE_MM}-{MAX_DISTANCE_MM}mm)", (30, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(preview, f"DEPTH ({MIN_DISTANCE_MM}-{MAX_DISTANCE_MM}mm)", (1310, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    # Napis za gumbe
    cv2.putText(preview, "A: Segmentacija | X: Sprejmi | B: Zavrni | RB: Zapri", (30, 700),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    return preview

# ==============================================================================
# GLAVNI PROGRAM
# ==============================================================================
def main():
    global pipeline, vision_window_open, vision_preview_mode, vision_segmentation_result
    global cycle_timestamp, mode_change_requested, timestamp_temp
    global current_angle, old_current_angle, delta_angle
    global last_color_frame, last_depth_frame

    # --- Inicializacija PLC ---
    print("Povezovanje s PLC-jem...")
    plc = pyads.Connection(PLC_AMS_ID, pyads.PORT_TC3PLC1, PLC_IP)
    try:
        plc.open()
        print("ADS povezava uspešna.")
    except Exception as e:
        print(f"[OPOZORILO] PLC ni dosegljiv: {e}")
        plc = None

    # --- Inicializacija kamere in modela ---
    try:
        init_vision()
    except Exception as e:
        print(f"[NAPAKA] Inicializacija visiona ni uspela: {e}")
        print("Program bo deloval brez segmentacije.")
        pipeline = None

    # --- Gamepad ---
    gamepad = SimpleGamepadPygame()
    gamepad.print_instructions()

    # --- OpenCV okno (sprva zaprto) ---
    cv2.namedWindow("Segmentacija", cv2.WINDOW_NORMAL)
    cv2.destroyWindow("Segmentacija")

    # --- Spremenljivke za zanko ---
    loop_rate = 0.02  # 50 Hz
    prev_plc_state = -1
    prev_rb_state = False
    prev_a_state = False
    prev_x_state = False
    prev_b_state = False

    print("Krmiljenje aktivno. Pritisnite Ctrl+C za izhod.")

    try:
        while True:
            start_time = time.time()
            cycle_timestamp += 1.0

            # ------------------------------------------------------------
            # 1. Zajem slik iz kamere (če je na voljo)
            # ------------------------------------------------------------
            color_frame = None
            depth_frame = None
            if pipeline is not None:
                try:
                    frames = pipeline.wait_for_frames()
                    aligned_frames = align.process(frames)
                    color_frame = aligned_frames.get_color_frame()
                    depth_frame = aligned_frames.get_depth_frame()
                    if color_frame and depth_frame:
                        last_color_frame = color_frame
                        last_depth_frame = depth_frame
                except Exception:
                    pass

            # ------------------------------------------------------------
            # 2. Branje gamepada
            # ------------------------------------------------------------
            gamepad.update_and_log()

            # ------------------------------------------------------------
            # 3. Obdelava gumbov za upravljanje segmentacije
            # ------------------------------------------------------------
            rb_pressed = False
            a_pressed = False
            x_pressed = False
            b_pressed = False
            if gamepad.joystick:
                rb_pressed = bool(gamepad.joystick.get_button(5))
                a_pressed = bool(gamepad.joystick.get_button(0))
                x_pressed = bool(gamepad.joystick.get_button(2))
                b_pressed = bool(gamepad.joystick.get_button(1))

            # --- RB: odpri/zapri okno ---
            if rb_pressed and not prev_rb_state:
                if not vision_window_open:
                    # Odpri okno
                    if pipeline is not None and last_color_frame is not None:
                        print("[PREVIEW] Odpiram okno s preview.")
                        vision_window_open = True
                        vision_preview_mode = True
                        vision_segmentation_result = None
                        cv2.namedWindow("Segmentacija", cv2.WINDOW_NORMAL)
                    else:
                        print("[PREVIEW] Kamera ni na voljo.")
                else:
                    # Zapri okno
                    print("[PREVIEW] Zapiram okno.")
                    vision_window_open = False
                    vision_preview_mode = True
                    vision_segmentation_result = None
                    cv2.destroyWindow("Segmentacija")
            prev_rb_state = rb_pressed

            # --- Če je okno odprto, obdelaj ostale gumbe ---
            if vision_window_open:
                # A: sproži segmentacijo (samo v preview načinu)
                if a_pressed and not prev_a_state and vision_preview_mode:
                    if last_color_frame is not None and last_depth_frame is not None:
                        print("[SEGMENTACIJA] Zaganjam segmentacijo...")
                        # Izvedemo meritev sinhrono (ker je hitra, lahko blokiramo)
                        res = run_measurement(last_color_frame, last_depth_frame)
                        if res is not None:
                            vis_img, coords, raw_color_img, contour, u, v, text_lines = res
                            vision_segmentation_result = res
                            vision_preview_mode = False
                            print("[SEGMENTACIJA] Rezultat prikazan. Uporabi X za sprejem, B za zavrnitev.")
                        else:
                            print("[SEGMENTACIJA] Meritev ni uspela (ni zaznanega objekta).")
                            # Prikažemo obvestilo na preview
                            preview = create_preview_frame(
                                np.asanyarray(last_color_frame.get_data()),
                                np.asanyarray(last_depth_frame.get_data())
                            )
                            cv2.putText(preview, "NI ZAZNANEGA OBJEKTA!", (300, 360),
                                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
                            cv2.imshow("Segmentacija", preview)
                    else:
                        print("[SEGMENTACIJA] Ni na voljo slik.")
                prev_a_state = a_pressed

                # X: sprejmi rezultat (pošlji na STM32) in vrni v preview
                if x_pressed and not prev_x_state and not vision_preview_mode:
                    if vision_segmentation_result is not None:
                        _, coords, _, _, _, _, _ = vision_segmentation_result
                        x, y, z, o = coords
                        send_to_stm32(x, y, z, o)
                        print("[SEGMENTACIJA] Koordinate poslane. Vračam se v preview.")
                        vision_preview_mode = True
                        vision_segmentation_result = None
                    else:
                        print("[SEGMENTACIJA] Ni rezultata za sprejeti.")
                prev_x_state = x_pressed

                # B: zavrni rezultat in vrni v preview
                if b_pressed and not prev_b_state and not vision_preview_mode:
                    print("[SEGMENTACIJA] Rezultat zavrnjen. Vračam se v preview.")
                    vision_preview_mode = True
                    vision_segmentation_result = None
                prev_b_state = b_pressed

                # --- Prikaz v oknu ---
                if vision_window_open:
                    if vision_preview_mode:
                        # Prikaži preview
                        if last_color_frame is not None and last_depth_frame is not None:
                            c_img = np.asanyarray(last_color_frame.get_data())
                            d_img = np.asanyarray(last_depth_frame.get_data())
                            preview = create_preview_frame(c_img, d_img)
                            cv2.imshow("Segmentacija", preview)
                    else:
                        # Prikaži rezultat segmentacije
                        if vision_segmentation_result is not None:
                            vis_img, _, _, _, _, _, _ = vision_segmentation_result
                            cv2.imshow("Segmentacija", vis_img)
                    cv2.waitKey(1)

            # ------------------------------------------------------------
            # 4. Pošiljanje PLC podatkov (med odprtim oknom zaustavimo vožnjo)
            # ------------------------------------------------------------
            # WATCHDOG
            if plc is not None:
                try:
                    plc.write_by_name('MAIN.CartContol.data.timeStamp', cycle_timestamp, pyads.PLCTYPE_LREAL)
                except Exception as e:
                    print(f"[PLC] Napaka pri timestampu: {e}")

            # Če je okno odprto, pošljemo zaustavitev
            if vision_window_open:
                try:
                    if plc is not None:
                        master_data = [
                            cycle_timestamp,   # 1
                            0.0,               # 2 masterSwich OFF
                            0.0,               # 3 mode stop
                            0.0,               # 4 hitrost %
                            2.0, 2.0, 2.0, 2.0, # 5-8 hub stop
                            2.0, 2.0, 2.0, 2.0, # 9-12 max stop
                            2.0, 2.0, 2.0,     # 13-15 dodatna oprema stop
                            0.0                # 16 steeringMode
                        ]
                        plc.write_by_name("MAIN.MasterContol.dataArray", master_data, pyads.PLCTYPE_LREAL*16)
                        plc.write_by_name("MAIN.CartContol.data.hitrost_ms", 0.0, pyads.PLCTYPE_LREAL)
                except Exception as e:
                    print(f"[PLC] Napaka pri pošiljanju zaustavitve: {e}")
            else:
                # Normalno krmiljenje (iz mobilna_drive_main.py)
                if gamepad.plc_state != prev_plc_state:
                    try:
                        if plc is not None:
                            if gamepad.plc_state == 1:
                                plc.write_by_name('MAIN.MasterContol.data.mode', 1, pyads.PLCTYPE_LREAL)
                                plc.write_by_name('MAIN.MasterContol.data.masterSwich', 1, pyads.PLCTYPE_LREAL)
                                plc.write_by_name('MAIN.MasterContol.data.steeringMode', gamepad.selected_mode, pyads.PLCTYPE_LREAL)
                                print(f"PLC: START (mode {gamepad.selected_mode})")
                            else:
                                plc.write_by_name('MAIN.MasterContol.data.mode', 0, pyads.PLCTYPE_LREAL)
                                plc.write_by_name('MAIN.MasterContol.data.masterSwich', 0, pyads.PLCTYPE_LREAL)
                                print("PLC: STOP")
                            prev_plc_state = gamepad.plc_state
                    except Exception as e:
                        print(f"[PLC] Napaka pri vpisu stanja: {e}")

                # Ob spremembah (hitrost, kot) pošiljamo podatke
                if gamepad.has_changed and not mode_change_requested:
                    if gamepad.e_stop == 0 and gamepad.drive_mode == 1.0 and gamepad.selected_mode != 5.0:
                        master_data = [
                            cycle_timestamp,
                            1.0,
                            1.0,
                            20.0,
                            1.0, 1.0, 1.0, 1.0,
                            1.0, 1.0, 1.0, 1.0,
                            0.0, 0.0, 0.0,
                            gamepad.selected_mode
                        ]
                        try:
                            if plc is not None:
                                plc.write_by_name("MAIN.MasterContol.dataArray", master_data, pyads.PLCTYPE_LREAL*16)
                                plc.write_by_name("MAIN.CartContol.dataArray[0]", cycle_timestamp, pyads.PLCTYPE_LREAL)
                                plc.write_by_name("MAIN.CartContol.dataArray[1]", gamepad.y_axis, pyads.PLCTYPE_LREAL)
                                # Pošiljanje kota (delta_angle) samo ob spremembi
                                if abs(delta_angle) >= angle_step:
                                    print(f"Zavijam na PLC -> Kot: {current_angle}°")
                                    plc.write_by_name('STEERING.WHEEL_1_ANG_REF', current_angle, pyads.PLCTYPE_LREAL)
                                    old_current_angle = current_angle
                                    delta_angle = 0.0
                        except Exception as e:
                            print(f"[PLC] Napaka pri pisanju: {e}")
                    else:
                        # Če ni pogojev za vožnjo, pošljemo zaustavitev
                        try:
                            if plc is not None:
                                master_data = [
                                    cycle_timestamp,
                                    0.0, 0.0, 0.0,
                                    2.0, 2.0, 2.0, 2.0,
                                    2.0, 2.0, 2.0, 2.0,
                                    2.0, 2.0, 2.0,
                                    0.0
                                ]
                                plc.write_by_name("MAIN.MasterContol.dataArray", master_data, pyads.PLCTYPE_LREAL*16)
                                plc.write_by_name("MAIN.CartContol.data.hitrost_ms", 0.0, pyads.PLCTYPE_LREAL)
                        except Exception as e:
                            print(f"[PLC] Napaka pri pisanju zaustavitve: {e}")

                # Obdelava mode_change_requested (počakamo, da se kolesa ustavijo)
                if mode_change_requested:
                    try:
                        if plc is not None:
                            if (cycle_timestamp - timestamp_temp) >= 50:
                                plc.write_by_name('MAIN.MasterContol.data.steeringMode', gamepad.selected_mode, pyads.PLCTYPE_LREAL)
                                print(f"PLC: mode nastavljen na {gamepad.selected_mode}")
                                mode_change_requested = False
                            else:
                                # Med čakanjem pošljemo hitrost 0
                                plc.write_by_name('MAIN.CartContol.data.hitrost_ms', 0.0, pyads.PLCTYPE_LREAL)
                    except Exception as e:
                        print(f"[PLC] Napaka pri pisanju mode: {e}")

            # ------------------------------------------------------------
            # 5. OpenCV osveževanje okna (waitKey) – že narejeno zgoraj
            # ------------------------------------------------------------

            # ------------------------------------------------------------
            # 6. Zakasnitev za želeno frekvenco
            # ------------------------------------------------------------
            elapsed = time.time() - start_time
            time.sleep(max(0.0, loop_rate - elapsed))

    except KeyboardInterrupt:
        print("\n[INFO] Prekinitev s tipkovnico.")
    finally:
        # Zapiranje virov
        if pipeline is not None:
            pipeline.stop()
        cv2.destroyAllWindows()
        if plc is not None:
            plc.close()
        pygame.quit()
        print("[INFO] Program končan.")

if __name__ == "__main__":
    main()
