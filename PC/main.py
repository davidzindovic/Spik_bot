#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
main.py – Združena koda za krmiljenje mobilne platforme in vizualno segmentacijo z RealSense.
"""

import os
import sys
import time
import math
import threading
import queue

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
# UVOZ KNJIŽNIC (s samodejno namestitvijo)
# ==============================================================================
def install_and_import(package, import_name=None, extra=None):
    try:
        if import_name is None:
            import_name = package
        return __import__(import_name)
    except ImportError:
        print(f"[NAPAKA] Manjka '{package}'. Nameščam ...")
        cmd = f'python -m pip install {package}'
        if extra:
            cmd += f'[{extra}]'
        os.system(cmd)
        return __import__(import_name)

try:
    sys.modules['torchaudio'] = None
except Exception:
    pass

# ==============================================================================
# PREVERJANJE IN UVOZ KNJIŽNIC
# ==============================================================================
import time
import pygame
import pyads
try:
    import pyrealsense2 as rs
except ImportError:
    print("[NAPAKA] Manjka knjižnica 'pyrealsense2'. Nameščam jo z: pip install pyrealsense2")
    os.system('python -m pip install pyrealsense2')
finally:
    import pyrealsense2 as rs

try:
    import numpy as np
except ImportError:
    print("[NAPAKA] Manjka knjižnica 'numpy'. Nameščam jo z: pip install numpy")
    os.system('python -m pip install pip install numpy')
finally:
    import numpy as np

try:
    import cv2
except ImportError:
    print("[NAPAKA] Manjka knjižnica 'opencv-python'. Nameščam jo z: pip install opencv-python")
    os.system('python -m pip install pip install opencv-python')
finally:
    import cv2

try:
    import serial
except ImportError:
    print("[NAPAKA] Manjka knjižnica 'pyserial'. Nameščam jo z: pip install pyserial")
    os.system('python -m pip install pip install pyserial')
finally:
    import serial

try:
    import torch
    from transformers import CLIPSegProcessor, CLIPSegForImageSegmentation
except ImportError as e:
    print("\n" + "="*80)
    print(f"[NAPAKA] Težava pri uvozu PyTorch/Transformers: {e}")
    os.system('python -m pip install pip uninstall -y torch torchvision torchaudio')
    os.system('python -m pip install pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu')
    os.system('python -m pip install pip install transformers')
finally:
    import torch
    from transformers import CLIPSegProcessor, CLIPSegForImageSegmentation

# ==============================================================================
# KONFIGURACIJA
# ==============================================================================
# --- Serijska povezava za STM32 ---
SERIAL_PORT = 'COM3'          # prilagodi (npr. /dev/ttyUSB0)
BAUD_RATE = 115200

# --- RealSense in segmentacija ---
MIN_DISTANCE_MM = 200
MAX_DISTANCE_MM = 400
ENABLE_AVERAGING = False      # vedno uporabimo eno sliko
NUM_SAMPLES = 7               # ni v uporabi, če je ENABLE_AVERAGING = False

X_SCALE = 1.0
X_OFFSET_MM = 0
Y_SCALE = 1.0
Y_OFFSET_MM = 0
Z_HEIGHT_MM = -100            # fiksna višina

# --- PLC ---
LOKALNI_AMS_ID = "192.168.65.121.1.1"
PLC_IP = "192.168.64.200"
PLC_AMS_ID = "169.254.220.1.1.1"

# ==============================================================================
# GLOBALNE SPREMENLJIVKE ZA VISION
# ==============================================================================
vision_active = False          # ali je segmentacija v teku ali prikaz rezultata
vision_ready = False           # ali je rezultat pripravljen za prikaz
vision_result = None           # (vis_img, coords, raw_color_img, contour, u, v, text_lines) ali None
vision_lock = threading.Lock()

# ==============================================================================
# RAZRED ZA GAMEPAD (iz mobilna_drive_main.py, prilagojen za integracijo)
# ==============================================================================
class SimpleGamepadPygame:
    def __init__(self):
        pygame.init()
        pygame.joystick.init()
        self.joystick = None
        self.x_axis = 0.0
        self.y_axis = 0.0
        self.e_stop = 0.0
        self.drive_mode = 1.0
        self.plc_state = 0
        self.dead_man_pressed = False
        self.cruise_control_active = False
        self.selected_mode = 1.0
        self.mode_changed = False

        self.prev_y_axis = 0.0
        self.prev_e_stop = 0.0
        self.prev_drive_mode = 1.0
        self.has_changed = False

        # spremenljivke za D‑pad (uporabljajo se v glavni zanki)
        self.hat_x = 0
        self.hat_y = 0
        self.prev_hat_x = 0
        self.prev_hat_y = 0

        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            print(f"[INFO] Zaznan plošček: {self.joystick.get_name()}")
        else:
            print("[OPOZORILO] Noben igralni plošček ni zaznan!")

    def print_instructions(self):
        print("\n" + "="*70)
        print("                NAVODILA ZA UPORABO IGRALNEGA PLOŠČKA                 ")
        print("="*70)
        print(" * DESNI ANALOG (Y os)          : Hitrost NAPREJ (+) / NAZAJ (-)")
        print(" * D-PAD (Levo/Desno)           : Nastavitev kota koles (samo v načinu 1)")
        print(" * GUMB LB (4)                  : DEAD MAN SWITCH (drži za delovanje)")
        print(" * GUMB A (0)                   : Tempomat (zakleni hitrost)")
        print(" * GUMB Y (3), X (2), B (1)     : Izbira načina (1, 2, 5)")
        print(" * GUMB START (7)               : Zagon / Zaustavitev")
        print(" * GUMB RB (5)                  : Sproži segmentacijo / prekliči rezultat")
        print(" * Ko je prikazana segmentacija:")
        print("     D-PAD GOR    → pošlji koordinate na STM32")
        print("     D-PAD DOL    → shrani sliko")
        print("     RB           → prekliči")
        print("="*70 + "\n")

    def update_and_log(self):
        """Neblokirajoče branje stanja ploščka. Upošteva globalno vision_active."""
        global vision_active, mode_change_requested
        self.has_changed = False
        self.mode_changed = False

        pygame.event.pump()

        if not self.joystick:
            if pygame.joystick.get_count() > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                print(f"[INFO] Plošček ponovno povezan: {self.joystick.get_name()}")
            return

        # --- Dead Man Switch (vedno aktiven) ---
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

        # Če je segmentacija aktivna, ne spreminjamo krmilnih spremenljivk
        if vision_active:
            # Še vedno beremo hat za morebitno uporabo v glavni zanki
            hat = self.joystick.get_hat(0)
            self.prev_hat_x, self.prev_hat_y = self.hat_x, self.hat_y
            self.hat_x, self.hat_y = hat
            return

        # --- Branje analognih osi (samo če ni tempomata) ---
        if self.selected_mode != 5.0:
            if not self.cruise_control_active:
                raw_y = -self.joystick.get_axis(3)
                y_val = round(raw_y, 3) if abs(raw_y) > 0.05 else 0.0
                if y_val != self.y_axis:
                    self.y_axis = y_val
                    self.has_changed = True

        # --- Obdelava diskretnih dogodkov ---
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                if event.button == 7:          # Start
                    self.drive_mode = 1.0 if self.drive_mode == 0.0 else 0.0
                    self.has_changed = True
                elif event.button == 0 and self.dead_man_pressed:  # A – tempomat
                    self.cruise_control_active = not self.cruise_control_active
                    self.has_changed = True
                    if self.cruise_control_active:
                        print(f"[TEMPOMAT] VKLOPLJEN pri hitrosti: {self.y_axis:.2f} m/s")
                    else:
                        print("[TEMPOMAT] IZKLOPLJEN")
                elif event.button == 3:        # Y – mode 1
                    self.selected_mode = 1.0
                    self.mode_changed = True
                    mode_change_requested=True
                    print("[NAČIN] Mode 1 (Y)")
                elif event.button == 2:        # X – mode 2
                    self.selected_mode = 2.0
                    self.mode_changed = True
                    mode_change_requested=True
                    print("[NAČIN] Mode 2 (X)")
                elif event.button == 1:        # B – mode 5
                    self.selected_mode = 5.0
                    self.mode_changed = True
                    mode_change_requested=True
                    print("[NAČIN] Mode 5 (B)")

        # --- D‑pad (samo če ni vision_active) ---
        hat = self.joystick.get_hat(0)
        self.prev_hat_x, self.prev_hat_y = self.hat_x, self.hat_y
        self.hat_x, self.hat_y = hat

        if self.selected_mode == 1.0 and not self.e_stop and hat[0] != 0:
            # obdelava kota (originalna logika) – v tej integraciji se ne uporablja neposredno,
            # ker kot krmilimo preko PLC; ohranimo za morebitno uporabo
            pass

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
# FUNKCIJE ZA VISION (iz vision_sam3.py, prilagojene)
# ==============================================================================
# Globalne spremenljivke za kamero in model
pipeline = None
align = None
depth_scale = None
intrinsics = None
device = "cpu"
processor = None
model = None
spatial_filter = None
hole_filling = None
mode_change_requested=False
temp_timestamp=0

def init_vision():
    """Inicializacija RealSense in CLIPSeg modela."""
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
    mask = (probs > 0.35).cpu().numpy().astype(bool)

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


def run_measurement(color_frame=None, depth_frame=None):
    """
    Izvede meritev na podanih okvirjih (ali, če je ENABLE_AVERAGING True, povpreči več slik).
    Vrne (vis_img, coords, raw_color_img, contour, u, v, text_lines) ali None.
    """
    if ENABLE_AVERAGING:
        print(f"[DEBUG] Povprečenje vklopljeno ({NUM_SAMPLES} slik)")
        x_list, y_list = [], []
        last_vis_img = None
        last_raw_color = None
        last_contour = None
        last_u, last_v = 640, 360

        for _ in range(NUM_SAMPLES):
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            c_frame = aligned_frames.get_color_frame()
            d_frame = aligned_frames.get_depth_frame()
            if not c_frame or not d_frame:
                continue
            c_img = np.asanyarray(c_frame.get_data())
            res = process_single_frame(c_img, d_frame)
            if res is not None:
                u, v, raw_x, raw_y, contour, filtered_c_img, raw_color_img = res
                x_list.append(raw_x)
                y_list.append(raw_y)
                last_vis_img = filtered_c_img.copy()
                last_raw_color = raw_color_img.copy()
                last_contour = contour
                last_u, last_v = u, v

        if not x_list:
            return None

        avg_raw_x = np.median(x_list)
        avg_raw_y = np.median(y_list)
        u, v = last_u, last_v
        vis_img = last_vis_img
        raw_color_img = last_raw_color
        contour = last_contour
        raw_x, raw_y = avg_raw_x, avg_raw_y
        mode_label = f"POVPREČJE ({len(x_list)} slik)"
    else:
        if color_frame is None or depth_frame is None:
            return None
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


# ==============================================================================
# NIT ZA ASINHRONO SEGMENTACIJO
# ==============================================================================
def vision_thread_func(color_frame, depth_frame):
    """Ciljna funkcija za nit – izvede meritev in nastavi globalne rezultate."""
    global vision_ready, vision_result, vision_active
    try:
        res = run_measurement(color_frame, depth_frame)
        with vision_lock:
            vision_result = res
            vision_ready = True
    except Exception as e:
        print(f"[SEGMENTACIJA] Napaka: {e}")
        with vision_lock:
            vision_result = None
            vision_ready = True
    finally:
        # Po koncu niti ne spreminjamo vision_active; to bo glavna zanka obravnavala
        pass


# ==============================================================================
# GLAVNI PROGRAM
# ==============================================================================
def main():
    global vision_active, vision_ready, vision_result, pipeline, mode_change_requested, temp_timestamp

    # --- Inicializacija serijske povezave (ni nujno, da je odprta) ---
    try:
        ser_test = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        ser_test.close()
        print(f"[DEBUG] Serijski port {SERIAL_PORT} dosegljiv.")
    except:
        print(f"[OPOZORILO] Serijski port {SERIAL_PORT} ni na voljo – pošiljanje na STM32 ne bo delovalo.")

    # --- Inicializacija PLC ---
    print("Povezovanje s PLC-jem ...")
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

    # --- OpenCV okno ---
    cv2.namedWindow("Segmentacija", cv2.WINDOW_NORMAL)
    cv2.startWindowThread()
    cv2.destroyWindow("Segmentacija")  # sprva zapremo

    # --- Spremenljivke za zanko ---
    cycle_timestamp = 0.0
    loop_rate = 0.02  # 50 Hz
    prev_plc_state = -1
    old_plc_mode = 0

    # Stanje gumbov za robno detekcijo
    prev_rb_state = False
    prev_hat_y = 0
    prev_hat_x = 0

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
                except Exception:
                    pass

            # ------------------------------------------------------------
            # 2. Branje gamepada
            # ------------------------------------------------------------
            gamepad.update_and_log()

            # ------------------------------------------------------------
            # 3. Obdelava RB gumba (gumb 5) – zagon / preklic segmentacije
            # ------------------------------------------------------------
            rb_pressed = False
            if gamepad.joystick:
                rb_pressed = bool(gamepad.joystick.get_button(5))

            if rb_pressed and not prev_rb_state:  # robni prehod 0→1
                if not vision_active:
                    # Zaženi segmentacijo v ozadju
                    if pipeline is not None and color_frame is not None and depth_frame is not None:
                        print("[SEGMENTACIJA] Zagon meritve ...")
                        vision_active = True
                        vision_ready = False
                        vision_result = None
                        # Kopiramo okvirje, da jih nit uporabi
                        thr = threading.Thread(target=vision_thread_func,
                                               args=(color_frame, depth_frame),
                                               daemon=True)
                        thr.start()
                    else:
                        print("[SEGMENTACIJA] Kamera ni na voljo.")
                else:
                    # Če je že aktivna (prikaz rezultata ali čakanje), prekliči
                    #if vision_ready:
                    # zapri okno in izbriši rezultat
                    cv2.destroyWindow("Segmentacija")
                    with vision_lock:
                        vision_result = None
                        vision_ready = False
                    vision_active = False
                    print("[SEGMENTACIJA] Preklicano.")
                    #else:
                    # segmentacija še teče – ne moremo prekiniti, vendar lahko spremenimo stanje?
                    #    print("[SEGMENTACIJA] Meritev že poteka, počakajte.")
                        
            prev_rb_state = rb_pressed

            # ------------------------------------------------------------
            # 4. Preverimo, ali je segmentacija končana (vision_ready)
            # ------------------------------------------------------------
            if vision_ready and vision_active:
                with vision_lock:
                    res = vision_result
                    vision_ready = False
                if res is not None:
                    # Prikaži rezultat
                    vis_img, coords, raw_color_img, contour, u, v, text_lines = res
                    cv2.imshow("Segmentacija", vis_img)
                    # Shranimo podatke za kasnejšo uporabo
                    # (shranimo jih v spremenljivke, ki jih bomo uporabili ob D-pad)
                    # Uporabimo kar globalne spremenljivke ali pa jih shranimo v gamepad?
                    # Za preprostost bomo uporabili spremenljivke zunaj zanke:
                    # Uporabimo kar closure, vendar bomo morali shraniti v zunanje spremenljivke
                    # Ker smo v zanki, jih lahko shranimo v lokalne spremenljivke, vendar morajo biti dosegljive
                    # v naslednjih iteracijah. Uporabimo kar globalne spremenljivke.
                    global _last_vis_result
                    _last_vis_result = res
                    print("[SEGMENTACIJA] Rezultat prikazan. Uporabite D-pad GOR/DOL ali RB za preklic.")
                else:
                    print("[SEGMENTACIJA] Meritev ni uspela (ni zaznanega objekta).")
                    vision_active = False
                    cv2.destroyWindow("Segmentacija")

            # ------------------------------------------------------------
            # 5. Če je rezultat prikazan, obdelaj D-pad in RB za akcije
            # ------------------------------------------------------------
            if vision_active and vision_ready is False and hasattr(gamepad, 'hat_y'):
                # Preberemo trenutno stanje hat iz gamepad objekta (že posodobljeno v update_and_log)
                hat_y = gamepad.hat_y
                hat_x = gamepad.hat_x

                # Robna detekcija za D-pad gor (y=1) in dol (y=-1)
                if hat_y == 1 and prev_hat_y != 1:
                    # Pošlji koordinate
                    if '_last_vis_result' in globals() and globals()['_last_vis_result'] is not None:
                        _, coords, _, _, _, _, _ = globals()['_last_vis_result']
                        x, y, z, o = coords
                        send_to_stm32(x, y, z, o)
                        # Po pošiljanju zapri segmentacijo
                        cv2.destroyWindow("Segmentacija")
                        vision_active = False
                        print("[SEGMENTACIJA] Koordinate poslane. Zapiranje.")
                elif hat_y == -1 and prev_hat_y != -1:
                    # Shrani sliko
                    if '_last_vis_result' in globals() and globals()['_last_vis_result'] is not None:
                        _, _, raw_color_img, contour, u, v, text_lines = globals()['_last_vis_result']
                        save_full_color_image(raw_color_img, contour, u, v, text_lines)
                        cv2.destroyWindow("Segmentacija")
                        vision_active = False
                        print("[SEGMENTACIJA] Slika shranjena. Zapiranje.")


                prev_hat_y = hat_y
                prev_hat_x = hat_x

                # Če je RB ponovno pritisnjen, prekličemo (že obdelano zgoraj)
                # Dodatno: če pritisnemo katerikoli drug gumb (razen D-pad), morda ignoriramo
                # Ali pa prekličemo ob kateremkoli gumbu? Po navodilih naj bi samo RB preklical.

            # ------------------------------------------------------------
            # 6. Pošiljanje PLC podatkov
            # ------------------------------------------------------------
            # Vedno pošljemo timestamp (watchdog)
            if plc is not None:
                try:
                    #print("game pad :",gamepad.plc_state)
                    plc.write_by_name('MAIN.CartContol.data.timeStamp', cycle_timestamp, pyads.PLCTYPE_LREAL)
                except Exception as e:
                    print(f"[PLC] Napaka pri timestampu: {e}")

            # Če je vision_active (segmentacija v teku ali prikaz rezultata), pošljemo zaustavitev
            if vision_active:
                # Pošljemo ukaz za zaustavitev (hitrost 0, e-stop?)
                try:
                    if plc is not None:
                        # Pošljemo podatke za zaustavitev
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
                        print("5")
                        print("vision active:",vision_active)
                        print("vision ready:",vision_ready)
                        plc.write_by_name("MAIN.MasterContol.dataArray", master_data, pyads.PLCTYPE_LREAL*16)
                        # Poskrbimo še za hitrost 0
                        plc.write_by_name("MAIN.CartContol.data.hitrost_ms", 0.0, pyads.PLCTYPE_LREAL)
                except Exception as e:
                    print(f"[PLC] Napaka pri pošiljanju zaustavitve: {e}")
            else:
                # Normalno krmiljenje
                if gamepad.plc_state != prev_plc_state:
                    try:
                        if plc is not None:
                            if gamepad.plc_state == 1:
                                print("4")
                                print("vision active:",vision_active)
                                print("vision ready:",vision_ready)
                                plc.write_by_name('MAIN.MasterContol.data.mode', 1, pyads.PLCTYPE_LREAL)
                                plc.write_by_name('MAIN.MasterContol.data.masterSwich', 1, pyads.PLCTYPE_LREAL)
                                plc.write_by_name('MAIN.MasterContol.data.steeringMode', gamepad.selected_mode, pyads.PLCTYPE_LREAL)
                                print(f"PLC: START (mode {gamepad.selected_mode})")
                            else:
                                print("3")
                                print("vision active:",vision_active)
                                print("vision ready:",vision_ready)
                                plc.write_by_name('MAIN.MasterContol.data.mode', 0, pyads.PLCTYPE_LREAL)
                                plc.write_by_name('MAIN.MasterContol.data.masterSwich', 0, pyads.PLCTYPE_LREAL)
                                print("PLC: STOP")
                            prev_plc_state = gamepad.plc_state
                    except Exception as e:
                        print(f"[PLC] Napaka pri vpisu stanja: {e}")

                # Ostali podatki ob spremembah
                if gamepad.has_changed and not gamepad.mode_changed:
                    # Pošiljamo samo, če ni mode_changed (mode spremenimo posebej)
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
                        cart_data = [
                            cycle_timestamp,
                            gamepad.y_axis,
                            0.0,  # delta angle (ni v uporabi)
                            0.0, 0.0, 0.0, 0.0
                        ]
                        try:
                            if plc is not None:
                                print("2")
                                print("vision active:",vision_active)
                                print("vision ready:",vision_ready)
                                plc.write_by_name("MAIN.MasterContol.dataArray", master_data, pyads.PLCTYPE_LREAL*16)
                                plc.write_by_name("MAIN.CartContol.dataArray[0]", cart_data[0], pyads.PLCTYPE_LREAL)
                                plc.write_by_name("MAIN.CartContol.dataArray[1]", cart_data[1], pyads.PLCTYPE_LREAL)
                                # ostale ne pošiljamo
                        except Exception as e:
                            print(f"[PLC] Napaka pri pisanju: {e}")

                # Če je prišlo do spremembe načina (gumbi X,Y,B), pošljemo mode
                if mode_change_requested:
                    try:
                        if plc is not None and mode_change_requested:
                            print("1")
                            print("vision active:",vision_active)
                            print("vision ready:",vision_ready)

                            if gamepad.mode_changed:
                                temp_timestamp=cycle_timestamp

                            if ((cycle_timestamp-temp_timestamp)>=50):
                                plc.write_by_name('MAIN.MasterContol.data.steeringMode', gamepad.selected_mode, pyads.PLCTYPE_LREAL)
                                print(f"PLC: mode nastavljen na {gamepad.selected_mode}")
                                mode_change_requested=False

                    except Exception as e:
                        print(f"[PLC] Napaka pri pisanju mode: {e}")

            # ------------------------------------------------------------
            # 7. OpenCV osveževanje okna (waitKey)
            # ------------------------------------------------------------
            cv2.waitKey(1)

            # ------------------------------------------------------------
            # 8. Zakasnitev za želeno frekvenco
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
