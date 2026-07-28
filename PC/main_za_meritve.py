#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Združena koda: mobilna platforma + segmentacija z RealSense in CLIPSeg.
- Gamepad krmiljenje (hitrost, zavijanje z D-pad, načini, varnost)
- PLC komunikacija prek ADS
- RB (gumb 5) takoj odpre/zapre okno s preview in nastavi mode 5
- A (gumb 0) v preview sproži segmentacijo; v rezultatu sprejme koordinate in začne merilni protokol
- B (gumb 1) v rezultatu zavrne in vrne v preview
- Back (gumb 6) pošlje kalibracijski ukaz na STM32
- Med odprtim oknom je vožnja zaustavljena
- UART komunikacija: pošiljanje brez \r\n, branje odgovorov
- Merilni protokol: MERITEV 1–4 s časovnimi žigi, shranjevanje v CSV, graf razdalje
- GO se pošlje šele po končani meritvi (po MERITEV 4 STOP)
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
import csv
import matplotlib.pyplot as plt

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
SERIAL_PORT = 'COM3'
BAUD_RATE = 115200
UART_LOG_FILE = "uart_timestamps.txt"

MIN_DISTANCE_MM = 200
MAX_DISTANCE_MM = 400
ENABLE_AVERAGING = False
NUM_SAMPLES = 7

X_SCALE = 1.0
X_OFFSET_MM = 0
Y_SCALE = 1.0
Y_OFFSET_MM = 0
Z_HEIGHT_MM = 100

LOKALNI_AMS_ID = "192.168.65.121.1.1"
PLC_IP = "192.168.64.200"
PLC_AMS_ID = "169.254.220.1.1.1"

# --- Globalne spremenljivke ---
set_speed = 50
current_angle = 0.0
old_current_angle = 0.0
delta_angle = 0.0
max_angle = 30.0
angle_step = 10
mode_change_requested = False
timestamp_temp = 0
cycle_timestamp = 0

pipeline = None
align = None
depth_scale = None
intrinsics = None
device = "cpu"
processor = None
model = None
spatial_filter = None
hole_filling = None

vision_window_open = False
vision_preview_mode = True
vision_segmentation_result = None
vision_lock = threading.Lock()
last_color_frame = None
last_depth_frame = None

# UART
uart_ser = None
uart_lock = threading.Lock()
uart_sending = threading.Event()
exit_pending = False          # ali čakamo na EXIT po GO

# Merilni protokol
measurement_state = "idle"          # idle, meas1, wait1, meas2, wait2, meas3, wait3, meas4, done, aborted
measurement_folder = None
measurement_run_counter = 0
measurement_times = []
measurement_distances = []
measurement_step_start = None
measurement_step_number = 0
measurement_total_time = 0.0
measurement_aborted = False
measurement_waiting_for_approval = False

# UART sporočila iz poslušalca
uart_messages = []
uart_messages_lock = threading.Lock()

# ==============================================================================
# FUNKCIJE ZA UART
# ==============================================================================
def uart_log_timestamp(keyword):
    timestamp = time.time()
    try:
        with open(UART_LOG_FILE, 'a') as f:
            f.write(f"{timestamp:.6f} - {keyword}\n")
        print(f"[UART LOG] {keyword} @ {timestamp:.6f}")
    except Exception as e:
        print(f"[UART LOG] Napaka pri pisanju: {e}")

def uart_listener():
    """Neprekinjeno bere UART in dodaja sporočila v čakalno vrsto."""
    global uart_ser, uart_messages
    try:
        uart_ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0)
        print(f"[UART] Poslušalec aktiviran na {SERIAL_PORT}")
        while True:
            if uart_sending.is_set():
                time.sleep(0.01)
                continue
            with uart_lock:
                if uart_ser and uart_ser.is_open and uart_ser.in_waiting > 0:
                    try:
                        data = uart_ser.read(uart_ser.in_waiting).decode('utf-8', errors='ignore')
                        for line in data.splitlines():
                            if line.strip():
                                print(f"[UART RX] {line.strip()}")
                                with uart_messages_lock:
                                    uart_messages.append(line.strip())
                                keywords = ["START", "STOP", "MEASURE", "DONE", "ERROR", "OK", "GO", "ACK", "ECHO", "CALIBRATION", "EXIT", "MERITEV", "Razdalja"]
                                for kw in keywords:
                                    if kw in line.upper():
                                        uart_log_timestamp(f"RX_{kw}")
                                        break
                    except Exception as e:
                        print(f"[UART] Napaka pri branju: {e}")
            time.sleep(0.01)
    except Exception as e:
        print(f"[UART] Napaka v poslušalcu: {e}")

def send_uart_command(cmd):
    """Pošlje poljuben ukaz po UART (z \r\n)."""
    global uart_ser
    try:
        with uart_lock:
            if uart_ser is None or not uart_ser.is_open:
                uart_ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5)
            uart_ser.write(f"{cmd}\r\n".encode())
            uart_ser.flush()
            print(f"[SERIJSKI] Poslano: {cmd}")
    except Exception as e:
        print(f"[SERIJSKI] Napaka pri pošiljanju ukaza {cmd}: {e}")

def send_coordinates(x, y, z, o):
    """
    Pošlje koordinate na STM32 (X, Y, O) brez GO.
    Y = y - 205 - 20, X = -x (obrnjen predznak).
    """
    global uart_ser
    y_adjusted = y - 205 - 20
    x_neg = -x
    commands = [
        f"X={x_neg}",
        f"Y={y_adjusted}",
        f"O={o:.2f}"
    ]
    print(f"\n[SERIJSKI] Pošiljam koordinate (brez GO):")
    for cmd in commands:
        print(f"  {cmd}")
    
    uart_sending.set()
    try:
        with uart_lock:
            if uart_ser is None or not uart_ser.is_open:
                uart_ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5)
            time.sleep(0.05)
            for cmd in commands:
                uart_ser.write(f"{cmd}\r\n".encode())
                uart_ser.flush()
                print(f"[SERIJSKI] Poslano: {cmd}")
                time.sleep(0.5)
                # Preberemo morebitne odgovore
                responses = []
                start_t = time.time()
                while time.time() - start_t < 0.5:
                    if uart_ser.in_waiting > 0:
                        data = uart_ser.read(uart_ser.in_waiting).decode('utf-8', errors='ignore')
                        for line in data.splitlines():
                            if line.strip():
                                responses.append(line.strip())
                    time.sleep(0.02)
                for resp in responses:
                    print(f"[SERIJSKI] Odgovor: {resp}")
                    uart_log_timestamp(f"RX_{resp}")
    except Exception as e:
        print(f"[SERIJSKI] Napaka pri pošiljanju koordinat: {e}")
    finally:
        uart_sending.clear()

def send_go():
    """Pošlje GO ukaz in nastavi exit_pending."""
    global exit_pending
    print("[SERIJSKI] Pošiljam GO ...")
    send_uart_command("GO")
    exit_pending = True
    uart_log_timestamp("GO_SENT")

def send_exit():
    """Pošlje EXIT ukaz (običajno ob zaprtju okna)."""
    global exit_pending
    if not exit_pending:
        return
    print("[SERIJSKI] Pošiljam EXIT ...")
    send_uart_command("EXIT")
    exit_pending = False
    uart_log_timestamp("EXIT_SENT")

def send_calibration():
    """Pošlje kalibracijski ukaz."""
    send_uart_command("calibrate")

# ==============================================================================
# FUNKCIJE ZA MERILNI PROTOKOL
# ==============================================================================
def create_measurement_folder():
    global measurement_run_counter, measurement_folder
    base = "meritve"
    if not os.path.exists(base):
        os.makedirs(base)
    measurement_run_counter += 1
    folder_name = f"MERITEV_{measurement_run_counter}"
    measurement_folder = os.path.join(base, folder_name)
    os.makedirs(measurement_folder, exist_ok=True)
    print(f"[MERITEV] Ustvarjena mapa: {measurement_folder}")
    return measurement_folder

def start_measurement_step(step_num):
    """Začne korak meritve: pošlje MERITEV X START in zabeleži čas."""
    global measurement_step_start, measurement_step_number, measurement_state
    measurement_step_number = step_num
    measurement_step_start = time.time()
    cmd = f"MERITEV {step_num} START"
    send_uart_command(cmd)
    measurement_state = f"meas{step_num}"
    print(f"\n[MERITEV] Začetek koraka {step_num} (poslan ukaz: {cmd})")
    print(f"[MERITEV] Čakam na 'MERITEV {step_num} STOP' ...")

def handle_measurement_stop(step_num):
    """Ob prejemu MERITEV X STOP: zabeleži čas in preklopi v čakanje na potrditev."""
    global measurement_step_start, measurement_times, measurement_state, measurement_total_time, measurement_waiting_for_approval
    if measurement_step_start is None:
        print(f"[MERITEV] Prejet STOP za korak {step_num}, a ni bilo začetka.")
        return
    stop_time = time.time()
    elapsed = stop_time - measurement_step_start
    measurement_times.append({
        'step': step_num,
        'start': measurement_step_start,
        'stop': stop_time,
        'elapsed': elapsed
    })
    measurement_total_time += elapsed
    print(f"[MERITEV] Korak {step_num} končan. Trajanje: {elapsed:.3f} s")
    if step_num < 4:
        measurement_state = f"wait{step_num}"
        measurement_waiting_for_approval = True
        print(f"\n[MERITEV] Korak {step_num} zaključen. Pritisnite A za nadaljevanje na korak {step_num+1}, ali B za prekinitev.")
    else:
        # step_num == 4, končano
        measurement_state = "done"
        measurement_waiting_for_approval = False
        print("[MERITEV] Vsi koraki zaključeni (MERITEV 4 STOP). Zaključujem meritev.")
        finalize_measurement()

def abort_measurement():
    """Prekine meritev: pošlje MERITEV KONEC, vrne v predogled."""
    global measurement_state, measurement_aborted, vision_preview_mode, vision_segmentation_result, measurement_waiting_for_approval
    print("[MERITEV] Prekinitev meritve s strani uporabnika.")
    send_uart_command("MERITEV KONEC")
    measurement_aborted = True
    measurement_waiting_for_approval = False
    vision_preview_mode = True
    vision_segmentation_result = None
    measurement_state = "idle"
    print("[MERITEV] Meritev prekinjena, vračam se v predogled.")

def finalize_measurement():
    """Po končani meritvi (po MERITEV 4 STOP) shrani podatke, izriše graf, pošlje GO in zapre okno."""
    global measurement_state, measurement_folder, measurement_times, measurement_distances
    global vision_window_open, vision_preview_mode, vision_segmentation_result, exit_pending

    if measurement_folder is None:
        print("[MERITEV] Ni mape za shranjevanje.")
        return

    # Shrani časovni CSV
    times_csv = os.path.join(measurement_folder, "TEREN_BF_MERITVE.csv")
    with open(times_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Step', 'StartTime', 'StopTime', 'ElapsedSeconds'])
        for rec in measurement_times:
            writer.writerow([rec['step'], rec['start'], rec['stop'], rec['elapsed']])
        writer.writerow(['TOTAL', '', '', measurement_total_time])
    print(f"[MERITEV] Časovni podatki shranjeni v {times_csv}")

    # Shrani razdalje (če obstajajo)
    if measurement_distances:
        dist_csv = os.path.join(measurement_folder, "razdalje.csv")
        with open(dist_csv, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Timestamp', 'Razdalja_mm'])
            for ts, dist in measurement_distances:
                writer.writerow([ts, dist])
        print(f"[MERITEV] Razdalje shranjene v {dist_csv}")
        # Izriši graf
        try:
            times = [ts for ts, _ in measurement_distances]
            dists = [d for _, d in measurement_distances]
            plt.figure(figsize=(10,6))
            plt.plot(times, dists, marker='o', linestyle='-')
            plt.xlabel('Čas (s)')
            plt.ylabel('Razdalja (mm)')
            plt.title('Oddaljenost senzorja med MERITEV 3')
            plt.grid(True)
            graf_path = os.path.join(measurement_folder, "graf_razdalje.png")
            plt.savefig(graf_path)
            plt.close()
            print(f"[MERITEV] Graf shranjen v {graf_path}")
        except Exception as e:
            print(f"[MERITEV] Napaka pri risanju grafa: {e}")

    # Pošlji GO, da se platforma premakne
    send_go()

    # Zapri okno in omogoči vožnjo
    vision_window_open = False
    vision_preview_mode = True
    vision_segmentation_result = None
    cv2.destroyWindow("Segmentacija")
    measurement_state = "idle"
    print("[MERITEV] Meritev končana, GO poslan. Okno zaprto, platforma pripravljena na vožnjo.")

def process_uart_messages():
    """Preveri čakalno vrsto UART sporočil in obravnava ukaze MERITEV in Razdalja."""
    global measurement_state, measurement_distances, measurement_step_number
    with uart_messages_lock:
        while uart_messages:
            msg = uart_messages.pop(0)
            # MERITEV X STOP
            if msg.startswith("MERITEV") and "STOP" in msg:
                parts = msg.split()
                if len(parts) >= 3:
                    try:
                        step = int(parts[1])
                        if measurement_state == f"meas{step}":
                            handle_measurement_stop(step)
                        else:
                            print(f"[UART] Prejet STOP za korak {step}, vendar trenutno stanje ni ustrezno ({measurement_state})")
                    except ValueError:
                        pass
            # MERITEV KONEC (lahko pošlje STM32)
            elif msg.startswith("MERITEV KONEC"):
                print("[UART] STM32 je poslal MERITEV KONEC - prekinjam meritev.")
                if measurement_state != "idle":
                    abort_measurement()
            # Razdalja za MERITEV 3
            elif msg.startswith("Razdalja:"):
                if measurement_state in ("meas3", "wait3", "meas4", "wait4"):
                    try:
                        dist_str = msg.split(":")[1].strip()
                        dist_mm = float(dist_str)
                        measurement_distances.append((time.time(), dist_mm))
                        print(f"[MERITEV] Zabeležena razdalja: {dist_mm} mm")
                    except Exception as e:
                        print(f"[MERITEV] Napaka pri razčlenjevanju razdalje: {e}")

# ==============================================================================
# RAZRED ZA GAMEPAD
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
        print(" * GUMB A (0)                  : Vklop/izklop TEMPOMATA (samo ko okno ni odprto)")
        print(" * GUMBI Y (3), X (2), B (1)   : Nastavitev načina (Y->1, X->2, B->5) – DELUJEJO VEDNO")
        print(" * GUMB START (7)              : Preklop ZAGON / STOP")
        print(" * GUMB RB (5)                 : Odpre/zapre okno s preview (takoj, nastavi mode 5) ")
        print(" * GUMB BACK (6)               : Pošlje kalibracijski ukaz na STM32")
        print(" * Ko je okno odprto (preview):")
        print("     A (0)    : Sproži segmentacijo")
        print("     RB (5)   : Zapri okno")
        print(" * Ko je prikazan rezultat segmentacije:")
        print("     A (0)    : Sprejmi koordinate (začne meritev, shrani sliko)")
        print("     B (1)    : Zavrni (shrani sliko, vrni se v preview)")
        print(" * Med merilnim protokolom:")
        print("     A (0)    : Potrdi in nadaljuj na naslednji korak")
        print("     B (1)    : Prekini meritev")
        print("="*70 + "\n")

    def update_and_log(self):
        global current_angle, old_current_angle, delta_angle, mode_change_requested, cycle_timestamp, timestamp_temp
        global vision_window_open
        self.has_changed = False
        self.mode_changed = False
        pygame.event.pump()
        if not self.joystick:
            if pygame.joystick.get_count() > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                print(f"[INFO] Plošček ponovno povezan: {self.joystick.get_name()}")
            return

        if self.selected_mode != 5.0 and not vision_window_open:
            if not self.cruise_control_active:
                raw_y = -self.joystick.get_axis(3)
                y_val = round(raw_y, 3) if abs(raw_y) > 0.05 else 0.0
                if y_val != self.y_axis:
                    self.y_axis = y_val
                    self.has_changed = True

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

        hat = self.joystick.get_hat(0)
        self.prev_hat_x, self.prev_hat_y = self.hat_x, self.hat_y
        self.hat_x, self.hat_y = hat

        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                if event.button == 7:
                    if not vision_window_open:
                        self.drive_mode = 1.0 if self.drive_mode == 0.0 else 0.0
                        self.has_changed = True
                elif event.button == 0 and self.dead_man_pressed:
                    if not vision_window_open:
                        self.cruise_control_active = not self.cruise_control_active
                        self.has_changed = True
                        if self.cruise_control_active:
                            print(f"[TEMPOMAT] VKLOPLJEN pri hitrosti: {self.y_axis:.2f} m/s")
                        else:
                            print("[TEMPOMAT] IZKLOPLJEN")
                elif event.button == 3:
                    self.selected_mode = 1.0
                    self.mode_changed = True
                    mode_change_requested = True
                    timestamp_temp = cycle_timestamp
                    print("[NAČIN] Izbran Mode 1 (Y)")
                elif event.button == 2:
                    self.selected_mode = 2.0
                    self.mode_changed = True
                    mode_change_requested = True
                    timestamp_temp = cycle_timestamp
                    print("[NAČIN] Izbran Mode 2 (X)")
                elif event.button == 1:
                    self.selected_mode = 5.0
                    self.mode_changed = True
                    mode_change_requested = True
                    timestamp_temp = cycle_timestamp
                    print("[NAČIN] Izbran Mode 5 (B)")
                elif event.button == 6:  # BACK
                    if not vision_window_open and measurement_state == "idle":
                        print("[GAMEPAD] BACK pritisnjen, pošiljam kalibracijo...")
                        send_calibration()
                    else:
                        print("[GAMEPAD] BACK pritisnjen, a okno je odprto ali meritev poteka – kalibracija ni dovoljena.")
            elif event.type == pygame.JOYHATMOTION and self.selected_mode == 1.0:
                hat_x, hat_y = event.value
                if hat_x != 0 and not self.e_stop and not vision_window_open and measurement_state == "idle":
                    new_angle = current_angle + (hat_x * angle_step)
                    if -max_angle <= new_angle <= max_angle:
                        current_angle = new_angle
                        delta_angle = current_angle - old_current_angle
                        self.has_changed = True
                        print(f"[D-PAD] Kot: {current_angle}° (Δ={delta_angle}°)")

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
# VISION FUNKCIJE
# ==============================================================================
def init_vision():
    global pipeline, align, depth_scale, intrinsics, processor, model, spatial_filter, hole_filling
    print("[DEBUG] Inicializacija RealSense kamere ...")
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
    profile = pipeline.start(config)
    dev = profile.get_device()
    depth_sensor = dev.first_depth_sensor()
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
            depth_sensor.set_option(rs.option.visual_preset, 3)
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

def save_full_color_image(raw_color_img, contour, u, v, text_lines, folder="slike_segmentacija"):
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
    uart_log_timestamp("IMAGE_SAVED")

def process_single_frame(color_image, depth_frame_raw):
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
    mask = (probs > 0.25).cpu().numpy().astype(bool)
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
    start_time = time.time()
    c_img = np.asanyarray(color_frame.get_data())
    res = process_single_frame(c_img, depth_frame)
    if res is None:
        return None
    u, v, raw_x, raw_y, contour, filtered_c_img, raw_color_img = res
    vis_img = filtered_c_img.copy()
    mode_label = "POSAMEZNA SLIKA"
    if contour is not None:
        cv2.drawContours(vis_img, [contour], -1, (0, 255, 0), 2)
    X_coord_mm = int(round((-raw_x * X_SCALE) + X_OFFSET_MM))
    Y_coord_mm = int(round((raw_y * Y_SCALE) + Y_OFFSET_MM))
    Z_coord_mm = int(Z_HEIGHT_MM)
    radial_angle_rad = math.atan2(X_coord_mm, Y_coord_mm)
    radial_angle_deg = math.degrees(radial_angle_rad)
    orientation = float(np.clip(radial_angle_deg, -30.0, 30.0))
    text_x = f"X: {X_coord_mm} mm"
    text_y = f"Y: {Y_coord_mm-205-20} mm"
    text_z = f"Z: {Z_coord_mm} mm"
    text_o = f"O: {-orientation:.2f} deg"
    cv2.rectangle(vis_img, (10, 10), (550, 155), (0, 0, 0), -1)
    y_pos = 30
    cv2.putText(vis_img, text_x, (20, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2)
    y_pos += 25
    cv2.putText(vis_img, text_y, (20, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2)
    y_pos += 25
    cv2.putText(vis_img, text_z, (20, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 255), 2)
    y_pos += 25
    cv2.putText(vis_img, text_o, (20, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2)
    y_pos += 25
    cv2.putText(vis_img, "A: Sprejmi | B: Zavrni", (20, y_pos+5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    cv2.circle(vis_img, (u, v), 10, (0, 0, 255), -1)
    cv2.circle(vis_img, (u, v), 10, (255, 255, 255), 2)
    
    text_lines = [text_x, text_y, text_z, text_o]
    coords = (X_coord_mm, Y_coord_mm, Z_coord_mm, orientation)
    elapsed = time.time() - start_time
    uart_log_timestamp(f"MEASURE_DONE {elapsed:.3f}s")
    return vis_img, coords, raw_color_img, contour, u, v, text_lines

def create_preview_frame(color_image, depth_image):
    color_filt, depth_filt = filter_by_distance(color_image, depth_image)
    depth_colormap = cv2.applyColorMap(
        cv2.convertScaleAbs(depth_filt, alpha=0.08), cv2.COLORMAP_JET
    )
    preview = np.hstack((color_filt, depth_colormap))
    cv2.putText(preview, f"COLOR ({MIN_DISTANCE_MM}-{MAX_DISTANCE_MM}mm)", (30, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(preview, f"DEPTH ({MIN_DISTANCE_MM}-{MAX_DISTANCE_MM}mm)", (1310, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(preview, "A: Segmentacija | RB: Zapri", (30, 700),
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
    global measurement_state, measurement_waiting_for_approval, measurement_folder, measurement_times, measurement_distances, measurement_total_time, measurement_aborted, exit_pending

    # Zaženi UART listener
    uart_thread = threading.Thread(target=uart_listener, daemon=True)
    uart_thread.start()
    print("[UART] Poslušalec zagnan.")

    # PLC
    print("Povezovanje s PLC-jem...")
    plc = pyads.Connection(PLC_AMS_ID, pyads.PORT_TC3PLC1, PLC_IP)
    try:
        plc.open()
        print("ADS povezava uspešna.")
    except Exception as e:
        print(f"[OPOZORILO] PLC ni dosegljiv: {e}")
        plc = None

    # Kamera
    try:
        init_vision()
    except Exception as e:
        print(f"[NAPAKA] Inicializacija visiona ni uspela: {e}")
        print("Program bo deloval brez segmentacije.")
        pipeline = None

    gamepad = SimpleGamepadPygame()
    gamepad.print_instructions()

    cv2.namedWindow("Segmentacija", cv2.WINDOW_NORMAL)
    cv2.destroyWindow("Segmentacija")

    loop_rate = 0.02
    prev_plc_state = -1
    # Spremenljivke za detekcijo robov
    prev_rb_state = False
    prev_a_state = False
    prev_b_state = False

    print("Krmiljenje aktivno. Pritisnite Ctrl+C za izhod.")

    try:
        while True:
            start_time = time.time()
            cycle_timestamp += 1.0

            # Zajem slik
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

            gamepad.update_and_log()

            # Trenutna stanja gumbov (za detekcijo robov)
            if gamepad.joystick:
                current_a = bool(gamepad.joystick.get_button(0))
                current_b = bool(gamepad.joystick.get_button(1))
                current_rb = bool(gamepad.joystick.get_button(5))
            else:
                current_a = False
                current_b = False
                current_rb = False

            # Obdelava UART sporočil (merilni protokol)
            process_uart_messages()

            # Čakanje na potrditev uporabnika med meritvijo
            if measurement_waiting_for_approval:
                # Detekcija A (potrditev) ali B (prekinitev)
                if current_a and not prev_a_state:
                    step = measurement_step_number
                    if step < 4:
                        print(f"[MERITEV] Uporabnik potrdil, nadaljujem na korak {step+1}")
                        measurement_waiting_for_approval = False
                        start_measurement_step(step+1)
                elif current_b and not prev_b_state:
                    print("[MERITEV] Uporabnik prekinil meritev.")
                    abort_measurement()
                # Ne glede na to, nadaljujemo s prikazom
                if vision_window_open:
                    if vision_preview_mode:
                        if last_color_frame is not None and last_depth_frame is not None:
                            c_img = np.asanyarray(last_color_frame.get_data())
                            d_img = np.asanyarray(last_depth_frame.get_data())
                            preview = create_preview_frame(c_img, d_img)
                            cv2.imshow("Segmentacija", preview)
                    else:
                        if vision_segmentation_result is not None:
                            vis_img, _, _, _, _, _, _ = vision_segmentation_result
                            cv2.imshow("Segmentacija", vis_img)
                    cv2.waitKey(1)
                # Posodobimo prev stanja in nadaljujemo
                prev_a_state = current_a
                prev_b_state = current_b
                prev_rb_state = current_rb
                # Počakamo na konec zanke
                time.sleep(loop_rate)
                continue

            # RB: odpri/zapri okno (samo če ni v meritvi)
            if current_rb and not prev_rb_state and measurement_state == "idle":
                if not vision_window_open:
                    if pipeline is not None and last_color_frame is not None:
                        print("[PREVIEW] Odpiram okno, nastavljam mode 5.")
                        if plc is not None:
                            try:
                                plc.write_by_name('MAIN.MasterContol.data.steeringMode', 5.0, pyads.PLCTYPE_LREAL)
                                plc.write_by_name('MAIN.CartContol.data.hitrost_ms', 0.0, pyads.PLCTYPE_LREAL)
                                print("[PLC] Mode nastavljen na 5, vožnja ustavljena.")
                            except Exception as e:
                                print(f"[PLC] Napaka pri nastavljanju mode 5: {e}")
                        vision_window_open = True
                        vision_preview_mode = True
                        vision_segmentation_result = None
                        cv2.namedWindow("Segmentacija", cv2.WINDOW_NORMAL)
                    else:
                        print("[PREVIEW] Kamera ni na voljo.")
                else:
                    # Zapiranje okna: pošlji EXIT, če čaka
                    if exit_pending:
                        print("[PREVIEW] Zapiram okno, pošiljam EXIT.")
                        send_exit()
                    else:
                        print("[PREVIEW] Zapiram okno.")
                    vision_window_open = False
                    vision_preview_mode = True
                    vision_segmentation_result = None
                    cv2.destroyWindow("Segmentacija")
                    mode_change_requested = False

            # Obdelava, če je okno odprto in ni v meritvi
            if vision_window_open and measurement_state == "idle":
                # A v preview -> sproži segmentacijo
                if current_a and not prev_a_state and vision_preview_mode:
                    if last_color_frame is not None and last_depth_frame is not None:
                        print("[SEGMENTACIJA] Zaganjam segmentacijo...")
                        uart_log_timestamp("MEASURE_START")
                        res = run_measurement(last_color_frame, last_depth_frame)
                        if res is not None:
                            vis_img, coords, raw_color_img, contour, u, v, text_lines = res
                            vision_segmentation_result = (vis_img, coords, raw_color_img, contour, u, v, text_lines)
                            vision_preview_mode = False
                            print("[SEGMENTACIJA] Rezultat prikazan. Pritisnite A za sprejem, B za zavrnitev.")
                        else:
                            print("[SEGMENTACIJA] Meritev ni uspela (ni zaznanega objekta).")
                            preview = create_preview_frame(
                                np.asanyarray(last_color_frame.get_data()),
                                np.asanyarray(last_depth_frame.get_data())
                            )
                            cv2.putText(preview, "NI ZAZNANEGA OBJEKTA!", (300, 360),
                                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
                            cv2.imshow("Segmentacija", preview)
                    else:
                        print("[SEGMENTACIJA] Ni na voljo slik.")

                # A v rezultatu -> sprejem
                elif current_a and not prev_a_state and not vision_preview_mode:
                    if vision_segmentation_result is not None:
                        vis_img, coords, raw_color_img, contour, u, v, text_lines = vision_segmentation_result
                        x, y, z, o = coords
                        # Shrani sliko
                        save_full_color_image(raw_color_img, contour, u, v, text_lines)
                        # Pošlji koordinate (brez GO)
                        send_coordinates(x, y, z, o)
                        # Ustvari mapo in začni meritev
                        create_measurement_folder()
                        measurement_times = []
                        measurement_distances = []
                        measurement_total_time = 0.0
                        measurement_aborted = False
                        start_measurement_step(1)
                    else:
                        print("[SEGMENTACIJA] Ni rezultata za sprejeti.")

                # B v rezultatu -> zavrni
                elif current_b and not prev_b_state and not vision_preview_mode:
                    if vision_segmentation_result is not None:
                        _, _, raw_color_img, contour, u, v, text_lines = vision_segmentation_result
                        # Shrani sliko
                        save_full_color_image(raw_color_img, contour, u, v, text_lines)
                        print("[SEGMENTACIJA] Rezultat zavrnjen. Vračam se v preview.")
                        uart_log_timestamp("REJECT")
                        vision_preview_mode = True
                        vision_segmentation_result = None
                    else:
                        print("[SEGMENTACIJA] Ni rezultata za zavrniti.")

                # Prikaz okna
                if vision_window_open and measurement_state == "idle":
                    if vision_preview_mode:
                        if last_color_frame is not None and last_depth_frame is not None:
                            c_img = np.asanyarray(last_color_frame.get_data())
                            d_img = np.asanyarray(last_depth_frame.get_data())
                            preview = create_preview_frame(c_img, d_img)
                            cv2.imshow("Segmentacija", preview)
                    else:
                        if vision_segmentation_result is not None:
                            vis_img, _, _, _, _, _, _ = vision_segmentation_result
                            cv2.imshow("Segmentacija", vis_img)
                    cv2.waitKey(1)

            # PLC
            if plc is not None:
                try:
                    plc.write_by_name('MAIN.CartContol.data.timeStamp', cycle_timestamp, pyads.PLCTYPE_LREAL)
                except Exception as e:
                    print(f"[PLC] Napaka pri timestampu: {e}")

            # Zaustavitev vožnje med odprtim oknom ali med meritvijo
            if vision_window_open or measurement_state != "idle":
                try:
                    if plc is not None:
                        master_data = [
                            cycle_timestamp, 0.0, 0.0, 0.0,
                            2.0, 2.0, 2.0, 2.0,
                            2.0, 2.0, 2.0, 2.0,
                            2.0, 2.0, 2.0,
                            5.0
                        ]
                        plc.write_by_name("MAIN.MasterContol.dataArray", master_data, pyads.PLCTYPE_LREAL*16)
                        plc.write_by_name("MAIN.CartContol.data.hitrost_ms", 0.0, pyads.PLCTYPE_LREAL)
                except Exception as e:
                    print(f"[PLC] Napaka pri pošiljanju zaustavitve: {e}")
            else:
                # Normalno krmiljenje
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

                if gamepad.has_changed and not mode_change_requested and measurement_state == "idle":
                    if gamepad.e_stop == 0 and gamepad.drive_mode == 1.0 and gamepad.selected_mode != 5.0:
                        master_data = [
                            cycle_timestamp, 1.0, 1.0, 20.0,
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
                                if abs(delta_angle) >= angle_step:
                                    print(f"Zavijam na PLC -> Kot: {current_angle}°")
                                    plc.write_by_name('STEERING.WHEEL_1_ANG_REF', current_angle, pyads.PLCTYPE_LREAL)
                                    old_current_angle = current_angle
                                    delta_angle = 0.0
                        except Exception as e:
                            print(f"[PLC] Napaka pri pisanju: {e}")
                    else:
                        try:
                            if plc is not None:
                                master_data = [
                                    cycle_timestamp, 0.0, 0.0, 0.0,
                                    2.0, 2.0, 2.0, 2.0,
                                    2.0, 2.0, 2.0, 2.0,
                                    2.0, 2.0, 2.0,
                                    gamepad.selected_mode
                                ]
                                plc.write_by_name("MAIN.MasterContol.dataArray", master_data, pyads.PLCTYPE_LREAL*16)
                                plc.write_by_name("MAIN.CartContol.data.hitrost_ms", 0.0, pyads.PLCTYPE_LREAL)
                        except Exception as e:
                            print(f"[PLC] Napaka pri pisanju zaustavitve: {e}")

                if mode_change_requested and measurement_state == "idle":
                    try:
                        if plc is not None:
                            if (cycle_timestamp - timestamp_temp) >= 50:
                                plc.write_by_name('MAIN.MasterContol.data.steeringMode', gamepad.selected_mode, pyads.PLCTYPE_LREAL)
                                print(f"PLC: mode nastavljen na {gamepad.selected_mode}")
                                mode_change_requested = False
                            else:
                                plc.write_by_name('MAIN.CartContol.data.hitrost_ms', 0.0, pyads.PLCTYPE_LREAL)
                    except Exception as e:
                        print(f"[PLC] Napaka pri pisanju mode: {e}")

            # Posodobitev prejšnjih stanj gumbov (za naslednjo zanko)
            prev_a_state = current_a
            prev_b_state = current_b
            prev_rb_state = current_rb

            elapsed = time.time() - start_time
            time.sleep(max(0.0, loop_rate - elapsed))

    except KeyboardInterrupt:
        print("\n[INFO] Prekinitev s tipkovnico.")
    finally:
        if pipeline is not None:
            pipeline.stop()
        cv2.destroyAllWindows()
        if plc is not None:
            plc.close()
        pygame.quit()
        if uart_ser and uart_ser.is_open:
            uart_ser.close()
        print("[INFO] Program končan.")

if __name__ == "__main__":
    main()
