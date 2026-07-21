import os
import sys
import time
import math
import cv2
import numpy as np
import serial
import torch
from transformers import CLIPSegProcessor, CLIPSegForImageSegmentation

# Zaustavitev opozoril
os.environ["CUDA_VISIBLE_DEVICES"] = ""
os.environ["HF_TOKEN"] = ""

try:
    import pyrealsense2 as rs
except ImportError:
    print("[NAPAKA] Manjka 'pyrealsense2'. Nameščam...")
    os.system('python -m pip install pyrealsense2')
    import pyrealsense2 as rs


class RealSenseTracker:
    def __init__(self, serial_port='COM3', baud_rate=115200, min_dist=200, max_dist=400, z_height=-100):
        """
        Inicializacija kamere, AI modela in serijske povezave.
        Izvede se SAMO ENKRAT ob zagonu glavne skripte.
        """
        self.min_dist = min_dist
        self.max_dist = max_dist
        self.z_height = z_height
        
        # Kalibracija
        self.x_scale = 1.0
        self.x_offset = 0
        self.y_scale = 1.0
        self.y_offset = 0

        # Zastavice
        self.enable_averaging = False
        self.num_samples = 7
        self.data_sent = False  # Zastavica, ki se ponastavi ob pošiljanju

        # 1. Povezava STM32
        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=1)
            print(f"[DEBUG] Povezava uspešna na {serial_port}")
        except Exception as e:
            print(f"[DEBUG] Opozorilo pri serijski povezavi ({serial_port}): {e}")
            self.ser = None

        # 2. Inicializacija RealSense
        print("[DEBUG] Inicializacija RealSense kamere...")
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)

        profile = self.pipeline.start(config)
        dev = profile.get_device()
        depth_sensor = dev.first_depth_sensor()

        try:
            if dev.supports(rs.camera_info.advanced_mode):
                adv_mode = rs.rs400_advanced_mode(dev)
                if adv_mode.is_enabled():
                    depth_table = adv_mode.get_depth_table()
                    depth_table.disparityShift = 55  
                    adv_mode.set_depth_table(depth_table)
                    print("[DEBUG] Disparity Shift (55) uveljavljen.")
        except Exception as err:
            print(f"[DEBUG] Disparity shift opozorilo: {err}")

        if depth_sensor.supports(rs.option.visual_preset):
            try:
                depth_sensor.set_option(rs.option.visual_preset, 3)
            except Exception:
                pass

        self.spatial_filter = rs.spatial_filter()
        self.hole_filling = rs.hole_filling_filter()
        self.hole_filling.set_option(rs.option.holes_fill, 1)

        align_to = rs.stream.color
        self.align = rs.align(align_to)
        self.intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.depth_scale = depth_sensor.get_depth_scale()

        # 3. AI Model
        self.device = "cpu"
        print("[DEBUG] Nalagam CLIPSeg model...")
        self.processor = CLIPSegProcessor.from_pretrained("CIDAS/clipseg-rd64-refined")
        self.model = CLIPSegForImageSegmentation.from_pretrained("CIDAS/clipseg-rd64-refined")
        self.model.to(self.device)
        print("[DEBUG] CLIPSeg pripravljen.")

    def filter_by_distance(self, color_image, depth_image):
        """Ohrani barvo le za znane pikle v podanem območju razdalj."""
        depth_in_mm = depth_image.astype(np.float32) * self.depth_scale * 1000.0
        valid_mask = (depth_in_mm >= self.min_dist) & (depth_in_mm <= self.max_dist)

        filtered_color = color_image.copy()
        filtered_depth = depth_image.copy()

        filtered_color[~valid_mask] = 0
        filtered_depth[~valid_mask] = 0

        return filtered_color, filtered_depth

    def save_full_color_image(self, raw_color_img, contour, u, v, text_lines):
        """Shrani polno barvno sliko (brez črnin)."""
        folder_name = "slike_segmentacija"
        if not os.path.exists(folder_name):
            os.makedirs(folder_name)

        i = 1
        while True:
            file_path = os.path.join(folder_name, f"segmentacija_{i:03d}.png")
            if not os.path.exists(file_path):
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

        cv2.imwrite(file_path, save_img)
        print(f"[INFO] Slika shranjena v: {file_path}")

    def send_to_stm32(self, x, y, z, o):
        """Pošiljanje ukazov na STM32."""
        if self.ser and self.ser.is_open:
            cmd_x = f"X={x}\r\n"
            cmd_y = f"Y={y}\r\n"
            cmd_z = f"Z={z}\r\n"
            cmd_o = f"O={o:.2f}\r\n"

            print(f"\n[SERIJSKI] Pošiljam na STM32: X={x}, Y={y}, Z={z}, O={o:.2f}")
            
            self.ser.write(cmd_x.encode())
            time.sleep(0.1)
            self.ser.write(cmd_y.encode())
            time.sleep(0.1)
            self.ser.write(cmd_z.encode())
            time.sleep(0.1)
            self.ser.write(cmd_o.encode())
            time.sleep(0.1)
            self.ser.write("GO\r\n".encode())
            print("[SERIJSKI] GO poslan.")
        else:
            print(f"\n[DEBUG] (Simulacija) Bi poslal: X={x}, Y={y}, Z={z}, O={o:.2f}")
        
        # Ponastavitev zastavice ob uspešnem pošiljanju
        self.data_sent = True

    def process_single_frame(self, color_image, depth_frame_raw):
        """Obdelava posameznega okvira slikanja."""
        filtered_dframe = self.spatial_filter.process(depth_frame_raw)
        filtered_dframe = self.hole_filling.process(filtered_dframe)
        depth_image = np.asanyarray(filtered_dframe.get_data())

        color_filtered, depth_filtered = self.filter_by_distance(color_image, depth_image)
        
        rgb_image = cv2.cvtColor(color_filtered, cv2.COLOR_BGR2RGB)
        inputs = self.processor(text=["tree trunk"], images=[rgb_image], padding="max_length", return_tensors="pt")
        inputs = {k: v.to(self.device) for k, v in inputs.items()}
        
        with torch.no_grad():
            outputs = self.model(**inputs)
        
        logits = outputs.logits.unsqueeze(1)
        preds = torch.nn.functional.interpolate(logits, size=(720, 1280), mode="bilinear")
        probs = torch.sigmoid(preds[0][0])
        mask = (probs > 0.35).cpu().numpy().astype(bool)

        contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            
            masked_depth = np.where(mask, depth_filtered, 0)
            valid_depths = masked_depth[masked_depth > 0]
            
            if len(valid_depths) == 0:
                return None
                
            approx_depth_m = np.median(valid_depths) * self.depth_scale
            
            cy, fy = self.intrinsics.ppy, self.intrinsics.fy
            target_v = int(round(cy - ((self.z_height / 1000.0) * fy / approx_depth_m)))
            v = int(np.clip(target_v, 0, 719))

            row_mask = mask[v, :]
            x_indices = np.where(row_mask)[0]
            
            if len(x_indices) > 0:
                u = int(np.mean(x_indices))
            else:
                M = cv2.moments(largest_contour)
                u = int(M["m10"] / M["m00"]) if M["m00"] != 0 else 640

            half_w = 3
            depth_roi = depth_filtered[max(0, v-half_w):min(720, v+half_w+1), max(0, u-half_w):min(1280, u+half_w+1)]
            valid_roi_depths = depth_roi[depth_roi > 0]

            depth_raw = np.median(valid_roi_depths) if len(valid_roi_depths) > 0 else np.median(valid_depths)
            depth_in_meters = depth_raw * self.depth_scale

            rs_coords = rs.rs2_deproject_pixel_to_point(self.intrinsics, [u, v], depth_in_meters)
            
            raw_y_mm = rs_coords[2] * 1000  
            raw_x_mm = -rs_coords[0] * 1000 

            return u, v, raw_x_mm, raw_y_mm, largest_contour, color_filtered, color_image
        return None

    def run_measurement(self, color_frame, depth_frame):
        """Segmentira in izračuna koordinate."""
        c_img = np.asanyarray(color_frame.get_data())
        res = self.process_single_frame(c_img, depth_frame)
        if res is None:
            return None
            
        u, v, raw_x, raw_y, contour, filtered_c_img, raw_color_img = res
        vis_img = filtered_c_img.copy()

        if contour is not None:
            cv2.drawContours(vis_img, [contour], -1, (0, 255, 0), 2)

        X_coord_mm = int(round((raw_x * self.x_scale) + self.x_offset))
        Y_coord_mm = int(round((raw_y * self.y_scale) + self.y_offset))
        Z_coord_mm = int(self.z_height)

        radial_angle_rad = math.atan2(X_coord_mm, Y_coord_mm)
        orientation = float(np.clip(math.degrees(radial_angle_rad), -30.0, 30.0))

        cv2.circle(vis_img, (u, v), 7, (0, 0, 255), -1)

        text_x = f"X: {X_coord_mm} mm"
        text_y = f"Y: {Y_coord_mm} mm"
        text_z = f"Z: {Z_coord_mm} mm"
        text_o = f"O: {orientation:.2f} deg"

        cv2.rectangle(vis_img, (10, 10), (550, 135), (0, 0, 0), -1)
        cv2.putText(vis_img, text_x, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2)
        cv2.putText(vis_img, text_y, (20, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2)
        cv2.putText(vis_img, text_z, (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 255), 2)
        cv2.putText(vis_img, text_o, (20, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2)
        cv2.putText(vis_img, "[i] Shrani sliko | [p] Poslji | [q/Esc] Izstopi", (20, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 0), 1)

        text_lines = [text_x, text_y, text_z, text_o]
        return vis_img, (X_coord_mm, Y_coord_mm, Z_coord_mm, orientation), raw_color_img, contour, u, v, text_lines

    def open_preview(self):
        """
        Odpre predogled v živo. Ko uporabnik pošlje koordinate ('p') ali izstopi ('q'), 
        se okno ZAPRE in funkcija vrne nadzor vaši glavni skripti.
        """
        self.data_sent = False  # Ponastavimo zastavico ob vsakem vstopu
        
        cv2.namedWindow("RealSense - Predogled", cv2.WINDOW_NORMAL)
        cv2.startWindowThread()
        print("\n[INFO] Odprt predogled kamere. 's'=meritev, 'q'=izhod...")

        while True:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            live_color_filt, live_depth_filt = self.filter_by_distance(color_image, depth_image)
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(live_depth_filt, alpha=0.08), cv2.COLORMAP_JET
            )

            live_view = np.hstack((live_color_filt, depth_colormap))
            cv2.putText(live_view, f"COLOR ({self.min_dist}-{self.max_dist}mm)", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(live_view, f"DEPTH ({self.min_dist}-{self.max_dist}mm)", (1310, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(live_view, "Pritisni 's' za meritev | 'q' za izhod", (30, 700), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            cv2.imshow("RealSense - Predogled", live_view)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                print("[INFO] Zapiram predogled (brez pošiljanja)...")
                break

            elif key == ord('s'):
                res = self.run_measurement(color_frame, depth_frame)
                if res is not None:
                    vis_result, coords, raw_color_img, contour, u, v, text_lines = res
                    cv2.imshow("RealSense - Predogled", vis_result)
                    
                    while True:
                        decision_key = cv2.waitKey(0) & 0xFF
                        if decision_key == ord('i'):
                            self.save_full_color_image(raw_color_img, contour, u, v, text_lines)
                        elif decision_key == ord('p'):
                            x, y, z, o = coords
                            self.send_to_stm32(x, y, z, o)
                            break  # Po pošiljanju gremo ven iz zanke segmentacije
                        elif decision_key == ord('q') or decision_key == 27: # Esc ali 'q'
                            break

                    if self.data_sent:
                        # Če smo poslali podatke, zapremo predogled in vrnemo nadzor mehanizmu
                        break
                else:
                    print("[DEBUG] Objekt ni bil zaznan.")

        # Zapremo okno predogleda
        cv2.destroyAllWindows()
        cv2.waitKey(1)

    def close(self):
        """Ugasne vir kamere in zapre serijsko povezavo ob koncu celotnega programa."""
        self.pipeline.stop()
        if self.ser and self.ser.is_open:
            self.ser.close()
        print("[INFO] Kamera in serijska vrata zaprta.")
