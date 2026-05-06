import serial
import csv
import time
import os
import threading
import matplotlib.pyplot as plt
from datetime import datetime

# --- KONFIGURACIJA ---
SERIAL_PORT = 'COM6' 
BAUD_RATE = 115200
FOLDER_PATH = r"S:\LR_Studenti\Spik_Bot\PUS_projekt\meritve_pritisk_potisk"

filename_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
CSV_FILE_PATH = os.path.join(FOLDER_PATH, f'meritev_{filename_timestamp}.csv')

current_data = {"Izmerjeno": 0.0, "Nastavljeno": 0.0}
running = True

def keyboard_listener(ser):
    global running
    while running:
        try:
            user_input = input()
            if user_input:
                ser.write((user_input + '\r\n').encode('utf-8'))
        except (EOFError, KeyboardInterrupt):
            break

if not os.path.exists(FOLDER_PATH):
    os.makedirs(FOLDER_PATH)

print(f"--- ZAGON: {SERIAL_PORT} @ {BAUD_RATE} ---")
print(f"CSV (ločilo ';'): {CSV_FILE_PATH}")

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    
    # Odpremo datoteko s podpičjem kot ločilom
    with open(CSV_FILE_PATH, mode='w', newline='') as csvfile:
        fieldnames = ['Čas [s]', 'Izmerjeno', 'Nastavljeno']
        # DODANO: delimiter=';'
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames, delimiter=';')
        writer.writeheader()
        csvfile.flush()
        
        input_thread = threading.Thread(target=keyboard_listener, args=(ser,), daemon=True)
        input_thread.start()
        
        start_time = time.time()

        while running:
            if ser.in_waiting > 0:
                raw_bytes = ser.readline()
                line = raw_bytes.decode('utf-8', errors='ignore').strip()
                
                if line:
                    print(line)  # Echo v terminal
                    
                    rel_time = round(time.time() - start_time, 2)
                    found = False
                    
                    # Čiščenje za robustno iskanje
                    clean_line = line.replace(" ", "").lower()
                    
                    if "izmerjeno:" in clean_line:
                        try:
                            val_str = line.split(":")[-1].strip()
                            current_data["Izmerjeno"] = float(val_str)
                            found = True
                        except: pass
                            
                    elif "nastavljeno:" in clean_line:
                        try:
                            val_str = line.split(":")[-1].strip()
                            current_data["Nastavljeno"] = float(val_str)
                            found = True
                        except: pass

                    if found:
                        writer.writerow({
                            'Čas [s]': rel_time,
                            'Izmerjeno': current_data["Izmerjeno"],
                            'Nastavljeno': current_data["Nastavljeno"]
                        })
                        csvfile.flush()
                        os.fsync(csvfile.fileno())

            time.sleep(0.001)

except KeyboardInterrupt:
    print("\nUstavljam in zapiram...")
finally:
    running = False
    if 'ser' in locals() and ser.is_open:
        ser.close()

# --- GRAF ---
print("Priprava grafa...")
time.sleep(0.5)
t_p, y_i, y_n = [], [], []

if os.path.exists(CSV_FILE_PATH):
    with open(CSV_FILE_PATH, mode='r') as f:
        # Pri branju moramo prav tako določiti delimiter=';'
        reader = csv.DictReader(f, delimiter=';')
        for row in reader:
            try:
                t_p.append(float(row['Čas [s]']))
                y_i.append(float(row['Izmerjeno']))
                y_n.append(float(row['Nastavljeno']))
            except: continue

    if t_p:
        plt.figure(figsize=(10, 5))
        plt.plot(t_p, y_i, 'r-', label='Izmerjeno [bar]')
        plt.plot(t_p, y_n, 'b--', label='Nastavljeno [bar]')
        plt.title(f"Meritev tlaka\n{datetime.now().strftime('%d. %m. %Y')}")
        plt.xlabel("Čas [s]")
        plt.ylabel("Pritisk [bar]")
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.show()
