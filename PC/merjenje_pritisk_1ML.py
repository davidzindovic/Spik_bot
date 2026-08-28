import csv
import os
import re
import serial
import time
import matplotlib.pyplot as plt

# --- CONFIGURATION ---
COM_PORT = 'COM3'         # Nastavi ustrezen COM port (npr. '/dev/ttyUSB0' na Linuxu)
BAUD_RATE = 115200        # Nastavi hitrost serijske komunikacije
SUBFOLDER_NAME = 'MERITVE_1ML'

def get_next_filepaths(base_dir):
    """
    Poišče naslednji razpoložljiv številski indeks (00, 01, 02...)
    ter vrne poti za CSV in JPG datoteko.
    """
    target_dir = os.path.join(base_dir, SUBFOLDER_NAME)
    os.makedirs(target_dir, exist_ok=True)

    index = 0
    while True:
        csv_filename = f"meritev_{index:02d}.csv"
        jpg_filename = f"graf_{index:02d}.jpg"
        
        csv_path = os.path.join(target_dir, csv_filename)
        jpg_path = os.path.join(target_dir, jpg_filename)
        
        # Če CSV datoteka s tem indeksom še ne obstaja, uporabimo ta indeks
        if not os.path.exists(csv_path):
            return csv_path, jpg_path
        
        index += 1

def main():
    # Določi pot do imenika, kjer se nahaja ta skripta
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Inicializacija variabel za sledenje stanju
    recording = False
    start_time = None
    timestamps = []
    pressures = []

    # Regex vzorec za luščenje številke iz nizov oblika "\r\nPritisk: <stevilka>\r\n"
    pressure_pattern = re.compile(r'Pritisk:\s*([-+]?\d*\.?\d+)')

    print(f"Odpiram serijska vrata {COM_PORT} ({BAUD_RATE} baud)...")
    
    try:
        with serial.Serial(COM_PORT, BAUD_RATE, timeout=1) as ser:
            print("Čakam na začetek črpanja...")
            
            while True:
                # Branje vrstice iz serijskega vmesnika
                raw_line = ser.readline()
                if not raw_line:
                    continue
                
                # Dekodiranje bajtov v niz (ignoriramo morebitne napake pri dekodiranju)
                line = raw_line.decode('utf-8', errors='ignore')

                # 1. Preverjanje za začetek merjenja
                if "Zacetek crpanja 1mL" in line:
                    recording = True
                    start_time = time.time()
                    timestamps.clear()
                    pressures.clear()
                    print("Zaznan začetek črpanja! Snemanje podatkov...")
                    continue

                # 2. Preverjanje za konec merjenja
                if "Koncano crpanje 1mL." in line:
                    if recording:
                        print("Zaznan konec črpanja. Zaključujem snemanje...")
                        break
                    else:
                        print("Zaznan niz za konec, vendar se snemanje ni začelo.")

                # 3. Zajem podatkov med merjenjem
                if recording:
                    match = pressure_pattern.search(line)
                    if match:
                        current_time = time.time() - start_time
                        pressure_val = float(match.group(1))
                        
                        timestamps.append(current_time)
                        pressures.append(pressure_val)
                        print(f"Čas: {current_time:.3f} s | Pritisk: {pressure_val}")

    except serial.SerialException as e:
        print(f"Napaka pri povezavi na serijska vrata: {e}")
        return

    if not timestamps:
        print("Ni bilo zajetih podatkov za shranjevanje.")
        return

    # Določi novi poti za shranjevanje v podmapo MERITVE_1ML z inkrementalnim indeksom (00, 01, ...)
    output_csv, output_jpg = get_next_filepaths(script_dir)

    # --- SHRANJEVANJE V CSV ---
    print(f"Shranjujem podatke v '{output_csv}'...")
    with open(output_csv, mode='w', newline='', encoding='utf-8') as file:
        writer = csv.writer(file)
        writer.writerow(['Cas_s', 'Pritisk'])
        for t, p in zip(timestamps, pressures):
            writer.writerow([f"{t:.3f}", p])
    print("CSV datoteka uspešno shranjena.")

    # --- IZRIS IN SHRANJEVANJE GRAFA ---
    print(f"Generiram graf v '{output_jpg}'...")
    plt.figure(figsize=(10, 6))
    plt.plot(timestamps, pressures, marker='o', linestyle='-', color='b', label='Pritisk')
    plt.title('Prikaz pritiska med črpanjem 1 mL')
    plt.xlabel('Čas [s]')
    plt.ylabel('Pritisk')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    
    # Shranjevanje v JPG format
    plt.savefig(output_jpg, format='jpg', dpi=300)
    plt.close()
    print("Graf uspešno shranjen!")

if __name__ == '__main__':
    main()
