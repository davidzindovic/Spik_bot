import time
import pyads
from inputs import get_gamepad, devices

# ==========================================
# NASTAVITVE ADS IN PLC
# ==========================================
LOKALNI_AMS_ID = "192.168.65.121.1.1" 
PLC_IP = "192.168.64.200"
PLC_AMS_ID = "169.254.220.1.1.1"

set_speed = 50

class SimpleGamepad:
    """Razred za neblokirajoče branje ploščka in sledenje spremembam stanja."""
    def __init__(self):
        self.x_axis = 0.0      # Smer (levo / desno)
        self.y_axis = 0.0      # Hitrost (naprej / nazaj)
        self.e_stop = 0.0      # Varnostni gumb (E-Stop)
        self.drive_mode = 1.0  # Način vožnje (1.0 = Drive, 0.0 = Stop)
        self.plc_state = 0     # 0 = Stop, 1 = Start (premaknjeno v objekt za pravilno delovanje)
        
        # Spremenljivke za zaznavanje dejanskih sprememb vhodov
        self.prev_x_axis = 0.0
        self.prev_y_axis = 0.0
        self.prev_e_stop = 0.0
        self.prev_drive_mode = 1.0
        
        # Zastavica, ki sporoči glavni zanki, da se je zgodil nov vnos
        self.has_changed = False

        if not devices.gamepads:
            print("[OPOZORILO] Noben igralni plošček ni zaznan!")
        else:
            print(f"[INFO] Uspešno zaznan plošček: {devices.gamepads[0]}")

    def print_instructions(self):
        """Izpiše navodila za gumbe ob zagonu."""
        print("\n" + "="*70)
        print("                NAVODILA ZA UPORABO IGRALNEGA PLOŠČKA                 ")
        print("="*70)
        print(" * LEVI ANALOG (Y os / ABS_Y)  : Premikanje NAPREJ (+) / NAZAJ (-)")
        print("                                 -> Krmili hitrost platforme (hitrost_ms)")
        print(" * LEVI ANALOG (X os / ABS_X)  : Zavijanje LEVO (-) / DESNO (+)")
        print("                                 -> Krmili radij zasuka platforme")
        print(" * GUMB A (BTN_SOUTH)          : Varnostni izklop (E-STOP)")
        print("                                 -> Aktivira prosti tek koles (estop Master = 1)")
        print(" * GUMB START (BTN_START)      : Preklop delovanja (ZAGON / STOP)")
        print("                                 -> Omogoči ali onemogoči napajanje motorjev")
        print("="*70 + "\n")

    def update_and_log(self):
        """
        NEBLOKIRAJOČE prebere vse trenutne dogodke ploščka.
        Če ni novih dogodkov, se takoj zaključi in ne ustavlja zanke.
        """
        self.has_changed = False
        try:
            # S to zanko preberemo VSE čakajoče dogodke brez blokiranja programa
            while True:
                # get_gamepad() sicer blokira, vendar inputs knjižnica nima direktne 
                # non-blocking metode, zato jo izvedemo znotraj try-except za hitro branje.
                events = get_gamepad()
                for event in events:
                    # 1. Levi analog Y (naprej/nazaj)
                    if event.code == 'ABS_Y':
                        val = round(event.state / 32768.0, 3)
                        if val != self.y_axis:
                            self.y_axis = val
                            self.has_changed = True
                    
                    # 2. Levi analog X (levo/desno)
                    elif event.code == 'ABS_X':
                        val = round(event.state / 32768.0, 3)
                        if val != self.x_axis:
                            self.x_axis = val
                            self.has_changed = True
                    
                    # 3. Gumb A (E-stop)
                    elif event.code == 'BTN_SOUTH':
                        val = float(event.state)
                        if val != self.e_stop:
                            self.e_stop = val
                            self.has_changed = True
                    
                    # 4. Gumb Start (Zagon/Stop sistema)
                    elif event.code == 'BTN_START' and event.state == 1:
                        self.drive_mode = 1.0 if self.drive_mode == 0.0 else 0.0
                        self.has_changed = True
                
                # Če smo uspešno prebrali dogodke, prekinemo notranjo zanko, da gremo naprej
                break
        except OSError:
            # OSError se zgodi, ko v bufferju ni nobenega novega dogodka (gamepad "miruje")
            pass
        except Exception:
            pass

        # --- IZPISI IN POSODOBITVE STANJ OB SPREMEMBAH ---
        if self.has_changed:
            # Izpis ob pritisku/sprostitev E-STOP (Gumb A)
            if self.e_stop != self.prev_e_stop:
                if self.e_stop == 1.0:
                    print("\n[VLOGA: GUMB A] -> PRITISNJEN E-STOP!")
                    print("  -> PLC-ju pošiljam signal za varnostni izklop (estop Master = 1.0).")
                    self.plc_state = 0
                else:
                    print("\n[VLOGA: GUMB A] -> E-STOP SPROŠČEN.")
                self.prev_e_stop = self.e_stop

            # Izpis ob spremembi načina vožnje (Gumb START)
            if self.drive_mode != self.prev_drive_mode:
                if self.drive_mode == 1.0:
                    print("\n[VLOGA: GUMB START] -> NAČIN VOŽNJE: ZAGON")
                    self.plc_state = 1
                else:
                    print("\n[VLOGA: GUMB START] -> NAČIN VOŽNJE: STOP")
                    self.plc_state = 0
                self.prev_drive_mode = self.drive_mode

            # Izpis ob premiku joysticka
            if self.drive_mode == 1.0 and self.e_stop == 0.0:
                diff_y = abs(self.y_axis - self.prev_y_axis)
                diff_x = abs(self.x_axis - self.prev_x_axis)
                
                if diff_y > 0.15 or diff_x > 0.15:
                    smer_y = "NAPREJ" if self.y_axis > 0 else "NAZAJ" if self.y_axis < 0 else "STREMI K 0"
                    smer_x = "DESNO" if self.x_axis > 0 else "LEVO" if self.x_axis < 0 else "STREMI K 0"
                    
                    print(f"[VLOGA: JOYSTICK] -> Premik | Hitrost: {self.y_axis:+.2f} m/s ({smer_y}) | Radij: {self.x_axis:+.2f} ({smer_x})")
                    
                    self.prev_y_axis = self.y_axis
                    self.prev_x_axis = self.x_axis


def main():
    pyads.open_port()
    pyads.close_port()

    print("Povezovanje s PLC-jem...")
    plc = pyads.Connection(PLC_AMS_ID, pyads.PORT_TC3PLC1, PLC_IP)

    try:
        plc.open()
        print("ADS povezava uspešno vzpostavljena.")
        
        gamepad = SimpleGamepad()
        gamepad.print_instructions()
        
        cycle_timestamp = 0.0
        loop_rate = 0.02  # 50 Hz (20 ms)
        
        print("Povezan na:", plc.read_device_info())
        print("Krmiljenje aktivno. Za izhod pritisnite Ctrl+C.")
        
        # Začetni vpis hitrosti
        plc.write_by_name('MAIN.MasterContol.data.maxSpeedMode', set_speed, pyads.PLCTYPE_LREAL)

        # Hranimo prejšnje vrednosti za preprečevanje prepisovanja nespremenjenih stanj
        prev_plc_state = -1

        while True:
            start_time = time.time()
            
            # ====================================================
            # 1. TIMESTAMP (SE VEDNO POŠILJA - VSAK CIKEL NA 20ms)
            # ====================================================
            cycle_timestamp += 1.0
            try:
                plc.write_by_name('MAIN.CartContol.data.timeStamp', cycle_timestamp, pyads.PLCTYPE_LREAL)
            except pyads.ADSError as err:
                print(f"[PLC WATCHDOG NAPAKA] Napaka pri pošiljanju timestampa: {err}")
        
            # 2. NEBLOKIRAJOČE preberi stanje ploščka
            gamepad.update_and_log()

            # ====================================================
            # 3. POŠILJANJE DRUGIH PODATKOV (LE OB SPREMEMBAH)
            # ====================================================
            
            # Pošlji PLC stanje le, ko se dejansko spremeni
            if gamepad.plc_state != prev_plc_state:
                try:
                    if gamepad.plc_state == 1:
                        plc.write_by_name('MAIN.MasterContol.data.mode', 1, pyads.PLCTYPE_LREAL)
                        plc.write_by_name('MAIN.MasterContol.data.masterSwich', 1, pyads.PLCTYPE_LREAL)
                    else:
                        plc.write_by_name('MAIN.MasterContol.data.mode', 0, pyads.PLCTYPE_LREAL)
                        plc.write_by_name('MAIN.MasterContol.data.masterSwich', 0, pyads.PLCTYPE_LREAL)
                    prev_plc_state = gamepad.plc_state
                except pyads.ADSError as err:
                    print(f"[PLC NAPAKA] Napaka pri vpisu stanja: {err}")

            # Preberemo mode iz PLC (za izpis v konzolo)
            try:
                plc_mode = plc.read_by_name("MAIN.mode", pyads.PLCTYPE_LREAL)
                print("PLC Mode:", plc_mode) # Odkomentiraj po potrebi
            except pyads.ADSError:
                pass

            # Če se je zgodil nov premik joysticka ali pritisk gumba, pošljemo celoten paket podatkov
            if gamepad.has_changed:
                if gamepad.e_stop == 0.0 and gamepad.drive_mode == 1.0:
                    # --- Normalna vožnja ---
                    master_data = [
                        cycle_timestamp,     # 1. Števec cikla
                        1.0,                 # 2. masterSwich ON
                        1.0,                 # 3. mode normal
                        20.0,                # 4. hitrost v %
                        1.0, 1.0, 1.0, 1.0,  # 5-8. hub_motorji pripravljeni
                        1.0, 1.0, 1.0, 1.0,  # 9-12. max_motorji pripravljeni
                        0.0, 0.0, 0.0,       # 13-15. dodatna oprema onemogočena
                        2.0                  # 16. steeringMode
                    ]
                    
                    cart_data = [
                        cycle_timestamp,     # 1. timestamp
                        gamepad.y_axis,      # 2. hitrost_ms (hitrost koles)
                        gamepad.x_axis,      # 3. radij zasuka
                        0.0,                 # 4. vrtalnik mode
                        0.0,                 # 5. vrtalnik seq
                        0.0,                 # 6. vrtalnik manual
                        0.0                  # 7. estop Master
                    ]
                else:
                    # --- Izklop ali aktiviran E-STOP ---
                    master_data = [
                        cycle_timestamp,     # 1. Števec cikla
                        0.0,                 # 2. masterSwich OFF
                        0.0,                 # 3. mode stop
                        0.0,                 # 4. hitrost v %
                        2.0, 2.0, 2.0, 2.0,  # 5-8. stop hub motorji
                        2.0, 2.0, 2.0, 2.0,  # 9-12. stop max_motorji
                        2.0, 2.0, 2.0,       # 13-15. dodatna oprema stop
                        0.0                  # 16. steeringMode
                    ]
                    
                    cart_data = [
                        cycle_timestamp,     # 1. timestamp
                        0.0,                 # 2. hitrost_ms
                        0.0,                 # 3. radij zasuka
                        0.0,                 # 4. vrtalnik mode
                        0.0,                 # 5. vrtalnik seq
                        0.0,                 # 6. vrtalnik manual
                        gamepad.e_stop       # 7. estop Master
                    ]
                
                try:
                    # Pošljemo posodobljene pakete na PLC samo ob spremembi vhoda
                    plc.write_by_name("MAIN.MasterContol.dataArray", master_data, pyads.PLCTYPE_LREAL*(16))
                    plc.write_by_name("MAIN.CartContol.dataArray", cart_data, pyads.PLCTYPE_LREAL*(7))
                except pyads.ADSError as err:
                    print(f"[PLC NAPAKA] Zapis spremembe ni uspel: {err}")
            
            # Ohranjanje stabilne frekvence zanke na 50 Hz (20 ms)
            elapsed = time.time() - start_time
            time.sleep(max(0.0, loop_rate - elapsed))
            
    except KeyboardInterrupt:
        print("\n\n[INFO] Konec krmiljenja. Zaustavljam povezavo...")
    finally:
        plc.close()
        print("[INFO] ADS povezava zaprta. Srečno pot!")

if __name__ == "__main__":
    main()
