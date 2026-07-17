import time
import pyads
from inputs import get_gamepad, devices

# ==========================================
# NASTAVITVE ADS IN PLC
# ==========================================
REMOTE_IP = "192.168.65.82"
REMOTE_NETID = "5.30.230.184.1.1"

class SimpleGamepad:
    """Razred za branje ploščka, ki sledi spremembam stanja in izpisuje dogodke."""
    def __init__(self):
        self.x_axis = 0.0      # Smer (levo / desno)
        self.y_axis = 0.0      # Hitrost (naprej / nazaj)
        self.e_stop = 0.0      # Varnostni gumb (E-Stop)
        self.drive_mode = 1.0  # Način vožnje (1.0 = Drive, 0.0 = Stop)
        
        # Spremenljivke za zaznavanje sprememb (preprečujejo smetenje konzole)
        self.prev_x_axis = 0.0
        self.prev_y_axis = 0.0
        self.prev_e_stop = 0.0
        self.prev_drive_mode = 1.0

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
        print("                                 -> Krmili hitrost koles (hitrosti_koles)")
        print(" * LEVI ANALOG (X os / ABS_X)  : Zavijanje LEVO (-) / DESNO (+)")
        print("                                 -> Krmili kot zasuk koles (max_motor)")
        print(" * GUMB A (BTN_SOUTH)          : Varnostni izklop (E-STOP)")
        print("                                 -> Pritisnjen: takoj sprosti motorje (prosti tek)")
        print(" * GUMB START (BTN_START)      : Preklop načina vožnje (ZAGON / STOP)")
        print("                                 -> Omogoči ali popolnoma onemogoči pogon")
        print("="*70 + "\n")

    def update_and_log(self):
        """Prebere vnose in v realnem času izpisuje logiko delovanja ob spremembah."""
        if not devices.gamepads:
            return

        try:
            events = get_gamepad()
            for event in events:
                # 1. Levi analog Y (naprej/nazaj)
                if event.code == 'ABS_Y':
                    self.y_axis = round(event.state / 32768.0, 3)
                
                # 2. Levi analog X (levo/desno)
                elif event.code == 'ABS_X':
                    self.x_axis = round(event.state / 32768.0, 3)
                
                # 3. Gumb A (E-stop)
                elif event.code == 'BTN_SOUTH':
                    self.e_stop = float(event.state)
                
                # 4. Gumb Start (Zagon/Stop sistema)
                elif event.code == 'BTN_START' and event.state == 1:
                    self.drive_mode = 1.0 if self.drive_mode == 0.0 else 0.0

            # --- IZPISI DELOVANJA GLAVNIH FUNKCIJ ---
            
            # Izpis ob pritisku/sprostitev E-STOP (Gumb A)
            if self.e_stop != self.prev_e_stop:
                if self.e_stop == 1.0:
                    print("\n[VLOGA: GUMB A] -> PRITISNJEN E-STOP!")
                    print("  -> PLC-ju pošiljam signal za varnostni izklop (eStopMaster = 1.0).")
                    print("  -> Motorji se takoj sprostijo in preidejo v prosto premikanje.")
                else:
                    print("\n[VLOGA: GUMB A] -> E-STOP SPROŠČEN.")
                    print("  -> Sistem se vrača v pripravljenost za pogon.")
                self.prev_e_stop = self.e_stop

            # Izpis ob spremembi načina vožnje (Gumb START)
            if self.drive_mode != self.prev_drive_mode:
                if self.drive_mode == 1.0:
                    print("\n[VLOGA: GUMB START] -> NAČIN VOŽNJE: ZAGON")
                    print("  -> Master stikalo se vklopi (masterSwich = 1.0).")
                    print("  -> Kolesa so aktivirana in pripravljena na premik.")
                else:
                    print("\n[VLOGA: GUMB START] -> NAČIN VOŽNJE: STOP")
                    print("  -> Izklop delovanja motorjev (masterSwich = 0.0).")
                    print("  -> Hitrosti koles so varnostno postavljene na 0.0.")
                self.prev_drive_mode = self.drive_mode

            # Izpis ob premiku joysticka (samo ko je platforma aktivna in ni v E-stopu)
            if self.drive_mode == 1.0 and self.e_stop == 0.0:
                diff_y = abs(self.y_axis - self.prev_y_axis)
                diff_x = abs(self.x_axis - self.prev_x_axis)
                
                # Izpišemo le ob zaznavnem premiku, da ne smetimo zaslona
                if diff_y > 0.15 or diff_x > 0.15:
                    smer_y = "NAPREJ" if self.y_axis > 0 else "NAZAJ" if self.y_axis < 0 else "STREMI K 0"
                    smer_x = "DESNO" if self.x_axis > 0 else "LEVO" if self.x_axis < 0 else "STREMI K 0"
                    
                    print(f"[VLOGA: JOYSTICK] -> Premik osi | Hitrost koles: {self.y_axis:+.2f} ({smer_y}) | "
                          f"Kot krmiljenja: {self.x_axis:+.2f} ({smer_x})")
                    
                    self.prev_y_axis = self.y_axis
                    self.prev_x_axis = self.x_axis

        except Exception:
            pass


def main():
    # Zagon in vzpostavitev povezave s PLC
    print("Povezovanje s PLC-jem...")
    plc = pyads.Connection(REMOTE_NETID, pyads.PORT_SPS1, REMOTE_IP)
    
    try:
        plc.open()
        print("ADS povezava uspešno vzpostavljena.")
        
        # Inicializacija krmilnika in izpis navodil
        gamepad = SimpleGamepad()
        gamepad.print_instructions()
        
        # Števec ciklov, ki nadomešča real-time nanosekunde (preprečuje izpad komunikacije)
        cycle_timestamp = 0.0
        loop_rate = 0.02  # 50 Hz osveževanje (20 ms)
        
        print("Krmiljenje aktivno. Za izhod pritisnite Ctrl+C.")
        
        while True:
            start_time = time.time()
            
            # 1. Povečaj števec cikla za stabilen ADS Watchdog
            cycle_timestamp += 1.0
            
            # 2. Posodobi stanje ploščka in izpiši dogodke
            gamepad.update_and_log()
            
            # 3. Priprava paketov za PLC na podlagi trenutnih stanj ploščka
            if gamepad.e_stop == 0.0 and gamepad.drive_mode == 1.0:
                # --- Normalna vožnja ---
                master_data = [
                    cycle_timestamp, # Števec cikla
                    1.0,             # masterSwich ON
                    1.0,             # mode normal
                    100.0,           # hitrost v %
                    1.0, 1.0, 1.0, 1.0, # hub_motorji pripravljeni
                    1.0, 1.0, 1.0, 1.0, # max_motorji pripravljeni
                    0.0, 0.0, 0.0,      # dodatna oprema onemogočena
                    2.0              # steeringMode
                ]
                
                cart_data = [
                    cycle_timestamp,
                    gamepad.y_axis, gamepad.y_axis, gamepad.y_axis, gamepad.y_axis,  # hitrosti koles
                    gamepad.x_axis, gamepad.x_axis, gamepad.x_axis, gamepad.x_axis,  # krmiljenje (steering)
                    0.0, 0.0, 0.0,
                    0.0,             # eStopMaster OFF
                    0.0, 0.0, 0.0
                ]
            else:
                # --- Izklop ali aktiviran E-STOP ---
                master_data = [
                    cycle_timestamp, 0.0, 0.0, 0.0,
                    2.0, 2.0, 2.0, 2.0,  # stop hub motorji
                    2.0, 2.0, 2.0, 2.0,  # stop max_motorji
                    2.0, 2.0, 2.0, 0.0
                ]
                
                cart_data = [
                    cycle_timestamp,
                    0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 
                    gamepad.e_stop,      # eStopMaster signal PLC-ju
                    0.0, 0.0, 0.0
                ]
            
            # 4. Zapis v PLC (Vedno pošljemo nov timestamp)
            try:
                plc.write_by_name("MAIN.MasterContol.dataArray", master_data, pyads.PLCTYPE_ARR_LREAL(16))
                plc.write_by_name("MAIN.CartContol.dataArray", cart_data, pyads.PLCTYPE_ARR_LREAL(16))
            except pyads.ADSError as err:
                print(f"[PLC NAPAKA] Zapis ni uspel: {err}")
            
            # Ohranjanje stabilne frekvence zanke
            elapsed = time.time() - start_time
            time.sleep(max(0.0, loop_rate - elapsed))
            
    except KeyboardInterrupt:
        print("\n\n[INFO] Konec krmiljenja. Zaustavljam povezavo...")
    finally:
        plc.close()
        print("[INFO] ADS povezava zaprta. Srečno pot!")

if __name__ == "__main__":
    main()
