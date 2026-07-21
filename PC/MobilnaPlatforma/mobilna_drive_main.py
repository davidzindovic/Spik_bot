import os
# Skrijemo pygame pozdravno sporočilo ob uvozu, da ohranimo čist izpis
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"

import time
import pygame
import pyads



# to do dead man switch speed kolesa se vrtijo po vklopu (sumim estop master podatek na beckhoff side)


# ==========================================
# NASTAVITVE ADS IN PLC
# ==========================================
LOKALNI_AMS_ID = "192.168.65.121.1.1" 
PLC_IP = "192.168.64.200"
PLC_AMS_ID = "169.254.220.1.1.1"

set_speed = 50
current_angle = 0.0
old_current_angle = 0.0
delta_angle = 0.0
max_angle = 30.0
angle_step = 10  # Za koliko stopinj se spremeni kot ob enem pritisku D-pada
mode_change_requested=False
timestamp_temp=0
cycle_timestamp=0

class SimpleGamepadPygame:
    """Razred za neblokirajoče branje ploščka s pomočjo pygame."""
    def __init__(self):
        pygame.init()
        pygame.joystick.init()
        
        self.joystick = None
        self.x_axis = 0.0      
        self.y_axis = 0.0      # Hitrost (naprej / nazaj)
        self.e_stop = 0.0      # Varnostni gumb (E-Stop)
        self.drive_mode = 1.0  # Način vožnje (1.0 = Drive, 0.0 = Stop)
        self.plc_state = 0     # 0 = Stop, 1 = Start
        self.dead_man_pressed = False  # Status Dead Man Switcha (LB / Gumb 4)
        
        # Tempomat in načini
        self.cruise_control_active = False
        self.selected_mode = 1.0  # Privzeti način ob zagonu
        self.mode_changed = False
        
        # Spremenljivke za zaznavanje sprememb
        self.prev_y_axis = 0.0
        self.prev_e_stop = 0.0
        self.prev_drive_mode = 1.0
        
        self.has_changed = False
        
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            print(f"[INFO] Uspešno zaznan plošček: {self.joystick.get_name()}")
        else:
            print("[OPOZORILO] Noben igralni plošček ni zaznan!")

    def print_instructions(self):
        """Izpiše navodila za gumbe ob zagonu."""
        print("\n" + "="*70)
        print("                NAVODILA ZA UPORABO IGRALNEGA PLOŠČKA                 ")
        print("="*70)
        print(" * DESNI ANALOG (Y os)          : Premikanje NAPREJ (+) / NAZAJ (-)")
        print("                                 -> Krmili hitrost platforme (hitrost_ms)")
        print(" * D-PAD (Levo / Desno)        : Nastavljanje kota koles (ZAKLENJENO STANJE)")
        print("                                 -> Kot ostane nastavljen, dokler ne pritisneš kontra!")
        print(" * GUMB LB (Zadaj levo - 4)    : DEAD MAN SWITCH (Drži za delovanje, spusti za E-STOP)")
        print(" * GUMB A (Gumb 0)             : Vklop / Izklop TEMPOMATA (Zakleni trenutno hitrost)")
        print(" * GUMBI Y (3), X (2), B (1)   : Nastavitev načina (Y -> Mode 1, X -> Mode 2, B -> Mode 3)")
        print(" * GUMB START (Gumb 7 / Menu)  : Preklop delovanja (ZAGON / STOP)")
        print("="*70 + "\n")

    def update_and_log(self):
        """
        Neblokirajoče prebere vse dogodke iz pygame čakalne vrste.
        """
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

        # --- 1. Branje analognih osi (Desni joystick Y os (indeks 3) za hitrost) ---
        if self.selected_mode!=5.0:
            if not self.cruise_control_active:
                raw_y = -self.joystick.get_axis(3) 
                y_val = round(raw_y, 3) if abs(raw_y) > 0.05 else 0.0
                
                if y_val != self.y_axis:
                    self.y_axis = y_val
                    self.has_changed = True
            else:
                # Če je tempomat vklopljen, obdržimo trenutno vrednost y_axis in ne beremo joysticka
                pass

        # Preberemo trenutno realno stanje Dead Man gumba (LB / indeks 4)
        self.dead_man_pressed = bool(self.joystick.get_button(4))
        

        # Če je Dead Man switch spuščen, takoj aktiviramo E-STOP
        if not self.dead_man_pressed:
            if self.e_stop != 1.0:
                self.e_stop = 1.0
                print("Dead man switch spuscen!")
                # Ob e-stopu za vsak slučaj izklopimo tudi tempomat
                self.cruise_control_active = False
                self.has_changed = True
        else:
            # Če je pritisnjen in tempomat ni aktiviran ali pa ni drugih E-STOP pogojev
            if self.e_stop == 1.0 and not self.joystick.get_button(0):
                self.e_stop = 0.0
                self.has_changed = True

        #ce smo zahtevali spremembo mode-a pustimo zadevo pri miru, da se ustavi
        if not mode_change_requested:
            # --- 3. Procesiranje diskretnih dogodkov (Gumbi in D-pad) ---
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == 7: # Gumb Start
                        self.drive_mode = 1.0 if self.drive_mode == 0.0 else 0.0
                        self.has_changed = True
                    elif event.button == 0 and self.dead_man_pressed: # Gumb A - TEMPOMAT
                        self.cruise_control_active = not self.cruise_control_active
                        self.has_changed = True
                        if self.cruise_control_active:
                            print(f"[TEMPOMAT] VKLOPLJEN pri hitrosti: {self.y_axis} m/s")
                        else:
                            print("[TEMPOMAT] IZKLOPLJEN. Nadzor prevzema joystick.")
                    elif event.button == 3: # Gumb Y -> Mode 1
                        self.selected_mode = 1.0
                        self.mode_changed = True
                        mode_change_requested=True
                        timestamp_temp=cycle_timestamp
                        print("[NAČIN] Izbran Mode 1 (Gumb Y)")
                    elif event.button == 2: # Gumb X -> Mode 2
                        self.selected_mode = 2.0
                        self.mode_changed = True
                        mode_change_requested=True
                        timestamp_temp=cycle_timestamp
                        print("[NAČIN] Izbran Mode 2 (Gumb X)")
                    elif event.button == 1: # Gumb B -> Mode 5
                        self.selected_mode = 5.0
                        self.mode_changed = True
                        mode_change_requested=True
                        timestamp_temp=cycle_timestamp
                        print("[NAČIN] Izbran Mode 3 (Gumb B)")

                # Zaznavanje pritiska na D-PAD (Hat)
                elif event.type == pygame.JOYHATMOTION and self.selected_mode==1.0:
                    hat_x, hat_y = event.value
                    
                    # KLJUČNA SPREMEMBA: Če je hat_x == 0, pomeni, da je uporabnik spustil D-pad.
                    # V tem primeru ne naredimo NIČ, s čimer zaklenemo trenutni kot!
                    if hat_x != 0 and not self.e_stop:  
                        new_angle = current_angle + (hat_x * angle_step)
                        
                        # Omejitev kota znotraj meja [-max_angle, max_angle]
                        if ((-max_angle <= new_angle) and (new_angle <= max_angle)):
                            current_angle = new_angle
                            delta_angle = current_angle - old_current_angle
                            self.has_changed = True
                            print(f"[D-PAD] Spreemba kota: {current_angle}° (Delta za PLC: {delta_angle}°)")

        # Preverimo če je prišlo do sprostitve Dead Man Switcha za takojšnjo proženje
        if not self.dead_man_pressed:
            self.e_stop = 1.0
            self.has_changed = True

        # --- IZPISI DELOVANJA ---
        if self.has_changed:
            if self.e_stop != self.prev_e_stop:
                if self.e_stop == 1.0:
                    if not self.dead_man_pressed:
                        print("\n[VARNOST] -> DEAD MAN SWITCH SPROŠČEN! Sprožam prisilni E-STOP!")
                    else:
                        print("\n[VARNOST] -> E-STOP AKTIVIRAN!")
                    self.plc_state = 0
                else:
                    print("\n[VARNOST] -> E-STOP SPROŠČEN.")
                self.prev_e_stop = self.e_stop

            if self.drive_mode != self.prev_drive_mode:
                if self.drive_mode == 1.0:
                    print("\n[VLOGA: GUMB START] -> NAČIN VOŽNJE: ZAGON")
                    self.plc_state = 1
                else:
                    print("\n[VLOGA: GUMB START] -> NAČIN VOŽNJE: STOP")
                    self.plc_state = 0
                self.prev_drive_mode = self.drive_mode

            if self.drive_mode == 1.0 and self.e_stop == 0.0:
                diff_y = abs(self.y_axis - self.prev_y_axis)
                if diff_y > 0.15:
                    smer_y = "NAPREJ" if self.y_axis > 0 else "NAZAJ" if self.y_axis < 0 else "STREMI K 0"
                    print(f"[VLOGA: JOYSTICK] -> Hitrost: {self.y_axis:+.2f} m/s ({smer_y}) | Trenutni kot koles: {current_angle}°")
                    self.prev_y_axis = self.y_axis


def main():
    global old_current_angle
    global current_angle
    global delta_angle
    global mode_change_requested
    global timestamp_temp
    global cycle_timestamp

    pyads.open_port()
    pyads.close_port()

    print("Povezovanje s PLC-jem...")
    plc = pyads.Connection(PLC_AMS_ID, pyads.PORT_TC3PLC1, PLC_IP)

    try:
        plc.open()
        print("ADS povezava uspešno vzpostavljena.")
        
        gamepad = SimpleGamepadPygame()
        gamepad.print_instructions()
        
        cycle_timestamp = 0.0
        loop_rate = 0.02  # 50 Hz
        
        print("Povezan na:", plc.read_device_info())
        print("Krmiljenje aktivno. Za izhod pritisnite Ctrl+C.")
        
        plc.write_by_name('MAIN.MasterContol.data.maxSpeedMode', set_speed, pyads.PLCTYPE_LREAL)

        plc_mode = 0
        old_plc_mode = 0
        prev_plc_state = -1

        # Glavna kontrolna zanka
        running = True

        while running:
            start_time = time.time()
            
            # ====================================================
            # 1. WATCHDOG (TIMESTAMP)
            # ====================================================
            cycle_timestamp += 1.0
            try:
                plc.write_by_name('MAIN.CartContol.data.timeStamp', cycle_timestamp, pyads.PLCTYPE_LREAL)
            except pyads.ADSError as err:
                print(f"[PLC WATCHDOG NAPAKA] Napaka pri pošiljanju timestampa: {err}")
        
            # 2. Osvežitev stanja ploščka
            gamepad.update_and_log()


            # ====================================================
            # 3. POŠILJANJE OSTALIH PODATKOV OB SPREMEMBAH
            # ====================================================
            if gamepad.plc_state != prev_plc_state:
                try:
                    if gamepad.plc_state == 1:
                        # Vpišemo trenutno izbrani način preko gumbov X, Y, B
                        plc.write_by_name('MAIN.MasterContol.data.mode', 1, pyads.PLCTYPE_LREAL)
                        plc.write_by_name('MAIN.MasterContol.data.masterSwich', 1, pyads.PLCTYPE_LREAL)
                        plc.write_by_name('MAIN.MasterContol.data.steeringMode', gamepad.selected_mode, pyads.PLCTYPE_LREAL)
                        print(f"START (Mode: {gamepad.selected_mode})")
                    else:
                        plc.write_by_name('MAIN.MasterContol.data.mode', 0, pyads.PLCTYPE_LREAL)
                        plc.write_by_name('MAIN.MasterContol.data.masterSwich', 0, pyads.PLCTYPE_LREAL)
                        print("STOP")
                    prev_plc_state = gamepad.plc_state
                except pyads.ADSError as err:
                    print(f"[PLC NAPAKA] Napaka pri vpisu stanja: {err}")

            try:
                plc_mode = plc.read_by_name("MAIN.mode", pyads.PLCTYPE_LREAL)
                if plc_mode != old_plc_mode:
                    print("PLC Mode:", plc_mode)
                    old_plc_mode = plc_mode
            except pyads.ADSError:
                pass

            #damo čas da se kolesa ustavijo in potem v rednem delu spremenimo mode
            if mode_change_requested:
                if ((cycle_timestamp-timestamp_temp)>100):
                    timestamp_temp=cycle_timestamp
                    mode_change_requested=False
                    plc.write_by_name('MAIN.MasterContol.data.steeringMode', gamepad.selected_mode, pyads.PLCTYPE_LREAL)
                else:
                    #print("Cakam, mode bo:", gamepad.selected_mode)
                    #print("Timestamp:",cycle_timestamp-timestamp_temp)
                    plc.write_by_name('MAIN.CartContol.data.hitrost_ms', 0, pyads.PLCTYPE_LREAL)
                

            if (gamepad.has_changed or not running) and not mode_change_requested:
                if gamepad.e_stop == 0 and gamepad.drive_mode == 1.0 and running and gamepad.selected_mode!=5.0:
                    # --- Normalna vožnja ---
                    master_data = [
                        cycle_timestamp,     # 1. Števec cikla
                        1.0,                 # 2. masterSwich ON
                        1.0,# 3. način glede na gumbe X, Y, B
                        20.0,                # 4. hitrost v %
                        1.0, 1.0, 1.0, 1.0,  # 5-8. hub_motorji pripravljeni
                        1.0, 1.0, 1.0, 1.0,  # 9-12. max_motorji pripravljeni
                        0.0, 0.0, 0.0,       # 13-15. dodatna oprema onemogočena
                        gamepad.selected_mode# 16. steeringMode
                    ]

                    cart_data = [
                        cycle_timestamp,     # 1. timestamp
                        gamepad.y_axis,      # 2. hitrost_ms (hitrost koles)
                        delta_angle,         # 3. radij zasuka
                        0.0,                 # 4. vrtalnik mode
                        0.0,                 # 5. vrtalnik seq
                        0.0,                 # 6. vrtalnik manual
                        0.0                  # 7. estop Master
                    ]
                elif gamepad.selected_mode!=5.0:
                    
                    if not gamepad.cruise_control_active:
                        gamepad.y_axis=0
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
                        1.0                  # 7. estop Master
                    ]
                

                try:
                    if not mode_change_requested and gamepad.selected_mode!=5.0:
                        plc.write_by_name("MAIN.MasterContol.dataArray", master_data, pyads.PLCTYPE_LREAL*16)
                    
                        plc.write_by_name("MAIN.CartContol.dataArray[0]", cart_data[0], pyads.PLCTYPE_LREAL)
                        plc.write_by_name("MAIN.CartContol.dataArray[1]", cart_data[1], pyads.PLCTYPE_LREAL)
                        plc.write_by_name("MAIN.CartContol.dataArray[3]", cart_data[3], pyads.PLCTYPE_LREAL)
                        plc.write_by_name("MAIN.CartContol.dataArray[4]", cart_data[4], pyads.PLCTYPE_LREAL)
                        plc.write_by_name("MAIN.CartContol.dataArray[5]", cart_data[5], pyads.PLCTYPE_LREAL)
                        plc.write_by_name("MAIN.CartContol.dataArray[6]", cart_data[6], pyads.PLCTYPE_LREAL)
                        
                        # Komanda se pošlje samo ob dejanski (znatni) spremembi delte
                        if abs(delta_angle) >= angle_step and running and not gamepad.e_stop and not mode_change_requested:
                            print(f"Zavijam na PLC -> Pošiljam : {current_angle} (Kot je sedaj zaklenjen na {current_angle}°)")
                            plc.write_by_name('STEERING.WHEEL_1_ANG_REF', current_angle, pyads.PLCTYPE_LREAL)
                            #plc.write_by_name("MAIN.CartContol.dataArray[2]", current_angle, pyads.PLCTYPE_LREAL)
                            old_current_angle = current_angle
                            delta_angle = 0.0  # Takoj ponastavimo delto, da se ne pošilja večkrat v prazno

                    else:
                        plc.write_by_name('MAIN.CartContol.data.timeStamp', cycle_timestamp, pyads.PLCTYPE_LREAL)
                        if gamepad.selected_mode==5.0:
                            plc.write_by_name('MAIN.CartContol.data.hitrost_ms', 0, pyads.PLCTYPE_LREAL)

                except pyads.ADSError as err:
                    print(f"[PLC NAPAKA] Zapis spremembe ni uspel: {err}")
            
            elapsed = time.time() - start_time
            time.sleep(max(0.0, loop_rate - elapsed))
            
    except KeyboardInterrupt:
        print("\n\n[INFO] Konec krmiljenja. Zaustavljam povezavo...")
    finally:
        plc.close()
        pygame.quit()
        print("[INFO] Povezave uspešno zaprte. Srečno pot!")

if __name__ == "__main__":
    main()
