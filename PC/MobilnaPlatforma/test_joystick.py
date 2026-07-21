import sys
import time
import pygame

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("Napaka: Ni zaznanih joystickov.")
    sys.exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Spremljam vnose za: {joystick.get_name()}")
print("POZOR: Logika za D-pad in Levi Joystick je obrnjena v kodi zaradi napačne preslikave.")
print("Izhod s Ctrl+C.\n")

DEADZONE = 0.10

zadnje_aktivne_osi = {}
zadnji_aktivni_gumbi = set()
zadnji_aktivni_hats = {}

try:
    while True:
        pygame.event.pump()
        
        sprememba = False
        trenutni_izpis = []

        # 1. Preverjanje osi (Pygame pravi osi -> Koda jih izpiše kot D-pad)
        aktivne_osi = []
        for i in range(joystick.get_numaxes()):
            val = joystick.get_axis(i)
            if abs(val) > DEADZONE:
                zaokroženo = round(val, 2)
                # TUKAJ SMO ZAMENJALI IME V IZPISU
                aktivne_osi.append(f"  • D-pad (Os {i}): {zaokroženo:+.2f}")
                
                if zadnje_aktivne_osi.get(i) != zaokroženo:
                    sprememba = True
                    zadnje_aktivne_osi[i] = zaokroženo
            else:
                if i in zadnje_aktivne_osi:
                    sprememba = True
                    del zadnje_aktivne_osi[i]

        if aktivne_osi:
            trenutni_izpis.append("AKTIVIRAN D-PAD:")
            trenutni_izpis.extend(aktivne_osi)

        # 2. Preverjanje gumbov (ostane enako)
        aktivni_gumbi = set()
        for i in range(joystick.get_numbuttons()):
            if joystick.get_button(i):
                aktivni_gumbi.add(i)

        if aktivni_gumbi != zadnji_aktivni_gumbi:
            sprememba = True
            zadnji_aktivni_gumbi = aktivni_gumbi

        if aktivni_gumbi:
            gumbi_str = ", ".join(f"Gumb {g}" for g in sorted(aktivni_gumbi))
            trenutni_izpis.append("PRITISNJENI GUMBI:")
            trenutni_izpis.append(f"  • {gumbi_str}")

        # 3. Preverjanje D-pad-a (Pygame pravi D-pad -> Koda ga izpiše kot Levi Joystick)
        for i in range(joystick.get_numhats()):
            hat = joystick.get_hat(i)
            if hat != (0, 0):
                if zadnji_aktivni_hats.get(i) != hat:
                    sprememba = True
                    zadnji_aktivni_hats[i] = hat
            else:
                if i in zadnji_aktivni_hats:
                    sprememba = True
                    del zadnji_aktivni_hats[i]

        if zadnji_aktivni_hats:
            # TUKAJ SMO ZAMENJALI IME V IZPISU
            trenutni_izpis.append("LEVI JOYSTICK (premikan do konca):")
            for idx, pozicija in zadnji_aktivni_hats.items():
                # D-pad vrne (x, y), kar pretvorimo v bolj smiseln izpis za joystick
                smer_x = "Desno" if pozicija[0] > 0 else ("Levo" if pozicija[0] < 0 else "")
                smer_y = "Gor" if pozicija[1] > 0 else ("Dol" if pozicija[1] < 0 else "")
                smeri = " + ".join(filter(None, [smer_x, smer_y]))
                trenutni_izpis.append(f"  • Smer: {smeri} {pozicija}")

        # Izpis ob spremembi
        if sprememba and trenutni_izpis:
            print("\n--- NOVI VNOS ---")
            for vrstica in trenutni_izpis:
                print(vrstica)
            print("-" * 17)

        time.sleep(0.03)

except KeyboardInterrupt:
    print("\nProgram zaključen.")
    pygame.quit()
