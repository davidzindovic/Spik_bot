import time
from vision_sam3 import RealSenseTracker

def premakni_mehanizem():
    print("\n>>> MEHANIZEM SE ZASUKAVA (Upravljanje mehanizma) <<<")
    # TUKAJ DODAJTE TISTO KODO ZA ZASUK MEHANIZMA
    time.sleep(2) 
    print(">>> Mehanizem uspešno premaknjen! <<<\n")

def main():
    # 1. Enkratna inicializacija ob zagonu skripte
    tracker = RealSenseTracker(serial_port='COM3')

    try:
        while True:
            print("\n--- GLAVNI MENI ---")
            print("1. Odpri predogled kamere & izmeri")
            print("2. Zapri program")
            izbira = input("Izberi možnost (1 ali 2): ")

            if izbira == '1':
                # Odpre okno predogleda
                tracker.open_preview()

                # Ko zapreš okno ali pošlješ podatke, koda nadaljuje tukaj:
                if tracker.data_sent:
                    print("[SISTEM] Koordinate so bile poslane! Sedaj premikam mehanizem...")
                    premakni_mehanizem()
                    # Zastavica je v razredu že ponastavljena za naslednji vstop!
                else:
                    print("[SISTEM] Predogled zaprt brez pošiljanja koordinat.")

            elif izbira == '2':
                break

    finally:
        # Čiščenje virov ob izhodu iz programa
        tracker.close()

if __name__ == "__main__":
    main()
