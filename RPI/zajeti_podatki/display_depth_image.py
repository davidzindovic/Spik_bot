import numpy as np
import cv2

def prikazi_globinsko_sliko(pot_do_datoteke):
    try:
        # 1. Nalaganje numpy datoteke
        # Globinski podatki so običajno shranjeni kot uint16 (milimetri)
        depth_image = np.load(pot_do_datoteke)

        # Preverimo dimenzije
        print(f"Slika uspešno naložena. Resolucija: {depth_image.shape}")

        # 2. Normalizacija podatkov
        # Ker so surovi podatki v milimetrih (npr. 0-5000), jih moramo 
        # pretvoriti v 8-bitni format (0-255) za prikaz.
        depth_display = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

        # 3. Uporaba barvne sheme (ColorMap)
        # COLORMAP_JET ustvari značilen RealSense videz (modra = daleč, rdeča = blizu)
        depth_colormap = cv2.applyColorMap(depth_display, cv2.COLORMAP_JET)

        # 4. Prikaz slike v oknu
        cv2.imshow('RealSense Globinska Slika', depth_colormap)
        
        print("Pritisni katerokoli tipko za izhod.")
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    except FileNotFoundError:
        print(f"Napaka: Datoteke na poti '{pot_do_datoteke}' ni mogoče najti.")
    except Exception as e:
        print(f"Prišlo je do napake: {e}")

if __name__ == "__main__":
    # Tukaj vpiši ime svoje datoteke
    POT_DO_SLIKE = '20251214_210556_globina_surova.npy' 
    prikazi_globinsko_sliko(POT_DO_SLIKE)
