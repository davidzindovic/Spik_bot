Za vision je bila uporabljena Intel RealSense D455 kamera. Pred zagonom programa ```vision_sam3.py``` je potrebno namestiti knjižnice (glej poglavje KNJIŽNICE) in šele nato zagnati program. V primeru, da sta kaj pozabili, vas bo program opozoril oz. mogoče celo namestil namesto vas.

# KNJIŽNICE

Namestite sledeče knjižnice z zagonom primernih ukazov v ukaznem pozivu (angl. command prompt):

pyrealsense2 -> ```pip install pyrealsense2```

numpy -> ```pip install numpy```

opencv2 -> ```pip install opencv-python```

pyserial -> ```pip install pyserial```

pytorch, transformers... -> ```pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu``` -> ```pip install transformers```

Opomba: v primeru težav pri nalaganju pytorch naj omenim, da je ukaz za namestitev različice, ki deluje na procesorju. Obstaja možnost, da bo težave povzročala knjižnica za zvok, v tem primeru zaženite: ```pip uninstall -y torch torchvision torchaudio``` in zadevo ponovno namestite.

V primeru dodatnih težav pri namestitvi torch paketa na windowsih uporabite: ```conda install -c pytorch cpuonly mkl intel-openmp -y```. Ob zagonu ukaza ```python -c "import torch; print(torch.__version__)"``` v ukaznem pozivu, bi se vam morala izpisati različica pytorch s končnico +cpu.

# Segmentacija in izračun koordinat

## Pregled poteka

    **Zajem slik** – RealSense kamera vrne barvno in globinsko sliko (1280×720 px).

    **Filtracija po razdalji** – odstranimo vse točke izven območja MIN_DISTANCE_MM – MAX_DISTANCE_MM.

    **Segmentacija** – model CLIPSeg ("CIDAS/clipseg-rd64-refined") na podlagi besedila "tree trunk" vrne verjetnostno masko.

    **Maskiranje** – z uporabo praga 0.25 dobimo binarno masko.

    **Izbira točke** – znotraj maske poiščemo točko na določeni višini.

    **Izračun 3D koordinat** – pretvorba slikovnih koordinat v metrične vrednosti.

    **Orientacija** – izračun kota glede na X in Y.

## Ključne funkcije

- init_vision() - Inicializacija kamere, nastavitev disparityShift (trenutno 75). Nalaganje CLIPSeg modela in procesorja.

- filter_by_distance()	- Ustvari masko veljavnih globin znotraj MIN_DISTANCE_MM in MAX_DISTANCE_MM. Vrne filtrirani barvni in globinski sliki.

- process_single_frame()	- Glavna obdelava enega okvirja. Uporabi filtre, izvede CLIPSeg segmentacijo, vrne koordinate (u, v), surove X in Y, konturo, filtrirano in originalno barvno sliko.

- run_measurement()	- Kliče process_single_frame, izračuna končne koordinate X, Y, Z, O in pripravi slike za prikaz in shranjevanje. Meri čas segmentacije (segmentation_elapsed).

- create_annotated_image()	- Ustvari sliko za shranjevanje: originalna barvna slika z narisano konturo, rdečo piko in besedilom.

- create_preview_frame()	- Ustvari pogled z dvema slikama (barvna in globinska) za predogled.
- 
## Parametri, ki vplivajo na izbiro točke

- Parameter | Vrednost |	Pomen
- MIN_DISTANCE_MM |	200 |	Najmanjša razdalja za upoštevanje (mm)
- MAX_DISTANCE_MM	| 400	|Največja razdalja za upoštevanje (mm)
- Z_HEIGHT_MM	| 100	| Višina točke nad tlemi (mm) – določa, v kateri vrstici slike iščemo
- mask_threshold |	0.25 |	Prag za pretvorbo verjetnostne maske v binarno
- disparityShift |	75	| Nastavitev za izboljšanje globinske natančnosti (spreminjaj previdno)

## Kako se izbere točka

    Iz maske se izračuna približna globina (approx_depth_m) – mediana vseh veljavnih globin znotraj maske.

    Z uporabo višine kamere (Z_HEIGHT_MM) in goriščne razdalje (fy) se izračuna vrstica v:

```
    target_v = cy - ((Z_HEIGHT_MM / 1000.0) * fy / approx_depth_m)
    v = int(np.clip(target_v, 0, 719))
```
    V tej vrstici se poiščejo stolpci, kjer je maska aktivna. u je povprečje teh stolpcev (sredina debla v tej vrstici). Če v tej vrstici ni maske, se uporabi težišče konture.

    Globina v točki se izračuna kot mediana v okolici velikosti 7×7 pikslov.

    Z rs2_deproject_pixel_to_point se dobi točka v 3D prostoru:
```
        raw_y_mm = rs_coords[2] * 1000 (razdalja naprej)

        raw_x_mm = -rs_coords[0] * 1000 (horizontalni odmik, obrnjen predznak)
```
## Izračun orientacije
```
radial_angle_rad = math.atan2(X_coord_mm, Y_coord_mm)
radial_angle_deg = math.degrees(radial_angle_rad)
orientation = np.clip(radial_angle_deg, -30.0, 30.0)
```
orientation je kot, za katerega se mora platforma obrniti, da bo usmerjena proti točki.

0° pomeni, da je točka na sredini slike (X = 0).

Pozitivne vrednosti pomenijo zavoj v desno, negativne v levo.

## Kako spremeniti obnašanje

### Če želiš, da je orientacija vedno 0 (radialno)

V funkciji run_measurement zamenjaj:

orientation = float(np.clip(radial_angle_deg, -30.0, 30.0))

z:

orientation = 0.0

### Če želiš izbrati najbližjo točko (minimalno globino)

Namesto trenutnega iskanja po vrstici uporabi:

```
mask_indices = np.where(mask)
depths_in_mask = depth_filtered[mask]
min_idx = np.argmin(depths_in_mask)
u = mask_indices[1][min_idx]
v = mask_indices[0][min_idx]
depth_raw = depths_in_mask[min_idx]
```

### Sprememba višine vboda

Spremeni Z_HEIGHT_MM (npr. na 150 mm za višjo točko).

### Sprememba območja veljavnih razdalj

Prilagodi MIN_DISTANCE_MM in MAX_DISTANCE_MM.

### Sprememba občutljivosti segmentacije

Spremeni prag v process_single_frame:
```
mask = (probs > 0.25).cpu().numpy().astype(bool)
```

## Pomembne globalne spremenljivke
Spremenljivka	= Opis
segmentation_elapsed =	Čas, ki ga je segmentacija potrebovala (shrani se v CSV kot korak 0)
last_color_frame =	Zadnja barvna slika
last_depth_frame =	Zadnja globinska slika
vision_segmentation_result =	Rezultat segmentacije (slika, koordinate, kontura, ...)
vision_preview_mode	True = prikaz predogleda, False = prikaz rezultata

## Pogosti ukazi za testiranje

Segmentacija – pritisni A v predogledu.

Sprejem koordinat – pritisni A v rezultatu (začne meritev).

Zavrnitev – pritisni B v rezultatu (slika se shrani, CSV zabeleži čas segmentacije).

Kalibracija – pritisni BACK (gumb 6) v prostem teku.

## Shranjevanje slik

Slika segmentacije se shrani v mapo meritve/MERITEV_XXX/segmentacija.png takoj po uspešni segmentaciji.

Slika vsebuje originalno barvno sliko z narisano konturo, rdečo piko in izpisanimi koordinatami.

## Opombe

disparityShift vpliva na natančnost globine – če se ti zdi, da so razdalje napačne, poskusi spremeniti to vrednost (trenutno 75).

CLIPSeg model deluje na CPU, zato segmentacija traja nekaj sekund – to je normalno.

Če želiš hitrejšo segmentacijo, lahko zmanjšaš velikost slike (trenutno 1280×720).
