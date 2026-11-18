# OPIS
SpikBot je manipulator namenjen robotizirani endoterapiji dreves, ki obratuje s pomočjo STM32H750B-DK razvojne plošče.
Drevo zazna s pomočjo Intel RealSense D455 RGB-D kamere, ki je povezana na osebni računalnik.
Po sadovnjaku premikamo manipulator s pomočjo AWD+AWS mobilne platforme, ki jo upravlja BeckHoff PLC krmilnik.

# REPOZITORIJ

- **Hardware** - Datoteke za električno shemo in električno vezje
- **Meritve** - Datoteke namenjene uvodnim meritvah sile in traja (arhivirano)
- **PC** - Programske datoteke, ki tečejo na osebnem računalniku
- **RPI** - Programske datoteke, ki tečejo na RaspberryPi (arhivirano)
- **STM32** - Programske datoteke, ki teče na STM32

# UPORABA

Napravo lahko uporabimo v DEMO načinu z vmesnim potrjevanjem ```PC/main_za_meritve.py``` ali skupaj z mobilno platformo ```PC/MobilnaPlatforma/mobilna_drive_main.py```.

Pred uporabo preverite periferijo tako, da se povežete na STM32 (micro-usb kabel) in odprete Putty. Če ne vidite izpisa pritisnite reset tipko na elektro škatlici. Navodila se izpišejo, a vseeno na kratko:
 - ```sw``` da preverite če stikala delujejo
 - ```tof``` da preverite če je povezava s TOF senzorjem razdalje primerna
 - ```manual``` da krmilite motorje ročno s tipkovnico. Izklopite z ukazom ```cc```

Če vse deluje, zaženete kalibracijo (lahko z UART ukazom preko Putty ali pa z "MODE" tipko na igralnem ploščku F710).

Na serijskem izpisu vas obvesti, ko konča kalibracijo. Sledi tipka "RT" da se vision odpre. Nato "A" da zajamete sliko, če so podatki vredu pa še enkrat "A" da pošljemo manipulator v točko.

Ko se ustavi lahko pošljemo iglo bližje drevesu s tipko "A". Zavrnemo z "B" in se vrne v domačo lego. V primeru, da se nepremakne premaknite prst pred TOF senzorjem.

Nato še en pritisk "A" proži vbod. Med premiki bodite pozorni, da se ne dotakne končnega stikala (prej nujno prekinite). V primeru uspešnega vboda (se ustavi na podlagi podatkov iz TOF senzorja) boste lahko črpali 1 ml vode.

V primeru uspešnega vboda bo robot po vrnitvi v končno lego prožil črpalko, da prečisti odprtine igle. 
