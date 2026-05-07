# Opis
Program beleži izmerjene podatke v .csv datoteko. Meritve opravlja STM32 razvojna plošča. Uporabnik lahko preko serijskega okna odprtega v Python okolju nastavlja željeni tlak in bere izmerjeni ter nastavjleni pritisk. Ob koncu programa (Ctrl+C) program shrani podatke in na podlagi shranjene .csv datoteke nariše grafikon.

## Potrebne knjižnice za zagon:
pyserial, matplotlib

## Sprememiti je potrebno:
- ```COM port (preverimo v Upravitelj naprav ali Device manager)```
- ```Pot do mape, v katero želimo shranjevati meritve```

# Potek meritve
Motor za vbod v deblo krmili bela ST razvojna plošča, pri čemer s pritiskom na modro tipko menjavamo smer potiska, s potenciometrom pa hitrost potiska.

Motor za črpalko krmili modra ST razvojna plošča z ekranom, ki tudi bere podatke iz senzorja (od primernega upora).

Za izvedbo meritve je potrebno vklopiti oba velika napajalnika na 230V, nato pa v poljubnem vrstnem redu še ST razvojni plošči (sta neodvisni ena od druge).

Ne pozabite tudi poskrbeti za primerno vmestitev dovodne cevi za tekočino (označena z maskircem in napisom "IN").

Zaženemo datoteko ```meritve_pritisk_stm32.py``` in spremljamo serijski monitor. Za vnos zaželjenega pritiska z levim klikom miške izberemo serijsko okno in nato le vpišemo zaželjeno številko (lahko z decimalno vrednostjo ali brez, npr. 5 ali 5.23) in nato potrdimo vnos s tipko Enter. Sprememnjena nastavitev za pritisk bo vidna v izpisu.

Ko želimo nehati z meritvijo pritisnemo CTRL+C, kar konča izvajanje Python skripte, shrani .csv datoteko meritev in izriše graf meritve.

## POMEMBNO
Na ta velikem laptopu v LR je desni USB vhod bližje uporabniku na COM6.

Če v serijskem oknu ne vidite izpisa v obliki: ```Izmerjeno: xxxxx.yyyyy``` ```Nastavljeno: aaaaaa.bbbbb``` poterm pritisnite črno tipko na modri ST razvojni plošči z ekranom.
