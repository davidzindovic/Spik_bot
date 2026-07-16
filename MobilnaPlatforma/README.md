Za upravljanje mobilne platforme preko Python ukazov je potrebno:

- namestiti knjižnico *pyads* s pomočjo ukaza ```pip install pyads```
- vzpostaviti Route s pomočjo TwinCAT programske opreme (glej poglavje POVEZAVA)
- spisati kodo s primernimi ukazi za pisanje/branje

Pozor! Koda mora konstantno pošiljati timestamp (številko cikla), da koda deluje! (glej primer)

# POVEZAVA
V primeru uporabe FANUC prenosnika bodo nastavitve verjetno:
```
LOKALNI_AMS_ID = "192.168.65.121.1.1"  # AMS_ID računalnika
PLC_IP = "192.168.64.200"              # IP krmilnika
PLC_AMS_ID = "169.254.220.1.1.1"       # AMS ID krmilnika
```

IP krmilnika je potrebno nastaviti kot statičnega:

1. Odpreš *Nadzorna plošča* (angl. Control Panel)
2. Navigiraš v podmeni *Network and Internet* in za tem v *Network and Sharing Center*
3. Na strani najdeš *Change adapter setting*
4. Desni klik na opcijo *Ethernet* in izbereš *Properties*
5. Najdeš *Internet Protocol Verison 4* in izbereš ter odpreš *Properties*
6. V novem oknu izbereš *Use the following IP address:*, z nastavitvami:
```
IP address: 192.168.64.200
Subnet Mask 255.255.255.0
Default Gateway: 192.168.64.1
```
7. Izbereš *OK* in *Close*.
8. Nato onemogočiš Ethernet z *desni klik na Ethernet* in *Disable*
9. Počakaš par sekund, nato spet omogočiš z *desni klik na Ethernet* in *Enable*

Za informacijo o AMS ID krmilnika lahko (z zagnanim TwinCAT) spodaj desno v orodni vrstici z *desni klik na TwinCAT Config Mode* in izbiro *Route* in *Edit Routes*. Zanima nas CX-2A5526, v tisti vrstici najdemo informacije.


# PYTHON UKAZI
```
pyads.open_port()
pyads.close_port()
plc = pyads.Connection(PLC_AMS_ID, pyads.PORT_TC3PLC1, PLC_IP)
plc.write_by_name('MAIN.MasterContol.data.mode', 1, pyads.PLCTYPE_LREAL)
prebran_mode = plc.read_by_name("MAIN.mode", pyads.PLCTYPE_LREAL)
plc.close()
```
Pri ukazih za branje in pisanje bodite pozorni, kje lahko dostopate do željene spremenljivke. Če je to MAIN, potem je potrebno pred nazivom spremenljivke kot je prikazana v samem programu, dopisati še "MAIN". Pri tem se sklicujte na deklaracijo spremenljivke.

## UKAZI ZA PLATFORMO:

### Vklop motorjev
  ```
  plc.write_by_name('MAIN.MasterContol.data.mode', 1, pyads.PLCTYPE_LREAL)
  plc.write_by_name('MAIN.MasterContol.data.masterSwich', 1, pyads.PLCTYPE_LREAL)
```
### Zaustavitev koles (vrtenje)
```
    plc.write_by_name('MAIN.MasterContol.data.modeKolesHub[0]', 2, pyads.PLCTYPE_LREAL)
    plc.write_by_name('MAIN.MasterContol.data.modeKolesHub[1]', 2, pyads.PLCTYPE_LREAL)
    plc.write_by_name('MAIN.MasterContol.data.modeKolesHub[2]', 2, pyads.PLCTYPE_LREAL)
    plc.write_by_name('MAIN.MasterContol.data.modeKolesHub[3]', 2, pyads.PLCTYPE_LREAL)
```

### Vklop steeringa
```
    plc.write_by_name('MAIN.MasterContol.data.steeringMode', 1, pyads.PLCTYPE_LREAL)
```
Pozor: kolesa se ob tej komandi poravnajo naravnost

### Branje spremenljivk
```
prebran_mode = plc.read_by_name("MAIN.mode", pyads.PLCTYPE_LREAL)
```

### Zasuk koles (vseh?)
```
kot=10
plc.write_by_name('STEERING.WHEEL_1_ANG_REF', kot, pyads.PLCTYPE_LREAL)
```
### Hitrost vrtenja koles
```
set_speed=10
plc.write_by_name('MAIN.CartContol.data.hitrost_ms', set_speed, pyads.PLCTYPE_LREAL)
```

### Pošiljanje TimeStampa
```
timestamp+=1
 plc.write_by_name('MAIN.CartContol.data.timeStamp', timestamp, pyads.PLCTYPE_LREAL)
```
