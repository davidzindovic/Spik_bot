# 26.10.2025:
- end switch pini spremenjeni na en kos na motor (prej bila dva). Prostanek rešen SW
- end SW za M1=PE3, M2=PH15, M3=PB4
- en switch pini imajo delujoče GPIO triggerje (potrebno urediti še ustrezen odziv)

useful link:
https://github.com/LAPSyLAB/STM32H7_Discovery_VIN_Projects/blob/main/STM32H750B-DK_VIN_Basic/Core/Src/main.c

# 13.10.2025:
- ST UART skor konc
- RPI lahko transmita UART (se mi zdi)
- lahko controllam driverje (rabm še probat vse motorje)
- čakam akse
- realsense (z GUI) se odpre na RPI. Treba stuhtat data stream

# 02.09.2025:
- del inštalacije librealsense na RPI narejen
- delno narejena kinematika na STM32
- dodana funkcija za pumpanje tekočine
- v razmislek vzet merjenje prodora v deblo

# 27.8.2025:
- treba poštimat TIM12_CH2 (manjka IRQn)
- treba stestirat TIM12 z A4988

# 15.8.2025: 
- timer interupti (PWM) in inkrementacija pozicije deluje
- motor struct naštiman
- direction pini pripravljeni (ish)
  
TO DO: 
- dodatek end switchov (mby 2 na joint?)
- analogni pini za end switche
- koda za kalibracijo
- opencv camera
- rpi za kamero in serial do stm?? alpa kamera na stm

# 25.7.2025:
- sample code za motorje spisan z menjavo smeri

TO DO: 
- ~~tranzistorji/optocouplerji za TTL~~
- ~~test kode za motorje~~
