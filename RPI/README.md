# Knjižnice
Za poenostavljanje programa so temu namenjene knjižnice za:
- namestitev potrebnih knjižnice = ```lib_init.py```
- serijsko komunikacijo z STM32 = ```spik_bot_serial.py```
- obdelavo shranjenih slik = ```data_capture_and_process.py```
- zajem in shranjevanje slik = ```capture_and_save_images.py```

# Launch realsense GUI
```realsense-viewer```

# Virtual environment
Launch:
```
python -m venv virtual_environment_name
source virtual_environment_name/bin/activate
```

on lr@spikbot the venv is named "okolje"

Shutdown virtual environment:
```deactivate```

# Zajeti podatki
V mapi ```zajeti_podatki``` so pari barvnih in globinskih slik skupaj s programskimi datotekami, ki so po korakih prispevale k nastanku ```data_capture_and_process.py```.
Program ```capture_and_save_images.py``` je namenjen le zajemu in shranjevanju podatkov v podmapo ```zajeti_podatki```.
