Za vision je bila uporabljena Intel RealSense D455 kamera. Pred zagonom programa ```vision_sam3.py``` je potrebno namestiti knjižnice (glej poglavje KNJIŽNICE) in šele nato zagnati program. V primeru, da sta kaj pozabili, vas bo program opozoril oz. mogoče celo namestil namesto vas.

# KNJIŽNICE

Namestite sledeče knjižnice z zagonom primernih ukazov v ukaznem pozivu (angl. command prompt):

pyrealsense2 -> ```pip install pyrealsense2```

numpy -> ```pip install numpy```

opencv2 -> ```pip install opencv-python```

pyserial -> ```pip install pyserial```

pytorch, transformers... -> ```pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu``` -> ```pip install transformers```

Opomba: v primeru težav pri nalaganju pytorch naj omenim, da je ukaz za namestitev različice, ki deluje na procesorju. Obstaja možnost, da bo težave povzročala knjižnica za zvok, v tem primeru zaženite: ```pip uninstall -y torch torchvision torchaudio``` in zadevo ponovno namestite.
