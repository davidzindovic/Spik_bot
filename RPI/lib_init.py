
#import needed modules and install them if needed

from datetime import datetime
#import os
#import ctypes

def library_setup():

    try:
        import serial
    except:
        os.system('python -m pip install serial')
    finally:
        import serial

	try:
        import time
    except:
        os.system('python -m pip install time')
    finally:
        import time

	try:
        import cv2
    except:
        os.system('python -m pip install python-opencv')
    finally:
        import cv2
	
    try:
        import keyboard
    except:
        os.system('python -m pip install keyboard')
    finally:
        import keyboard

    try:
        import matplotlib.pyplot as plt
    except:
        os.system('python -m pip install matplotlib')
    finally:
        import matplotlib.pyplot as plt



