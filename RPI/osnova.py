test = 0
#import needed modules and install them if needed
from datetime import datetime
import tkinter as tk
import tkinter.font as tkFont
from tkinter import filedialog as fd
import os
import ctypes
import shutil
import getpass
import time
import csv

global joze

def setup():
    global joze
    try:
        from PIL import Image, ImageTk
    except:
        os.system('python -m pip install pillow')
    finally:
        from PIL import Image, ImageTk

    try:
        import openpyxl
        from openpyxl import load_workbook
    except:
        os.system('python -m pip install openpyxl')
    finally:
        import openpyxl
        from openpyxl import load_workbook
        
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

    #if test is disabled, it sets up comunication to the scope
    if test == 0:
        try:
            import pyvisa
        except:
            os.system('python -m pip install -U pyvisa')
        finally:
            import pyvisa
	
    #if test is disabled, it sets up comunication to the scope
    if test == 0:
        #create instance of resource manager
        try:
            rm = pyvisa.ResourceManager()
        except:
            program_errors = "VISA ni bila uspešno zagnana! Poskusi ponovno, če ne deluje kontaktiraj Jan Žagar:\n jan.zagar@mahle.com, v nujnih primerih 051 224 904."

        #list available devices
        
        #'ASRL1::INSTR', 'ASRL3::INSTR', 'GPIB0::10::INSTR'

        #open comunication with chosen device which is listed above
        try:
            joze = rm.open_resource('GPIB0::28::INSTR')
        except:
            program_errors = "Povezava z joze ni bila uspešna! Preveri če je osciloskop povezan z Ethernet kablom,\n je ip pravilno nastavljen in je oscilskop pripravljen za komunikacijo. Ponovno zaženi aplikacijo!"

    #display device identification, to confirm which device you are conected to
    #print(scope.query("*IDN?"))
    #print(rm.list_resources())

#joze.write("APPL:SIN 1 KHZ, 3.0 VPP, -2.5 V")
#time.sleep(2)
#joze.write("APPL:SQU 2 KHZ, 2.0 VPP, 0.5 V")
#time.sleep(2)
#joze.write("APPL:TRI 1 KHZ, 5.0 VPP, 1.0 V")
#time.sleep(2)
#joze.write("FUNC:USER CARDIAC")
#joze.write("FUNC:SHAP USER")

def signal(oblika, frekvenca, amplituda, offset):
    global joze
    input=""
    if oblika=="sinus":
        input+="APPL:SIN"
    elif oblika=="trikot":
        input+="APPL:TRI"
    elif oblika=="pravokoten":
        input+="APPL:SQU"
    if oblika!="cardiac":
        input=input+f" {frekvenca} HZ, {amplituda}, {offset} V"
        print(input)
        joze.write(input)
    elif oblika=="cardiac":
        joze.write("FUNC:USER CARDIAC")
        joze.write("FUNC:SHAP USER")

