# realsense installation:
https://github.com/datasith/Ai_Demos_RPi/wiki/Raspberry-Pi-4-and-Intel-RealSense-D435

# USART:
Raspberry Pi 5 UART Setup
1. Enable UART on Raspberry Pi 5

The Raspberry Pi 5 has multiple UART options. The main ones are:

    ttyAMA0 - Primary UART (usually on GPIO 14/15)

    ttyS0 - Mini UART (secondary)

Enable UART in raspi-config:
bash

sudo raspi-config

    Select Interface Options → Serial Port

    Would you like a login shell to be accessible over serial? → No

    Would you like the serial port hardware to be enabled? → Yes

    Reboot when prompted

Alternatively, edit config files manually:
bash

sudo nano /boot/firmware/config.txt

Add these lines:
text

Enable primary UART
enable_uart=1

Disable Bluetooth to free up ttyAMA0 (optional)
dtoverlay=disable-bt

bash

sudo nano /boot/firmware/cmdline.txt

Remove any console=serial0,115200 or console=ttyAMA0,115200 entries to free the UART for your use.
