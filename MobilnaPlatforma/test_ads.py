import pyads
import time

# 1. NASTAVI LOKALNI AMS NETID (NetID tvojega računalnika, kjer teče Python)
# To pove krmilniku, kdo ga sprašuje. Zamenjaj s svojim dejanskim NetID-jem!
LOKALNI_AMS_ID = "192.168.65.121.1.1" 

pyads.open_port()
#pyads.set_local_address(LOKALNI_AMS_ID)
pyads.close_port()

# 2. PODATKI O KRMILNIKU (PLC)
PLC_IP = "192.168.64.200"
# Zamenjaj z dejanskim AMS NetID-jem krmilnika (če je drugačen kot IP + .1.1)
#PLC_AMS_ID = "192.168.64.200.1.1" 
PLC_AMS_ID = "169.254.220.1.1.1"

# Odpiranje povezave (posredujemo AMS NetID, ADS Port in IP naslov)
plc = pyads.Connection(PLC_AMS_ID, pyads.PORT_TC3PLC1, PLC_IP)
plc.open()

timestamp=0
prebran_mode=-1
old_mode=-1

set_speed=50;

try:
    # Preveri, če je povezava res vzpostavljena (izpiše ime naprave in verzijo)
    print("Povezan na:", plc.read_device_info())
    
    plc.write_by_name('MAIN.MasterContol.data.mode', 1, pyads.PLCTYPE_LREAL)
    plc.write_by_name('MAIN.MasterContol.data.masterSwich', 1, pyads.PLCTYPE_LREAL)
    plc.write_by_name('MAIN.CartContol.data.timeStamp', 1, pyads.PLCTYPE_LREAL)
    plc.write_by_name('MAIN.MasterContol.data.maxSpeedMode', set_speed, pyads.PLCTYPE_LREAL)
    
    plc.write_by_name('MAIN.MasterContol.data.modeKolesHub[0]', 2, pyads.PLCTYPE_LREAL)
    plc.write_by_name('MAIN.MasterContol.data.modeKolesHub[1]', 2, pyads.PLCTYPE_LREAL)
    plc.write_by_name('MAIN.MasterContol.data.modeKolesHub[2]', 2, pyads.PLCTYPE_LREAL)
    plc.write_by_name('MAIN.MasterContol.data.modeKolesHub[3]', 2, pyads.PLCTYPE_LREAL)
    
    plc.write_by_name('MAIN.MasterContol.data.steeringMode', 1, pyads.PLCTYPE_LREAL)
    
    '''
            if (timestamp==200):
            set_speed=0
            plc.write_by_name('MAIN.CartContol.data.hitrost_ms', set_speed, pyads.PLCTYPE_LREAL)
            print("Nastavljena hitrost:",set_speed)
    '''
    # Branje spremenljivke
    # OPOMBA: Pri read_by_name je pametno definirati tudi tip podatkov (npr. pyads.PLCTYPE_INT)
    while(True):
        prebran_mode = plc.read_by_name("MAIN.mode", pyads.PLCTYPE_LREAL)
        
        if prebran_mode!=old_mode:
            print("Mode:", int(prebran_mode))
            old_mode=prebran_mode
        timestamp+=1
        if (timestamp==100):
                
            print("Angle:",0)
        if (timestamp==200):
            plc.write_by_name('STEERING.WHEEL_1_ANG_REF', 10, pyads.PLCTYPE_LREAL)
            print("Angle:",10)
        if (timestamp==300):
            plc.write_by_name('STEERING.WHEEL_1_ANG_REF', 0, pyads.PLCTYPE_LREAL)
            print("Angle:",0)
        if (timestamp==400):
            plc.write_by_name('STEERING.WHEEL_1_ANG_REF', -10, pyads.PLCTYPE_LREAL)
            print("Angle:",-10)
        if (timestamp==500):
            plc.write_by_name('STEERING.WHEEL_1_ANG_REF', 0, pyads.PLCTYPE_LREAL)
            print("Angle:",0)
        plc.write_by_name('MAIN.CartContol.data.timeStamp', timestamp, pyads.PLCTYPE_LREAL)
        #print("Poslan timestamp:", timestamp)
        #time.sleep(0.1)

except Exception as e:
    print("Napaka pri branju:", e)

finally:
    plc.close()
