import serial
import time
import binascii
import keyboard

#SERIAL_DEVICE_NAME_RX = "/dev/ttyO4"
#SERIAL_DEVICE_NAME_TX = "/dev/ttyO5"
#SERIAL_DEVICE_NAME_LG = "/dev/ttyO1"
SERIAL_DEVICE = "COM13"

print 'open log'
f = open("a.nmea")
d = f.readlines();
ser = serial.Serial(SERIAL_DEVICE)
br = 38400
ser.baudrate = br
ser.flushInput()

def nmv(s, v = 30):
    if s.find('RMC') > 0:
        sp = s.split(',')
        sp[7] = '{:f}'.format(v)
        news = ','.join(sp)
        #print news
        rn = news.split('*')[0]
        nm(rn)  
    else:
        nm(s)     

def nm(s):
    crc = 0
    for c in s:
        if c != '$':
             crc = crc ^ ord(c)
        
    s += '*{:02X}'.format(crc)
    print s
    ser.write(s+'\n')

def run():
    print 'Started'
    
    while not keyboard.is_pressed('a'):
        print '.'   
        nm('$GPRMC,104426.591,A,5920.7019,N,01803.2893,E,0.117980,320.93,141204,,')
        nm('$GPGGA,104427.591,5920.7009,N,01803.2938,E,1,05,3.3,78.2,M,23.2,M,0.0,0000')
        nm('$GPGSA,A,3,05,24,17,30,02,,,,,,,,5.6,3.3,4.5')
        nm('$GPGSV,3,1,12,30,72,254,30,05,70,125,39,24,37,083,43,02,36,113,45')
        time.sleep(0.3)
        
    print 'Started main loop'
    
    for s in d:
        nmv(s)
        time.sleep(0.01)  
        if keyboard.is_pressed('q'):
            ser.close()
            
        
        
run()
       
        
