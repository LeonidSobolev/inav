import serial
import time
import binascii
import keyboard
from mavlinkcrc import x25crc
import fg


#SERIAL_DEVICE_NAME_RX = "/dev/ttyO4"
#SERIAL_DEVICE_NAME_TX = "/dev/ttyO5"
#SERIAL_DEVICE_NAME_LG = "/dev/ttyO1"
SERIAL_DEVICE = "COM13"

ser = None

def init_ports():
    print 'open log'
    global f
    f    = open("a.nmea")
    global d
    d    = f.readlines();
    global ser
    ser    = serial.Serial(SERIAL_DEVICE)
    br = 57600
    ser.baudrate = br
    ser.flushInput()



def nm(s):
    crc = 0
    for c in s:
        if c != '$':
             crc = crc ^ ord(c)
        
    s += '*{:02X}'.format(crc)
    print s
    ser.write(s+'\n')

def send_coord( la, lo, h, t, az ):
    nm('$GPRMC,104426.591,A,5920.7019,N,01803.2893,E,0.117980,320.93,141204,,')
    nm('$GPGGA,104427.591,5920.7009,N,01803.2938,E,1,05,3.3,78.2,M,23.2,M,0.0,0000')
    nm('$GPGSA,A,3,05,24,17,30,02,,,,,,,,5.6,3.3,4.5')
    nm('$GPGSV,3,1,12,30,72,254,30,05,70,125,39,24,37,083,43,02,36,113,45')

def parse_servo(pl):
    s = ''
    servo = []
    servo2 = []
    plb = bytearray(pl)
    s = ''
    for x in plb:
        s  += '{:02x} '.format(x)
    print s
    for i in range(8):
        servo.append(  ord(pl[5+i*2]) + 8 * ord(pl[4+i*2]) ) 
        servo2.append(  ord(pl[5+i*2]) + 8 * ord(pl[6+i*2]) ) 
    
    print servo
    print servo2

def parse_mavlink():
    crc = x25crc()
    l = ord(ser.read(1))
    seq = ord(ser.read(1))
    sid = ord(ser.read(1))
    if sid != 1:
        return
    cid = ord(ser.read(1))
    if cid != 0xFA:
        return
       
    mid = ord(ser.read(1))
    #print 'S C MID = {}|{}|{}'.format(sid, cid, mid)
    
    if( mid == 35 ):
        pl = ser.read(l)
        print 'MID = 35 found, l = {}'.format(l)
        parse_servo(pl)

def run():
    fgsrv = fg.fgsrv()
    fgcli = fg.fgcli()
    
    fgsrv.start()
    fgcli.start()
    
    while True:
        fgsrv.a = -fgcli.r * 0.5
        fgsrv.e = fgcli.p * 0.5
        #fgsrv.a = -fgcli.r * 0.01
        time.sleep(0.01)

    init_ports()
    
    while not keyboard.is_pressed('q'):
        inw = ser.inWaiting()
        if inw <= 0:
            time.sleep(0.1)
        
        x = 'a'        
        while( x != chr(0xFE) ):
            x = ser.read(1)
            parse_mavlink()
            
    ser.close()
            
        
        
run()
       
        
