import time
#import keyboard
import serial

SERIAL_DEVICE = "COM15"



class XLCmd:
    def __init__(self):
        self.len = 0
        self.data = []
        
    def push(self, b):
        print("{:02x}".format(b), '')
        self.data.append(b)
        if len(self.data) >= self.len * 16:
            return True
        return False
        
    
    
class XLCmdParser:
    def __init__(self):
        self.data = []
        self.cur_cmd = XLCmd()
        self.pstate = 0
        
    def push(self, cmd):
        self.data.append(cmd)
        print("CMD = {:02x}".format(self.pop(0).data[0]))
        
    def push_char(self, c): 
        print(c)
        if self.pstate == 0:
            if ord(c) == 0x67:
                self.pstate += 1
            return
        elif self.pstate == 1:
            if ord(c) == 0x41:
                self.pstate += 1
            else:
                self.pstate = 0
            return
        elif self.pstate == 2:
            self.pstate += 1
        elif self.pstate == 3:
            #have no messages with >5 block count
            if( ord(c) < 6 ): 
                self.cur_cmd.len = ord(c)
        else:
            if self.cur_cmd.push( ord(c) ):
                self.push(cur_cmd)
                self.cur_cmd = XLCmd()
                self.pstate = 0    

parser = XLCmdParser()




def init_port():
    try:
        print( 'open ports')
        ser    = serial.Serial(SERIAL_DEVICE)
        br = 115200
        ser.baudrate = br
        ser.flushInput()
        return ser
    except:
        print( 'Fail to open port')
        return None
        
def app_crc( l ):
    ck1 = 0
    ck2 = 0
    for i in range(2, len(l)):        
        ck1 = (ck1 + l[i]) & 0xFF
        ck2 = (ck2 + ck1 + l[i]) & 0xFF

    l.append(ck1)
    l.append(ck2)    
     


        
    
def check_input(ser): 
    global parser
    while ser.inWaiting() > 0:
        c = ser.read(1) 
        parser.push_char( c ) 
  

def get_bridge():
    return init_port()
    
def update(ser):  
    print('.')
    # ser.write( pack_coord( gpos[0], gpos[1], gpos[2]*10, rpy[0], rpy[1], rpy[2], speed * 10 ) )
    check_input(ser)
    
    
    
print("Starting XLogger client")
ser = init_port()
if(ser):
    while(True):    
        update(ser)
        time.sleep(0.1)
    