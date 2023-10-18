import time
#import keyboard
import serial
import fg
import math


SERIAL_DEVICE = "COM13"
#SERIAL_DEVICE = "COM4"

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
        
        

def pack_coord( la, lo, h, r, p, y, spd ):
    lai = int(la * 10000000)
    loi = int(lo * 10000000)
    h = int(h * 10)
    r = int(r * 10)
    p = int(p * 10)
    y = int((y%360) * 10)
    spd = int(spd * 10)
    
    cmd = [0xAA, 0x01]
    
    for i in range(4):
        cmd.append( (lai >> (i*8)) & 0xFF )
    for i in range(4):
        cmd.append( (loi >> (i*8)) & 0xFF )
    for i in range(2):
        cmd.append( (h >> (i*8)) & 0xFF )
    for i in range(2):
        cmd.append( (r >> (i*8)) & 0xFF )
    for i in range(2):
        cmd.append( (p >> (i*8)) & 0xFF )
    for i in range(2):
        cmd.append( (y >> (i*8)) & 0xFF )
    for i in range(2):
        cmd.append( (spd >> (i*8)) & 0xFF )        
    #print "{}:{} -> {}:{}".format(la, lo, lai, loi)  
    app_crc(cmd)    
    #print cmd
    return cmd    
    
pstate = 0
cmd = []
e = 0
a = 0
t = 0
nav_mode = 0
d1 = 0
d2 = 0
d3 = 0

def ppush( c ):
    global pstate
    global cmd
    global a, e, t
    global nav_mode
    global d1, d2, d3
    
    if pstate == 0:
        if ord(c) == 0xAA:
            pstate += 1
        return
    if pstate == 1:
        if ord(c) == 0x02:
            pstate += 1
        else:
            pstate = 0
        return

    pstate += 1;        
    cmd.append( ord(c) )
    if len(cmd) >= 14+1+13+2:
        pstate = 0
        tmpa = (-1500 + (cmd[1] + ((cmd[2] & 0xFF) << 8)))
        tmpe = (-1500 + (cmd[3] + ((cmd[4] & 0xFF) << 8)))
        a = tmpa/ 500.0
        e = tmpe / 500.0
        t0 = -(1000 - (cmd[5] + ((cmd[6] & 0xFF) << 8))) / 1000.0
        t1 = -(1000 - (cmd[7] + ((cmd[8] & 0xFF) << 8))) / 1000.0
        t2 = -(1000 - (cmd[8] + ((cmd[10] & 0xFF) << 8))) / 1000.0
        t3 = -(1000 - (cmd[11] + ((cmd[12] & 0xFF) << 8))) / 1000.0
        nav_mode = cmd[15]
        t = ( t0 + t1 + t2 + t3 ) / 4
        dbg = (cmd[13] + ((cmd[14] & 0xFF) << 8));
        d1 = (cmd[16] + ((cmd[17] & 0xFF) << 8) + ((cmd[18] & 0xFF) << 16) + ((cmd[19] & 0xFF) << 24))
        d2 = (cmd[20] + ((cmd[21] & 0xFF) << 8) + ((cmd[22] & 0xFF) << 16) + ((cmd[23] & 0xFF) << 24))
        d3 = (cmd[24] + ((cmd[25] & 0xFF) << 8) + ((cmd[27] & 0xFF) << 16) + ((cmd[27] & 0xFF) << 24))                
        
        #print 'A={}\t E = {}, \tRAW = {}\t{}'.format(a, e, tmpa, tmpe)
        #print cmd
        #print ''
        
        cmd = []

        #print "A={:04.1f}, E={:04.1f}, T={:04.1f}, D = {}, M = {}, D = {:04.1f}\t{:04.1f}\t{:04.1f}".format( a, e, t, dbg, nav_mode, d1, d2, d3)
        
    
def check_input(ser): 
    while ser.inWaiting() > 0:
        ppush( ser.read(1) ) 
  

def get_bridge():
    return init_port()
    
def update(ser, gpos, rpy, speed):  
    global a,e,t
    ser.write( pack_coord( gpos[0], gpos[1], gpos[2]*10, rpy[0], rpy[1], rpy[2], speed * 10 ) )
    check_input(ser)
    return (a, e, t)   
            
# def run_fg():
    # global a,e,t
    # ser = init_port()
    # fgsrv = fg.fgsrv()
    # fgcli = fg.fgcli()
    
    # fgsrv.start()
    # fgcli.start()
    
    # print 'port ok'
    # if ser is not None:
        # print 'port ready'
        # while not keyboard.is_pressed('q'):
            # ser.write( pack_coord( fgcli.la, fgcli.lo, fgcli.h, fgcli.r, -fgcli.p, fgcli.y, fgcli.spd ) )
            # #ser.write( pack_coord( 0,0,0,0,0,0,0 ) )
            # check_input(ser)
            # time.sleep(0.001)
            
            # fgsrv.a = -int(a)
            # fgsrv.e = -int(e)
            # fgsrv.t = 50
            # #print '.'
        
    # print 'Good bye!'    
    
# def run_airsim():
    # import airsim
    # # connect to the AirSim simulator
    # print 'Test starting...'
    # client = airsim.MultirotorClient()
    # print 'Client created.'
    # client.confirmConnection()
    # print 'Connected.'
    # client.enableApiControl(True)
    # client.armDisarm(True)    
    

    
    # global a,e,t
    # ser = init_port()
    
    # client.moveByManualAsync(vx_max = 1E6, vy_max = 1E6, z_min = -1E6, duration = 1E10)

    # print 'port ok'
    # i = 0

    # if ser is not None:
        # print 'port ready'
        # while not keyboard.is_pressed('q'):
            # state = client.getMultirotorState() 
            # la = state.gps_location.latitude
            # lo = state.gps_location.longitude
            # h = state.gps_location.altitude
            # ori = state.kinematics_estimated.orientation
            # spd_v = state.kinematics_estimated.linear_velocity
            # spd = math.sqrt( spd_v.x_val * spd_v.x_val + spd_v.y_val * spd_v.y_val )
            # ser.write( pack_coord( la, lo, h, ori.x_val, ori.y_val, ori.z_val, spd ) )
            # #ser.write( pack_coord( 0,0,0,0,0,0,0 ) )
            # check_input(ser)
            # time.sleep(0.001)
            
            # thro = (t+1)/2
            # if thro < 0:
                # thro = 0
            
            # rcdata = airsim.RCData(yaw = a, throttle = thro, pitch = e, is_initialized = True, is_valid = True)
            
            # client.moveByRC(rcdata)
            
            # i += 1
            # if i > 100:
                # #print '{}, {}, {}, spd = {}, ori = {}/{}/{}'.format(la, lo, h, spd, ori.x_val, ori.y_val, ori.z_val)
                # #print rcdata
                # print thro
                # i = 0
                # #client.moveByManualAsync(vx_max = 1E6, vy_max = 1E6, z_min = -1E6, duration = 1E10)
        
    # print 'Good bye!'
