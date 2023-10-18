import math
import time
import threading

def sign(x):
    if x >= 0:
        return 1.0
    return -1.0

class XPlane( threading.Thread ):

#HAPO-OIE

    def __init__(self, la = 59.8766, lo = 30.72284, h = 0):
#    def __init__(self, la = 59.66739, lo = 29.55868, h = 0):
#    def __init__(self, la = 55.36739, lo = 37.19868, h = 0):
#    def __init__(self, la = 43.26715, lo = 43.53025, h = 0):
#    def __init__(self, la = 54.97027, lo = 60.99490, h = 0):
        #X axe is plane forward direction, rising forward
        #Y is wing, rising right
        #Z is height, rising up
        self.pos = [0,0,0]
        self.vel = [0,0,0]
        self.velx = 0
        self.velz = 0
        self.w = [0,0,0]
        self.rpy = [0,0,0]
        self.gps_pos = [la, lo, h]
        self.origin = [la,lo,h]
        
        
        self.o_scale =  math.cos( abs(3.1415 / 180 *  self.origin[0]))# * 0.0174532925
        print( 'Origin scale is: {}'.format(self.o_scale))

        # self.pos = [0,55800.0,50]
        # self.set_gps()
        # print 'GPS = {}'.format( self.gps_pos )

        
        #Weight
        self.M = 10
        self.g = 9.8
        #Force of friction in parrots
        self.Ff = [0.1, 0, 5]
        #Plane-copter offset, degrees
        self.poffset = 40.0 * 3.1415 / 180.0
        
        #additional variables
        #linear_accel
        #self.la = [0,0,0]
        
        #RC
        self.set_rc( (0, 0, 0, ) )
        
        #ThrottleForse
        self.F = 0
        
        #Momentums'
        self.A = 2
        self.E = 1
        self.R = 1
        
        self.L = 0#0.5
        
        self.force_mg = 0
        self.force_F = 0
        self.force_L = 0
        self.force_Ffz = 0
        self.force_Fx = 0
        self.force_Ffx = 0
        self.force_Lx = 0     
        
        
        self.is_pause = 0
        threading.Thread.__init__(self)
        self.is_run = True
        self.start()
        
    def run(self):
        while(self.is_run):
            time.sleep(0.01)
            if not self.is_pause:
                self.step(0.03)

            
    def pause(self):
        self.is_pause = 1
    def resume(self):
        self.is_pause = 0
    def restart(self):
        self.pos = [0,0, 0]               
        print("Restart")
            
    def set_rc(self, aet):
        if aet[2] < 0:
            self.t = 0
        else:
            self.t = aet[2]
        self.a = aet[0]
        self.e = aet[1]
        
        self.F = self.t * 200;
      
    def set_gps(self):
        self.gps_pos[2] = self.pos[2]
        DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR = 111319.5
        self.gps_pos[0] = self.origin[0] + self.pos[0] / DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR 
        self.gps_pos[1] = self.origin[1] + self.pos[1] / DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR / self.o_scale
        

        
    def get_rpy(self):
        outrpy = ( self.rpy[0] * 180/3.1415, (self.rpy[1])*180/3.1415, self.rpy[2]*180/3.1415 )
        #outrpy = ( self.rpy[0] * 180/3.1415, (self.rpy[1] - self.poffset)*180/3.1415, self.rpy[2]*180/3.1415 )
        return outrpy
    
    def step(self, dt):
        
        self.force_mg = -self.M * self.g
        #self.force_F = self.F * math.cos( -self.rpy[1] - self.poffset ) * self.cos_r()
        self.force_F = self.F * math.cos( -self.rpy[1] ) * self.cos_r()
        if self.velx < 0 :
            self.force_L =  0
        else:    
            self.force_L = self.L * self.velx * self.velx * math.cos( -self.rpy[1] )
        self.force_Ffz = -self.Ff[2] * self.velz * self.velz * sign(self.velz)
        az = (self.force_mg + self.force_F + self.force_L + self.force_Ffz ) / self.M
        self.velz += dt * az
        dz = dt * self.velz / 2
        np = self.pos[2] + dz #+ self.sin_p() * dr   
        #hit
        if np < 0:
            np = 0
            self.velx = 0
            self.velz = 0   
            self.rpy[0] = 0
            self.rpy[1] = 0
        self.pos[2] = np  
            
        if np < 0.01:
            self.pos[2] = np 
            return
            
           
        
        #ROTATION
        self.rpy[2] += dt * self.velx * self.rpy[0] * self.A * 3.1415 / 180.0 
        self.rpy[1] += dt * self.e * self.E
        self.rpy[0] += dt * self.a * self.R
        
        #X
        #Ma = Fsin(a) - Ff * V
        #self.force_Fx = self.F * math.sin( self.poffset + self.rpy[1] )
        self.force_Fx = self.F * math.sin( self.rpy[1] )
        #print '{:6.2f} | {:6.2f} | sin(p-a) = {:6.2f}, full F = {:6.2f}'.format( self.rpy[1], self.poffset, math.sin( self.rpy[1] - self.poffset ), self.F )
        self.force_Ffx = - self.Ff[0] * self.velx * self.velx * sign(self.velx)
        self.force_Lx = self.L * self.velx * self.velx * math.sin( +self.rpy[1] )
        ax = ( self.force_Fx + self.force_Lx +  self.force_Ffx ) / self.M
        #first integral
        self.velx += dt * ax
        #second integral -- path len
        dr = dt * self.velx / 2
        
        #if( self.pos[2] > 0.1 ):
        self.pos[0] += dr * self.cos_y()
        
        #Y
        self.pos[1] += dr * self.sin_y()
            
            # #Z
            # #Ma = -Mg + Fcos(P)cos(a)cos(R) + L*Vx*Vx - Ff * Vz * Vz
            # self.force_mg = -self.M * self.g
            # self.force_F = self.F * math.cos( self.rpy[1] - self.a ) * self.cos_r()
            # self.force_L = self.L * self.velx * self.velx
            # self.force_Ffz = -self.Ff[2] * self.velz * self.velz * sign(self.velz)
            # az = (self.force_mg + self.force_F + self.force_L + self.force_Ffz ) / self.M
            # self.velz += dt * az
            # dz = dt * self.velz / 2
            # np = self.pos[2] + dz #+ self.sin_p() * dr
            
            # #print '{}\t{}\t{}\t{}\t'.format( az, self.velz, dz, np)
            # if np < 0:
                # np = 0
            # self.pos[2] = np
        
        
        if self.velx < 1 :
            self.pos[2] = np
        else:
            plane_gain = self.velx * self.velx / 400
            if plane_gain > 1:
                plane_gain = 1
            self.pos[2] = np - math.sin(  self.rpy[1]  - self.poffset ) * dr * plane_gain
        
        self.set_gps()
        
        
       

    def cos_y(self):
        return math.cos( self.rpy[2] )
    def sin_y(self):
        return math.sin( self.rpy[2] ) 
    def cos_r(self):
        return math.cos( self.rpy[0] )
    def sin_r(self):
        return math.sin( self.rpy[0] ) 
    def cos_p(self):
        return math.cos( self.rpy[1] )
    def sin_p(self):
        return math.sin( self.rpy[1] )         
        
    
    