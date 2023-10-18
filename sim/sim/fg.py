import socket
import time
import threading

UDP_IP = "127.0.0.1"
    
class fgsrv ( threading.Thread ):
    def __init__(self):
        threading.Thread.__init__(self)
        self.a = 0
        self.e = 0
        self.t = 0
        
    def run(self):
        UDP_PORT = 5006
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
        while True:
            msg = '{:02.4f},{:02.4f},{:02.4f}\n'.format(self.a, self.e, self.t)
            sock.sendto(msg, (UDP_IP, UDP_PORT))    
            time.sleep(0.01)        
            #print msg
            
class fgcli ( threading.Thread ):
    def __init__(self):
        threading.Thread.__init__(self)
        self.la = 0
        self.lo = 0
        self.h = 0
        self.spd = 0
        self.r = 0
        self.p = 0
        self.y = 0
        self.params = []
        
    def run(self):
        UDP_PORT = 5005
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
        sock.bind((UDP_IP, UDP_PORT))
    
        while True:
            data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
            #print "received message:", data  
            data_s = data.split(",")            
            self.params = list(map(float, data_s))
            #print self.params
            self.la, self.lo, self.h, self.spd, self.p, self.r, self.y = self.params

        
