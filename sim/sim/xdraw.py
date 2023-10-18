
import pygame
from pygame.locals import *

track = []
t_offser = 0
track2 = []
t_offset2 = 0

def rot_center(image, angle):
    """rotate an image while keeping its center and size"""
    orig_rect = image.get_rect()
    rot_image = pygame.transform.rotate(image, angle)
    rot_rect = orig_rect.copy()
    rot_rect.center = rot_image.get_rect().center
    rot_image = rot_image.subsurface(rot_rect).copy()
    return rot_image

class PlanePic(pygame.sprite.Sprite):
    def __init__(self):
        super(PlanePic, self).__init__()
        self.image_orig = pygame.image.load('pl.png').convert_alpha()
        self.image = self.image_orig
        #self.image.set_colorkey((255, 255, 255), RLEACCEL)
        self.rect = self.image.get_rect()
        
    def set_pos(self, x, y, az):
        self.image = rot_center( self.image_orig, -az )
        self.rect = self.image.get_rect()
        self.rect.move_ip(x - 32, y - 32)
      

    
        
def init_screen():
    global planepic
 
    # Define some colors
    BLACK = (0, 0, 0)
    WHITE = (255, 255, 255)
    GREEN = (0, 255, 0)
    RED = (255, 0, 0)
     
    pygame.init()
    

    
    # Set the width and height of the screen [width, height]
    size = (600, 400)
    screen = pygame.display.set_mode(size)
     
    pygame.display.set_caption("XSim Test")
    clock = pygame.time.Clock()
    
    pygame.font.init() # you have to call this at the start, 
                   # if you want to use this module.


    screen.fill(WHITE)
    planepic =  PlanePic()     
    pygame.display.flip()
    
    return screen
    
def draw_pixel(x,y,s):
    pygame.draw.circle(s, (255, 0, 0), [x, y], 1)
    
def draw_track(screen, x, y):
    global track
    
    for event in pygame.event.get():
        if event.type == KEYDOWN:
            if event.key == K_ESCAPE:
                running = False
        elif event.type == QUIT:
            running = False
    

    
    x = int(x)
    y = int(y)
    
    track.append( [x,y] )
    
    if len(track) > 500:
        track.pop(0)
       
    offset_x = 0       
    offset_y = 0       
    if( x > 400):
        offset_x = x - 400 
    if( y > 400):
        offset_y = y - 400 
                    
    # screen.fill((255, 255, 255))
    for p in track:
        draw_pixel(p[0] - offset_x, p[1] - offset_y, screen)
        
    # pygame.draw.line(screen, (255,0,0), (300,200-5), (300,200+5), 1)
    # pygame.draw.line(screen, (255,0,0), (300-5,200), (300+5,200), 1)
    
def draw_height(screen, x, y):
    global track2
    
    for event in pygame.event.get():
        if event.type == KEYDOWN:
            if event.key == K_ESCAPE:
                running = False
        elif event.type == QUIT:
            running = False
    
    x = int(x)
    y = int(y)
    
    offset = 0
    if( x > 500):
        offset = x - 500    
    
    track2.append( [x,y] )
    if len(track2) > 500:
        track2.pop(0)    
    
    # screen.fill((255, 255, 255))
    for p in track2:
        draw_pixel(p[0]-offset, p[1], screen)
        
    # pygame.draw.line(screen, (255,0,0), (300,200-5), (300,200+5), 1)
    # pygame.draw.line(screen, (255,0,0), (300-5,200), (300+5,200), 1)    
    

