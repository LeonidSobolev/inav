import phys
import time

import fgemul

p = phys.XPlane()

p.set_rc((0, 0, 0.2))

# import the pygame module
import pygame

# import random for random numbers!
import random

# import pygame.locals for easier access to key coordinates
from pygame.locals import *


import xdraw

x = -900
y = 400
import os
#os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (x,y)

# initialize pygame
pygame.init()

# create the screen object
# here we pass it a size of 800x600
screen = pygame.display.set_mode((800, 600))
pygame.display.set_caption('XSim Fixar')

# Create a custom event for adding a new enemy.
ADDENEMY = pygame.USEREVENT + 1
pygame.time.set_timer(ADDENEMY, 250)

# create our 'player', right now he's just a rectangle
plane = xdraw.PlanePic()

map_size = (screen.get_size()[0], screen.get_size()[1] - 100 ) 
background = pygame.Surface(map_size)
background.fill((135, 206, 250))

h_size = (screen.get_size()[0], 100 ) 
background_h = pygame.Surface(h_size)
hgraph = pygame.Surface(h_size)
#background_h.fill((155, 186, 250))
background_h.fill((255, 244, 250))

all_sprites = pygame.sprite.Group()
all_sprites.add(plane)

running = True

#EMUL_BRIDGE
try:
    eb = fgemul.get_bridge()
except:
    print( 'CAN NOT FIND AP or pors settings wrong.')

myfont = pygame.font.SysFont('Courier new', 15)

curtime = 0

a = 0
e = 0

while running:
    for event in pygame.event.get():
        if event.type == KEYDOWN:
            if event.key == K_ESCAPE:
                running = False
                p.is_run = False
        elif event.type == QUIT:
            running = False
            p.is_run = False
        elif event.type == ADDENEMY:
            curtime += 1
            screen.blit(background, (0, 0))
            hgraph.blit(background_h, (0, 0)) 
            pressed_keys = pygame.key.get_pressed()
            if pressed_keys[K_LEFT]:
                a -= 0.05
                print( 'a={}'.format(a))
            if pressed_keys[K_RIGHT]:
                a += 0.05
                print( 'a={}'.format(a) )              
            if pressed_keys[K_UP]:
                e += 0.05
                print( 'e={}'.format(e))
            if pressed_keys[K_DOWN]:
                e -= 0.05
                print( 'e={}'.format(e) )  
            if pressed_keys[K_p]:
                p.pause()
            if pressed_keys[K_o]:
                p.resume()   
            if pressed_keys[K_l]:
                p.restart()                   
            for entity in all_sprites:
                screen.blit(entity.image, entity.rect)   

            plane.set_pos( p.pos[1] + 400, -p.pos[0] + 300, p.rpy[2] * 180 /3.1415 )
            xdraw.draw_track(screen, p.pos[1] + 400, -p.pos[0] + 300 )
            xdraw.draw_height(hgraph,  curtime, 100-p.pos[2] )
            screen.blit(hgraph, (0, map_size[1]))
            s = "{:6.2f} {:6.2f} {:6.2f} VEL = {:6.2f}|{:6.2f}".format( p.pos[0], p.pos[1], p.pos[2], p.velx, p.velz )
            #print s 

            try:
                aet = fgemul.update( eb, p.gps_pos, p.get_rpy(), p.velx )
            except:
                aet = (a, e, 0.89)
            p.set_rc( aet )
            
            #print 'AET = {}\t{}\t{}\t'.format( aet[0], aet[1], aet[2] ) 

            
            textsurface = myfont.render(s, False, (0, 0, 0))
            screen.blit(textsurface,(0,0))
            
            forces = 'X:  Fx = {:6.2f}, L = {:6.2f}, FricX = {:6.2f}'.format( p.force_Fx, p.force_Lx, p.force_Ffx)
            textsurface = myfont.render(forces, False, (0, 0, 0))
            screen.blit(textsurface,(0,20))
            
            forces = 'Z: mg = {:6.2f}, F = {:6.2f}, L = {:6.2f}, FricZ = {:6.2f}'.format( p.force_mg, p.force_F, p.force_L, p.force_Ffz)
            textsurface = myfont.render(forces, False, (0, 0, 0))
            screen.blit(textsurface,(0,40))
            
            angs = 'roll = {:6.2f}, pitch = {:6.2f}, yaw = {:6.2f}, ail = {:6.2f}, elev = {:6.2f}, thro = {:6.2f}'.format( p.rpy[0] * 180/3.14, p.rpy[1] * 180/3.14, p.rpy[2] * 180/3.14, p.a, p.e, p.t)
            textsurface = myfont.render(angs, False, (0, 0, 0))
            screen.blit(textsurface,(0,60))            
    
    
            pygame.display.flip()
