import pygame
from kinematic import *
from pygame.locals import *

from OpenGL.GL import *
from OpenGL.GLU import *

#############################################################
###############  PRIMERA ENTREGA DE CI6450  #################
############ LARRY PEREZ ########## 10-10547 ################
#############################################################


#Constantes y variables para la parte grafica

LIM = 10
MOV = 0.01

vertexx = []
vertexy = []
gedges = []


for x in range(-LIM,LIM+1):
    vertexx.append((x,LIM,0))
    vertexx.append((x,-LIM,0))
    vertexy.append((LIM,x,0))
    vertexy.append((-LIM,x,0))

for g in range(0,(4*LIM)+1,2):
    gedges.append((g,g+1))

#funciones graficas con OpenGL:

def grid(gedges,vertexx,vertexy):

    glBegin(GL_LINES)
    glColor3fv((1,1,0)) 
    glVertex3fv((-LIM,LIM,-0.25))
    glVertex3fv((LIM,LIM,-0.25))
    glVertex3fv((LIM,LIM,-0.25))
    glVertex3fv((LIM,-LIM,-0.25))
    glVertex3fv((LIM,-LIM,-0.25))
    glVertex3fv((-LIM,-LIM,-0.25))
    glVertex3fv((-LIM,-LIM,-0.25))
    glVertex3fv((-LIM,LIM,-0.25))
    glEnd()

    glBegin(GL_LINES)
    glColor3fv((1,1,0))    
    glVertex3fv((-LIM,LIM,-0.5))
    glVertex3fv((LIM,LIM,-0.5))
    glVertex3fv((LIM,LIM,-0.5))
    glVertex3fv((LIM,-LIM,-0.5))
    glVertex3fv((LIM,-LIM,-0.5))
    glVertex3fv((-LIM,-LIM,-0.5))
    glVertex3fv((-LIM,-LIM,-0.5))
    glVertex3fv((-LIM,LIM,-0.5))
    glEnd()

    glBegin(GL_QUADS)
    glColor3fv((0,0,1))
    glVertex3fv((-LIM,LIM,0))
    glVertex3fv((LIM,LIM,0))
    glVertex3fv((LIM,-LIM,0))
    glVertex3fv((-LIM,-LIM,0))
    glEnd()
    
    glBegin(GL_LINES)
    glColor3fv((0,0,0))
    for edge in gedges:
        for vertex in edge:
            glVertex3fv(vertexx[vertex])

        for vertex in edge:
            glVertex3fv(vertexy[vertex])
    glEnd()

def gridat(x,y,z,gedges,vertexx,vertexy):
    glPushMatrix();
    glTranslatef(x,y,z)
    grid(gedges,vertexx,vertexy)
    glTranslatef(-x,-y,-z)
    glPopMatrix();

#Para determinar caidas de las plataformas

stageBoundsUp = [LIM,LIM]
stageBoundsDown = [-LIM,-LIM]

def outofgrid(supLimits,infLimits,pc,gridUbicX,gridUbicY):
    ubics = [gridUbicX,gridUbicY]
    violatedbounds = 0
    for i in range(0,2):
        if pc.p[i] >= ubics[i]+supLimits[i]+pc.body:
            return True
        elif pc.p[i]<= ubics[i]+infLimits[i]-pc.body:
            return True
    else: return False
    
def fallfromgrids(pc,grids,out_stage,sj,gridUbicX,gridUbicY):
    gravityornot = True
    for grid in grids:
        if sj and pc.isGrounded(grid[2]):
            pc.simpleJump(0.5)
            sj = False
        else:
            if pc.isGrounded(grid[2]) and not outofgrid(gridUbicX,gridUbicY ,pc,grid[0],grid[1]):
                if not pc.isGrounded(grid[2]-0.5) and pc.ms[2]<0:
                        pc.p[2]=grid[2]
                        pc.ms[2]=0
                gravityornot = gravityornot and False
            elif pc.isGrounded(grid[2]-20*LIM):
                pc.p[2]= grid[2]-20*LIM
                pc.ms[2]=0
            else:
                gravityornot = gravityornot and True
        if gravityornot:
            pc.gravity()
    return sj
        
    
def main(pc,npc1,npc2,npc3,npc4):
    pygame.init()
    display = (800,600)
    pygame.display.set_mode(display, DOUBLEBUF|OPENGL)

    gluPerspective(45, (display[0]/display[1]), 0.1, 100.0)

    glTranslatef(0.0,0.0, -30)
    glRotatef(-60, 1, 0, 0)

    #ubicacion de las plataformas
    plattforms = (
        (2*LIM+1,2*LIM+1,-3),
        (2*LIM+1,0,-2),
        (0,0,0),
        (-2*LIM-2,0,1)
        )
    out_stage = False
    
    mdown = False
    mup = False
    mleft = False
    mright = False
    rleft = False
    rright = False
    startjump = False
    recam = 0
    cam = 0

    #manejador de colisiones (bordes de una de las plataformas)
    oob = SqBoundariesCollision([-LIM-2,LIM,LIM],[-3*LIM-2,-LIM,-LIM])


    #detectar los eventos
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()

            if event.type == KEYDOWN:
                if event.key == K_DOWN: 
                    mdown = True
                if event.key == K_LEFT:
                    mleft = True
                if event.key == K_UP: 
                    mup = True
                if event.key == K_RIGHT:
                    mright = True
                if event.key == K_a:
                    rleft = True
                if event.key == K_d:
                    rright = True
                if event.key == K_SPACE:
                    startjump = True
                if event.key == K_q:
                    if cam == 0:
                        cam = 2
                if event.key == K_e:
                    if cam == 1:
                        cam = 3                

            if event.type == KEYUP:
                if event.key == K_DOWN:
                    mdown = False
                if event.key == K_LEFT:
                    mleft = False
                if event.key == K_UP: 
                    mup = False
                if event.key == K_RIGHT:
                    mright = False
                if event.key == K_a:
                    rleft = False
                if event.key == K_d:
                    rright = False                    

        #movimientos del personaje (no IA)

        if mdown:
            pc.changemov([0,-MOV,0])

            
        if mleft:
            pc.changemov([-MOV,0,0])


        if mright:
            pc.changemov([MOV,0,0])


        if mup:
            pc.changemov([0,MOV,0])

            
        if rleft:
            pc.changerot(-MOV*5)

            
        if rright:
            pc.changerot(MOV*5)





            
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)


        out_stage = True
        grounded = False

        #cambios de perspectiva, nada importante                
        if pc.p[0] > LIM and recam == 0:
            if cam == 1:
                glRotatef(60, 1, 0, 0)
                glTranslatef(-2*LIM,0.0, 0.0)
                glRotatef(-60, 1, 0, 0)
            else:
                glTranslatef(-2*LIM,0.0, 0.0)
            recam = 1
        if pc.p[0] < LIM and recam == 1:
            if cam == 1:
                glRotatef(60, 1, 0, 0)
                glTranslatef(2*LIM,0.0, 0.0)
                glRotatef(-60, 1, 0, 0)
            else:
                glTranslatef(2*LIM,0.0, 0.0)
            recam = 0
        if pc.p[1] > LIM and recam == 1:
            if cam == 1:
                glRotatef(60, 1, 0, 0)
                glTranslatef(0.0,-2*LIM,0.0)
                glRotatef(-60, 1, 0, 0)
            else:
                glTranslatef(0.0,-2*LIM, 0.0)
            recam = 2
        if pc.p[1] < LIM and recam == 2:
            if cam == 1:
                glRotatef(60, 1, 0, 0)
                glTranslatef(0.0,2*LIM,0.0)
                glRotatef(-60, 1, 0, 0)
            else:
                glTranslatef(0.0,2*LIM, 0.0)
            recam = 1
        if pc.p[0] < -LIM and recam == 0:
            if cam == 1:
                glRotatef(60, 1, 0, 0)
                glTranslatef(2*LIM,0.0, 0.0)
                glRotatef(-60, 1, 0, 0)
            else:
                glTranslatef(2*LIM,0.0, 0.0)
            recam = -1
        if pc.p[0] > -LIM and recam == -1:
            if cam == 1:
                glRotatef(60, 1, 0, 0)
                glTranslatef(-2*LIM,0.0, 0.0)
                glRotatef(-60, 1, 0, 0)
            else:
                glTranslatef(-2*LIM,0.0, 0.0)
            recam = 0
        if cam == 2:
            glRotatef(60, 1, 0, 0)
            cam = 1
        if cam == 3:
            glRotatef(-60, 1, 0, 0)
            cam = 0

        #chequeos de las plataformas y caidas            
        for plat in plattforms:
            gridat(plat[0],plat[1],plat[2],gedges,vertexx,vertexy)
            out_stage = out_stage and outofgrid(stageBoundsUp ,stageBoundsDown ,pc,plat[0],plat[1])

        startjump = fallfromgrids(pc,plattforms,out_stage,startjump,stageBoundsUp ,stageBoundsDown)

        for npc in [npc1,npc2,npc3,npc4]:
            for plat in plattforms:
                out_stage = out_stage and outofgrid(stageBoundsUp ,stageBoundsDown ,npc,plat[0],plat[1])
            xxx = fallfromgrids(npc,plattforms,out_stage,startjump,stageBoundsUp ,stageBoundsDown)

        #comportamientos implementados en esta prueba, como ejemplo.
        #hay algunos mas derivados de estos en kinematic.py

        #persecusion
        #huida
        #wander
        #evitar obstaculos
        #mexcla de comportamientos
        #alineamiento
            
        behavior1 = npc1.pursue(pc,5)
        behavior2 = npc2.flee(pc,5)
        behavior3 = npc3.simpleWander()
        behavior4 = npc3.obstacleAvoidance(oob,3,2)
        behavior5 = npc3.blending([behavior3,behavior4],[1,1000000])
        behavior6 = npc4.align(pc)


        #ordena a los npc a que se comporten
        npc1.behave(behavior1)
        npc2.behave(behavior2)
        npc3.behave(behavior5)
        npc4.behave(behavior6)

        #funciones de friccion
        pc.friction(MOV)
        npc1.friction(MOV)
        npc2.friction(MOV)
        npc3.friction(MOV)
        npc4.friction(MOV)

        #actualizar los estados de los cuerpos/personajes
        pc.act()

        npc1.act()
        npc2.act()
        npc3.act()
        npc4.act()

        pc.o = pc.orientationSteering()

        #dibujarlos
        
        if pc.p[2] > -10*LIM:
            pc.cubeman()

        if npc1.p[2] > -10*LIM:
            npc1.drawnpc()
        if npc2.p[2] > -10*LIM:
            npc2.drawnpc()
        if npc3.p[2] > -10*LIM:
            npc3.drawnpc()
        if npc4.p[2] > -10*LIM:
            npc4.drawnpc()
        
        
        
        pygame.display.flip()
        pygame.time.wait(2)

#creacion de los objetos correspondientes, e inicio del main
player = Kinematic([0,0,0],[0,0,0])
pursuer = Kinematic([5,5,0],[2,2,0],60,10,0.04,10)
coward = Kinematic([2*LIM+1,0,-2],[0.01,0.01,0],60,10,0.04,10)
wanderer = Kinematic([-2*LIM-2,0,1],[0,0,0],0,0,0.08,8)
aligner = Kinematic([2*LIM+1,2*LIM+1,-3],[0.15,0,0],0,0,0.05,1,0.05,1)
main(player,pursuer,coward,wanderer,aligner)
