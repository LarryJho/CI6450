import pygame

#####################
import time
#####################


from pathgraph import *
from statemachine import *
from trianglesmaps import *
from kinematics import *
from pygame.locals import *

from OpenGL.GL import *
from OpenGL.GLU import *

################################################################
######## Larry Perez 10-10547 --- ENTREGA 2 TOPICOS IA #########
################################################################






#constantes del programa

#tamano de los grids
LIM = 20
#magnitud del movimiento
MOV = 0.01

vertexx = []
vertexy = []
gedges = []
stageBoundsUp = [LIM,LIM]
stageBoundsDown = [-LIM,-LIM]

for x in range(-LIM,LIM+1):
    vertexx.append((x,LIM,0))
    vertexx.append((x,-LIM,0))
    vertexy.append((LIM,x,0))
    vertexy.append((-LIM,x,0))

for g in range(0,(4*LIM)+1,2):
    gedges.append((g,g+1))

#Instrucciones apra crear el grid

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
    


#crea un grid en una posicion con sus limites especificos

def gridat(x,y,z,gedges,vertexx,vertexy):
    glPushMatrix();
    glTranslatef(x,y,z)
    grid(gedges,vertexx,vertexy)
    glTranslatef(-x,-y,-z)
    glPopMatrix();

#determina si un personaje esta afuera del grid

def outofgrid(supLimits,infLimits,pc,gridUbicX,gridUbicY):
    ubics = [gridUbicX,gridUbicY]
    for i in range(0,2):
        if pc.p[i] >= ubics[i]+supLimits[i]+pc.body/2:
            return True
        elif pc.p[i]<= ubics[i]+infLimits[i]-pc.body/2:
            return True
    else: return False


    
    
#determina si un personaje esta cayendo de los grids
    
def fallfromgrids(pc,grids,out_stage,sj,gridUbicX,gridUbicY):
    gravityornot = True
    for grid in grids:
        if sj and pc.isGrounded(grid[2]):
            pc.simpleJump(0.3)
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

def fallfromgridsnpc(pc,grids,out_stage,gridUbicX,gridUbicY):
    gravityornot = True
    for grid in grids:
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

def closest_node(node, nodes):
    nodes = numpy.asarray(nodes)
    dist_2 = numpy.sum((nodes - node)**2, axis=1)
    return numpy.argmin(dist_2)

def navmeshes(graph,boundsvertex, items):
    critical_p = []
    for vert in boundsvertex:
        critical_p.append(vert)
    for item in items:
        for vert in item.verticies:
            if vert[2] == 0:
                critical_p.append(vert)

        
    for node1 in graph.nodes:
        for node2 in graph.nodes:
            if node1.share2(node2):
                x = []
                for i in range (0,3):
                    x.append(node2.location[i]-node1.location[i])
                
                conn = connection(node1,node2,numpy.linalg.norm(x))
                graph.addconnection(conn)

           
    return graph   


    






        
########################################################################################        
    
def main(pc,itemlist,npcs):
    pygame.init()
    display = (800,600)
    pygame.display.set_mode(display, DOUBLEBUF|OPENGL)

    gluPerspective(45, (display[0]/display[1]), 0.1, 100.0)
    glLineWidth(1)
    glTranslatef(0.0,0.0, -2.7*LIM)
    glRotatef(-60, 1, 0, 0)


    #Para esta entrega utilizaremos una sola plataforma
    plattforms = (

        (0,0,0),

        )
    
    out_stage = False

    mdown = False
    mup = False
    mleft = False
    mright = False
    rleft = False
    rright = False
    startjump = False
    sj_npcs = []
    seegraph = True
    recam = 0
    cam = 2
    oob = SqBoundariesCollision((-LIM-2,LIM,LIM),(-3*LIM-2,-LIM,-LIM))
    for i in range(0,len(npcs)):
        sj_npcs.append(False)

    

    bounds = []

    ##### Vertices para los triangulos del grafo #####

    gamegraph = pathgraph([],{})
    bounds.append((0,LIM,0))#0
    bounds.append((0,-LIM,0))#1
    bounds.append((LIM,0,0))#2
    bounds.append((-LIM,0,0))#3
    bounds.append((LIM,LIM,0))#4
    bounds.append((-LIM,-LIM,0))#5
    bounds.append((-LIM,LIM,0))#6
    bounds.append((LIM,-LIM,0))#7
    bounds.append((3,3,0))#8
    bounds.append((-3,-3,0))#9
    bounds.append((-3,3,0))#10
    bounds.append((3,-3,0))#11

    #####Primer Cuadrante#####
    

    #obj de arriba
    bounds.append((5,9,0))#12
    bounds.append((7,9,0))#13
    bounds.append((5,15,0))#14
    bounds.append((7,15,0))#15

    #obj de la derecha
    bounds.append((9,7,0))#16
    bounds.append((9,5,0))#17
    bounds.append((15,7,0))#18
    bounds.append((15,5,0))#19

    #Bordes

    bounds.append((5,LIM,0))#20
    bounds.append((7,LIM,0))#21

    bounds.append((LIM,7,0))#22
    bounds.append((LIM,5,0))#23


    #####Segundo Cuadrante#####

    #obj de abajo
    bounds.append((5,-9,0))#24
    bounds.append((7,-9,0))#25
    bounds.append((5,-15,0))#26
    bounds.append((7,-15,0))#27

    #obj de la derecha
    bounds.append((9,-5,0))#28
    bounds.append((9,-7,0))#29
    bounds.append((15,-7,0))#30
    bounds.append((15,-5,0))#31

    #Bordes

    bounds.append((5,-LIM,0))#32
    bounds.append((7,-LIM,0))#33

    bounds.append((LIM,-7,0))#34
    bounds.append((LIM,-5,0))#35


    #####Tercer Cuadrante#####

    #obj de abajo
    bounds.append((-5,-9,0))#36
    bounds.append((-7,-9,0))#37
    bounds.append((-5,-15,0))#38
    bounds.append((-7,-15,0))#39

    #obj de la izquierda
    bounds.append((-9,-5,0))#40
    bounds.append((-9,-7,0))#41
    bounds.append((-15,-7,0))#42
    bounds.append((-15,-5,0))#43

    #Bordes

    bounds.append((-5,-LIM,0))#44
    bounds.append((-7,-LIM,0))#45

    bounds.append((-LIM,-7,0))#46
    bounds.append((-LIM,-5,0))#47


    #####Cuarto Cuadrante#####

    #obj de arriba
    bounds.append((-5,9,0))#48
    bounds.append((-7,9,0))#49
    bounds.append((-5,15,0))#50
    bounds.append((-7,15,0))#51

    #obj de la izquierda
    bounds.append((-9,5,0))#52
    bounds.append((-9,7,0))#53
    bounds.append((-15,7,0))#54
    bounds.append((-15,5,0))#55

    #Bordes

    bounds.append((-5,LIM,0))#56
    bounds.append((-7,LIM,0))#57

    bounds.append((-LIM,7,0))#58
    bounds.append((-LIM,5,0))#59


    #Nodos Intercuadrante
    p1 = bounds[0]
    p2 = bounds[8]
    p3 = bounds[10]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[1]
    p2 = bounds[9]
    p3 = bounds[11]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[2]
    p2 = bounds[8]
    p3 = bounds[11]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[3]
    p2 = bounds[9]
    p3 = bounds[10]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    #mapeo de  puntos perimetro:
    # en el sentido de las agujas del reloj:
    #0,4,2,7,1,5,3,6 --> 8,11,9,10

    #################################
    #Triangulos del 1C:
    p1 = bounds[12]
    p2 = bounds[0]
    p3 = bounds[8]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[12]
    p2 = bounds[14]
    p3 = bounds[0]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[14]
    p2 = bounds[0]
    p3 = bounds[20]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[14]
    p2 = bounds[20]
    p3 = bounds[15]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[20]
    p2 = bounds[21]
    p3 = bounds[15]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[21]
    p2 = bounds[15]
    p3 = bounds[4]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[15]
    p2 = bounds[4]
    p3 = bounds[13]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[13]
    p2 = bounds[4]
    p3 = bounds[16]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[16]
    p2 = bounds[4]
    p3 = bounds[18]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[18]
    p2 = bounds[4]
    p3 = bounds[22]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[18]
    p2 = bounds[22]
    p3 = bounds[23]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[18]
    p2 = bounds[19]
    p3 = bounds[23]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[19]
    p2 = bounds[23]
    p3 = bounds[2]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[19]
    p2 = bounds[17]
    p3 = bounds[2]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[17]
    p2 = bounds[2]
    p3 = bounds[8]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[16]
    p2 = bounds[17]
    p3 = bounds[8]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[16]
    p2 = bounds[13]
    p3 = bounds[8]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)


    p1 = bounds[8]
    p2 = bounds[12]
    p3 = bounds[13]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    ###################################
    #Triangulos del 2C:
    
    p1 = bounds[11]
    p2 = bounds[28]
    p3 = bounds[2]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[31]
    p2 = bounds[28]
    p3 = bounds[2]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[31]
    p2 = bounds[35]
    p3 = bounds[2]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[30]
    p2 = bounds[31]
    p3 = bounds[35]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[34]
    p2 = bounds[30]
    p3 = bounds[35]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)


    
    p1 = bounds[34]
    p2 = bounds[7]
    p3 = bounds[30]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)



    p1 = bounds[29]
    p2 = bounds[30]
    p3 = bounds[7]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[25]
    p2 = bounds[7]
    p3 = bounds[29]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)
    
    p1 = bounds[25]
    p2 = bounds[7]
    p3 = bounds[27]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[27]
    p2 = bounds[7]
    p3 = bounds[33]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[32]
    p2 = bounds[33]
    p3 = bounds[27]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[26]
    p2 = bounds[27]
    p3 = bounds[32]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[1]
    p2 = bounds[26]
    p3 = bounds[32]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[1]
    p2 = bounds[26]
    p3 = bounds[24]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[1]
    p2 = bounds[24]
    p3 = bounds[11]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[11]
    p2 = bounds[24]
    p3 = bounds[25]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[11]
    p2 = bounds[29]
    p3 = bounds[25]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[11]
    p2 = bounds[29]
    p3 = bounds[28]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    ################################
    #Triangulos del 3C:
    
    p1 = bounds[1]
    p2 = bounds[9]
    p3 = bounds[36]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[1]
    p2 = bounds[36]
    p3 = bounds[38]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[1]
    p2 = bounds[44]
    p3 = bounds[38]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[44]
    p2 = bounds[39]
    p3 = bounds[38]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[44]
    p2 = bounds[45]
    p3 = bounds[39]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[39]
    p2 = bounds[45]
    p3 = bounds[5]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[39]
    p2 = bounds[37]
    p3 = bounds[5]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[41]
    p2 = bounds[37]
    p3 = bounds[5]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[41]
    p2 = bounds[42]
    p3 = bounds[5]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[46]
    p2 = bounds[42]
    p3 = bounds[5]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[47]
    p2 = bounds[42]
    p3 = bounds[46]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[43]
    p2 = bounds[42]
    p3 = bounds[47]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[43]
    p2 = bounds[47]
    p3 = bounds[3]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[43]
    p2 = bounds[40]
    p3 = bounds[3]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[3]
    p2 = bounds[40]
    p3 = bounds[9]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[40]
    p2 = bounds[41]
    p3 = bounds[9]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[41]
    p2 = bounds[37]
    p3 = bounds[9]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[37]
    p2 = bounds[36]
    p3 = bounds[9]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    ###########################
    #Triangulos del 4C:
    
    p1 = bounds[3]
    p2 = bounds[52]
    p3 = bounds[10]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[55]
    p2 = bounds[52]
    p3 = bounds[3]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[59]
    p2 = bounds[55]
    p3 = bounds[3]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[59]
    p2 = bounds[54]
    p3 = bounds[55]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[59]
    p2 = bounds[58]
    p3 = bounds[54]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[58]
    p2 = bounds[54]
    p3 = bounds[6]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[6]
    p2 = bounds[53]
    p3 = bounds[54]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[49]
    p2 = bounds[53]
    p3 = bounds[6]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[49]
    p2 = bounds[51]
    p3 = bounds[6]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[51]
    p2 = bounds[57]
    p3 = bounds[6]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[51]
    p2 = bounds[57]
    p3 = bounds[56]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[51]
    p2 = bounds[56]
    p3 = bounds[50]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[56]
    p2 = bounds[50]
    p3 = bounds[0]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[48]
    p2 = bounds[50]
    p3 = bounds[0]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[10]
    p2 = bounds[48]
    p3 = bounds[0]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[48]
    p2 = bounds[49]
    p3 = bounds[10]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[49]
    p2 = bounds[53]
    p3 = bounds[10]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    p1 = bounds[52]
    p2 = bounds[53]
    p3 = bounds[10]
    t = triangle(p1,p2,p3)
    aux = node(t.baricentro(),t)
    gamegraph.addnode(aux)

    ###############################################################################

    
    gamegraph = navmeshes(gamegraph,bounds, itemlist)


    ############# Maquinas de Estado ############

    maquinas = []

    initial = [None,None]
    seeker = [None,None]
    jumper = [None,None]

    def dist(point1,point2):
        data = []
        for i in range (0,3):
            data.append(point2[i] - point1[i])
        return numpy.linalg.norm(data)
        
    chnode = False #Variable para indicar si el player cambio de nodo recientemente
    pi = []


    
    def ini(dat_npc):
        if dat_npc.nodeat == None:
            dat_npc.nodeat = gamegraph.inwhatnode2d(dat_npc.p)
            return dat_npc.seekRandNode(gamegraph)
        if (dat_npc.nodeat != None)  and (gamegraph.inwhatnode2d(dat_npc.p) != None):
            if dat_npc.nodeat.name != gamegraph.inwhatnode2d(dat_npc.p).name:
                dat_npc.nodeat = gamegraph.inwhatnode2d(dat_npc.p)
                return dat_npc.seekRandNode(gamegraph)
        data = SteeringOutput(list([0,0,0]), 0)
        for i in range (0,3):
            data.linear[i] = dat_npc.ms[i]
        data.normalize
        for i in range (0,3):
            data.linear[i] *= 1.02
        return data 

    def sik(dat_npc):
        if (pc.nodeat != None) and (gamegraph.inwhatnode2d(pc.p) != None):
            if pc.nodeat.name != gamegraph.inwhatnode2d(pc.p).name:
                pc.nodeat = gamegraph.inwhatnode2d(pc.p)
                dat_npc.path = dat_npc.seekInGraph(gamegraph,pc.p)
        if dat_npc.path != None:
            if len(dat_npc.path) > 2:
                if gamegraph.inwhatnode2d(dat_npc.p) == gamegraph.inwhatnode2d(dat_npc.path[1].node.location):
                    del dat_npc.path[0]
                return dat_npc.seekPoint2d(dat_npc.path[0].connection.nodeto.location)
            if len(dat_npc.path) == 2:
                if gamegraph.inwhatnode2d(dat_npc.p) == pc.nodeat:
                    del dat_npc.path[0]
                    return dat_npc.seekPoint2d(pc.p)
                return dat_npc.seekPoint(dat_npc.path[0].connection.nodeto.location)
            if len(dat_npc.path) <= 1:
                return dat_npc.seekPoint2d(pc.p)

        return None

    def yom(dat_npc):
        if dat_npc.ms[2] > 0.1:
            return dat_npc.stopjump()
        if dat_npc.isGrounded(0):

            return dat_npc.simpleJump(0.1)
        

        return None

    def halt(dat_npc):
        return dat_npc.stop()



    ############ Triggers del main ##############

    def inrangepc(not_pc):
        x = list([])
        for i in range(0,3):
            x.append(pc.p[i]-not_pc.p[i])
        return (numpy.linalg.norm(x) < 10)

    def pcingraph(not_pc):
        return gamegraph.ingraph(pc.p)



    def tojump(not_pc):
        return not gamegraph.ingraph(pc.p)

    def toinitial(not_pc):
        return (not inrangepc(not_pc)) and pcingraph(not_pc)

    def toseek(not_pc):
        return  inrangepc(not_pc) and pcingraph(not_pc)



    

        
        
    
    for i in range(0,len(npcs)):

        #### Moverse a uno de los nodos cercanos ####

        initial[i] = state(ini,None,halt)

        #### Moverse hacia pc #######################

        
        seeker[i] = state(sik,None,halt)

        #### Saltar #################################

        jumper[i] = state(yom,None,halt)

        ############ Transiciones ###################

        transitions = [transition(tojump,jumper[i],seeker[i]),transition(tojump,jumper[i],initial[i])]
        transitions.append(transition(toinitial,initial[i],seeker[i]))
        transitions.append(transition(toinitial,initial[i],jumper[i]))
        transitions.append(transition(toseek,seeker[i],initial[i]))
        transitions.append(transition(toseek,seeker[i],jumper[i]))
                    
        
        q = statemachine(list([initial[i],seeker[i],jumper[i]]),initial[i],npcs[i],transitions)
        maquinas.append(q)
        

    
    
        

    pc.nodeat = gamegraph.inwhatnode2d(pc.p)       
    
    for npc in npcs:
        npc.path = npc.seekInGraph(gamegraph,pc.p)



    ###############################################################################
    ######################### CICLO PRINCIPAL DEL MAIN ############################
    ###############################################################################

    
    while True:
        


        
        #start = time.time()
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
                if event.key == K_p:
                    if  not seegraph:
                        seegraph = True
                if event.key == K_o:
                    if seegraph:
                        seegraph = False                

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
        inobj = [False,False]


                
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


        if seegraph:
            for nodes in gamegraph.nodes:
                nodes.triangle.draw()

            gamegraph.draw()

        glEnable(GL_DEPTH_TEST)

      
        for plat in plattforms:
            if not seegraph:
                gridat(plat[0],plat[1],plat[2],gedges,vertexx,vertexy)
            out_stage = out_stage and outofgrid(stageBoundsUp ,stageBoundsDown ,pc,plat[0],plat[1])

        startjump = fallfromgrids(pc,plattforms,out_stage,startjump,stageBoundsUp ,stageBoundsDown)



        
        if not inobj[0]:
            for item in itemlist:
                if collided_sq_item(pc,item):
                    aux = list(pc.ms)
                    pc.stop()
                    pc.changemov([-aux[0],-aux[1],-aux[2]])
                    pc.act()
                    inobj[0] = True
        else:
            for item in itemlist:
                if collided_sq_item(pc,item):
                    continue
                inobj[0] = False

        for npc in npcs:
            if not inobj[1]:
                for item in itemlist:
                    if collided_sq_item(npc,item):
                        aux = list(npc.ms)
                        npc.stop()
                        npc.changemov([-aux[0],-aux[1],-aux[2]])
                        npc.act()
                        inobj[1] = True
            else:
                for item in itemlist:
                    if collided_sq_item(npc,item):
                        continue
                    inobj[1] = False



        for i in range(0,len(npcs)):          
            for plat in plattforms:
                out_stage = out_stage and outofgrid(stageBoundsUp ,stageBoundsDown ,npcs[i],plat[0],plat[1])
            fallfromgridsnpc(npcs[i],plattforms,out_stage,stageBoundsUp ,stageBoundsDown)
            
        pc.friction(MOV)
        pc.act()


        
        for i in range(0,len(npcs)):
            action = maquinas[i].update()
            for j in action:
                if j:
                    behavior = j(npcs[i])
                    npcs[i].behave(behavior)


        for npc in npcs:
            if npc.p[2] > -10*LIM:
                npc.friction(MOV)
            npc.act()
            if npc.p[2] > -10*LIM:
                npc.drawnpc()

        pc.o = pc.orientationSteering()

        if pc.p[2] > -10*LIM:
            pc.cubeman()


        for obstacle in itemlist:
            obstacle.draw()
            
        pygame.display.flip()


############ Jugador #################
player = Kinematic([-12,0,0],[0,0,0])
######################################

############ NPCs ####################
npc1 = Kinematic([12,0,0],list([0,0,0]),0.75,0,0,0.1)
npc2 = Kinematic([0,-12,0],list([0,0,0]),0.75,0,0,0.1)
######################################



############ Obstáculos ##############

obstacles = []
o = obstacle((-12,-6,0), [],[])
o.makewall(6,2,2)
obstacles.append(o)

o = obstacle((-12,6,0), [],[])
o.makewall(6,2,2)
obstacles.append(o)

o = obstacle((-6,12,0), [],[])
o.makewall(2,6,2)
obstacles.append(o)

o = obstacle((6,12,0), [],[])
o.makewall(2,6,2)
obstacles.append(o)

o = obstacle((12,6,0), [],[])
o.makewall(6,2,2)
obstacles.append(o)

o = obstacle((12,-6,0), [],[])
o.makewall(6,2,2)
obstacles.append(o)

o = obstacle((6,-12,0), [],[])
o.makewall(2,6,2)
obstacles.append(o)

o = obstacle((-6,-12,0), [],[])
o.makewall(2,6,2)
obstacles.append(o)

    
o = obstacle((0,0,0), [],[])
o.makecube(6)
obstacles.append(o)



    

##################################

##################################
########)---(START)---(###########
##################################

main(player,obstacles,[npc1,npc2])

##################################



