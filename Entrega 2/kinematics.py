import numpy
import math
import random
from pathgraph import *



from numpy import zeros, ones, arange, asarray, concatenate
from scipy.optimize import linprog
from scipy.spatial import ConvexHull

from SteeringOutput import *
from OpenGL.GL import *
from OpenGL.GLU import *


######### Funcion que detecta colisiones. ############
######  Es la unica que no esta hecha por mi #########
######  La dejo como referencia para poligonos #######
######  Mas complejos que cubos y paralelepipedos ####
######  Es O(nlog(n)) asi que lagea el juego :( ######
######  Es posible hacer una lineal. #################

def pnt_in_cvex_hull_1(hull, pnt):
    
    '''
    Checks if `pnt` is inside the convex hull.
    `hull` -- a QHull ConvexHull object
    `pnt` -- point array of shape (3,)
    '''
    new_hull = ConvexHull(concatenate((hull.points, [pnt])))
    return numpy.array_equal(new_hull.vertices, hull.vertices)


#########  Funcion que determina si un objeto colisiono un obstaculo  #########

def collided_p_o(player,obstacle):
    for vertex in player.verticies:
        if pnt_in_cvex_hull_1(obstacle.hull,[vertex[0]+player.p[0],vertex[1]+player.p[1],vertex[2]+player.p[2]]):
            return True
    return False

#Funcion de colision entre personaje y obstaculo

def collided_sq_item(pc,obstacle,):
    col = 0
    for i in range(0,2):
        if (pc.p[i] <= obstacle.position[i]+obstacle.sizes[i]/2+pc.body/2) and (pc.p[i] >= obstacle.position[i]-obstacle.sizes[i]/2-pc.body/2):
            col += 1
    if (pc.p[2] <= obstacle.position[2]+obstacle.sizes[2]) and (pc.p[2] >= obstacle.position[2]-obstacle.sizes[2]-pc.body):
        col += 1 
       
    if col == 3:
        return True
    return False

######### Funcion que determina si un objeto esta cerca de un obstaculo ########

def cerca(player,obstacle):
    c = []
    for i in range(0,3):
        c.append(player.p[i] - obstacle.position[i])

    aux = (numpy.linalg.norm(c) < ((player.body/2)+(obstacle.maxp*math.sqrt(2)/2)))
    return aux

######### Clase Obstaculo #########     
        
class obstacle:
    def __init__(self,position,verticies,sizes,typee = "none"):
        self.position = position
        self.verticies = verticies
        self.sizes = sizes
        self.maxp = 0
        self.type = typee
        self.hull = None

    def makecube(self,size):
        vert = [
        (self.position[0]+(size/2),self.position[1]-(size/2), self.position[2]),
        (self.position[0]+(size/2),self.position[1]+(size/2), self.position[2]),
        (self.position[0]-(size/2),self.position[1]+(size/2), self.position[2]),
        (self.position[0]-(size/2),self.position[1]-(size/2), self.position[2]),
        (self.position[0]+(size/2),self.position[1]-(size/2), self.position[2]+size),
        (self.position[0]+(size/2),self.position[1]+(size/2), self.position[2]+size),
        (self.position[0]-(size/2),self.position[1]-(size/2), self.position[2]+size),
        (self.position[0]-(size/2),self.position[1]+(size/2), self.position[2]+size),
        ]
        self.verticies = vert
        if len(self.sizes) < 3:
            self.sizes.append(size)
            self.sizes.append(size)
            self.sizes.append(size)
        else:
            for i in range (0,3):
                sizes[i] = size
        self.maxp = size
        self.type = "cube"
        self.hull =  ConvexHull(vert)

    def makewall(self,sizex, sizey, sizez):
        vert = [
        (self.position[0]+(sizex/2),self.position[1]-(sizey/2), self.position[2]),
        (self.position[0]+(sizex/2),self.position[1]+(sizey/2), self.position[2]),
        (self.position[0]-(sizex/2),self.position[1]+(sizey/2), self.position[2]),
        (self.position[0]-(sizex/2),self.position[1]-(sizey/2), self.position[2]),
        (self.position[0]+(sizex/2),self.position[1]-(sizey/2), self.position[2]+sizez),
        (self.position[0]+(sizex/2),self.position[1]+(sizey/2), self.position[2]+sizez),
        (self.position[0]-(sizex/2),self.position[1]-(sizey/2), self.position[2]+sizez),
        (self.position[0]-(sizex/2),self.position[1]+(sizey/2), self.position[2]+sizez),
        ]
        self.verticies = vert
        if len(self.sizes) < 3:
            self.sizes.append(sizex)
            self.sizes.append(sizey)
            self.sizes.append(sizez)
        else: 
            self.sizes[0] = sizex
            self.sizes[1] = sizey
            self.sizes[2] = sizez
        self.maxp = max(self.sizes[0],self.sizes[1],self.sizes[2])
        self.type = "wall"
        self.hull =  ConvexHull(vert)



    def draw(self):

        if (self.type == "cube" or self.type == "wall"):


            edges = [
                (0,1),
                (0,3),
                (0,4),
                (2,1),
                (2,3),
                (2,7),
                (6,3),
                (6,4),
                (6,7),
                (5,1),
                (5,4),
                (5,7),
            ]

            colors = [
                (1,0,0),
                (1,1,0),
                (1,1,0),
                (0,1,0),
            ]

            surfaces = [
                (0,1,2,3),
                (3,2,7,6),
                (6,7,5,4),
                (4,5,1,0),
                (1,5,7,2),
                (4,0,3,6)
            ]

            glBegin(GL_QUADS)
            for surface in surfaces:
                x = 0
                for vertex in surface:
                    glColor3fv(colors[x])
                    glVertex3fv(self.verticies[vertex])
                    x+=2
                    x = x%3
            glEnd()
            


########### Manejos de colisiones de la primera entrega ############

class Collision:
    def __init__(self, point, normal):
        self.point = point
        self.normal = normal
    
class CollisionDectector:
    def getCollision(position,moveAmount):
        return None

class SqBoundariesCollision:

    def __init__(self,maxbound,minbound):
        self.maxs = maxbound
        self.mins = minbound
        
    def getCollision(self,position,moveAmount):
        data = position
        normal = [0,0,0]
        colliding = False
        for i in range(0,2):
            futurecoord = position[i] + moveAmount[i]
            if futurecoord >= self.maxs[i]:
                data[i] = self.maxs[i]
                normal[i] = -self.maxs[i]/(self.maxs[i]*(futurecoord-self.maxs[i]))
                colliding = True
            else:
                if futurecoord <= self.mins[i]:
                    data[i] = self.mins[i]
                    normal[i] = -self.maxs[i]/self.maxs[i]*(futurecoord-self.maxs[i])
                    colliding = True
        if colliding:
            for i in range(0,2):
                normal[i] /= numpy.linalg.norm(normal)
                normal[i] *= 1.5
            return Collision(data, normal)
        else:
            return None


        
############ Objeto cinematico ##############
        
class Kinematic:

    
    def __init__(self,position, mov_speed, body = 1, orientation = 0, rot_speed = 0, max_speed = 0.15,  max_rot = 1, max_accel = 0.1, max_ang_a = 1):
        self.p = position
        self.o = (orientation) % 360
        self.ms = mov_speed
        self.rs = rot_speed
        self.max = max_speed
        self.max_rot = max_rot
        self.max_accel = max_accel
        self.max_ang_a = max_ang_a
        self.path = None
        self.nodeat = None
        self.body = body
        self.verticies = [
            ((self.body/2), -(self.body/2), 0),
            ((self.body/2), (self.body/2), 0),
            (-(self.body/2), (self.body/2), 0),
            (-(self.body/2), -(self.body/2), 0),
            ((self.body/2), -(self.body/2), 1),
            ((self.body/2), (self.body/2), 1),
            (-(self.body/2), -(self.body/2), 1),
            (-(self.body/2), (self.body/2), 1),
            #rombo
            ((self.body/2)+0.1, 0, 0),
            ((self.body/2)+0.1, 0, 1),
            ((self.body/2)+0.1, (self.body/2), (self.body/2)),
            ((self.body/2)+0.1, -(self.body/2), (self.body/2)),
            (0,0,0)
            ]

    def evaluate(self,function):
        return function(self)
        
    def dirmov(self):
        isneg = [False,False,False]
        for i in range(0,3):
            isneg[i] = (self.ms[i] < 0)
        return isneg

    def distance(self):
        xyz = self.p[0]*self.p[0]
        xyz += self.p[1]*self.p[1]
        xyz += self.p[2]*self.p[2]        
        return math.sqrt(xyz)

    def speed(self):
        xyz = self.ms[0]*self.ms[0]
        xyz += self.ms[1]*self.ms[1]
        xyz += self.ms[2]*self.ms[2]        
        return math.sqrt(xyz)

    def relativeSpeed(self,target):
        xyz = (target.ms[0]-self.ms[0])*(target.ms[0]-self.ms[0])
        xyz += (target.ms[1]-self.ms[1])*(target.ms[1]-self.ms[1])
        xyz += (target.ms[2]-self.ms[2])*(target.ms[2]-self.ms[2])        
        return math.sqrt(xyz)

    def relativePos(self,target):
        xyz = (target.p[0]-self.p[0])*(target.p[0]-self.p[0])
        xyz += (target.p[1]-self.p[1])*(target.p[1]-self.p[1])
        xyz += (target.p[2]-self.p[2])*(target.p[2]-self.p[2])        
        return math.sqrt(xyz)


    def orientationSteering(self):
        if (self.ms[0] == 0) and (self.ms[1] == 0) :
            return self.o
        else:
            return (math.atan2(self.ms[1],self.ms[0]))*180/math.pi

                
    def act(self):
        for i in range(0,3):
            self.p[i] += self.ms[i]
        self.o = self.o + self.rs
        self.o = (self.o) % 360



    ################ MOVIMIENTOS ############################


    def changemove(self, sout):
        self.changemov(sout.linear)
        self.changerot(sout.angular)
        
    def changepos(self,pos):
        self.p[0] += pos[0]
        self.p[1] += pos[1]
        self.p[2] += pos[2]

    def changemov(self,mov):
        self.ms[0] += mov[0]
        self.ms[1] += mov[1]
        self.ms[2] += mov[2]
        x = []
        x += self.ms
        aux = SteeringOutput(x,0)
        if aux.norm2d() != 0:
            aux.normalize2d()
        for i in range(0,2):
            if self.ms[i] > aux.linear[i]*self.max and aux.linear[i] > 0:
                self.ms[i] = aux.linear[i]*self.max
            if self.ms[i] < aux.linear[i]*self.max and aux.linear[i] < 0:
                self.ms[i] = aux.linear[i]*self.max
        if self.ms[2] > 0.5:
            self.ms[2] = 0.5
        if self.ms[2] < -0.5:
            self.ms[2] = -0.5

    def stop(self):
        self.ms[0] = 0
        self.ms[1] = 0
        self.ms[2] = 0

    def stopjump(self):
        self.ms[2] = 0
        
    def oldchangemov(self,mov):
        self.ms[0] += mov[0]
        self.ms[1] += mov[1]
        self.ms[2] += mov[2]
        for i in range(0,3):
            if self.ms[i] > self.max:
                self.ms[i] = self.max
            if self.ms[i] < -self.max:
                self.ms[i] = -self.max


    def slowspd(self,mov):
        negspd = self.dirmov()
        self.ms[0] -= mov[0]
        self.ms[1] -= mov[1]
        self.ms[2] -= mov[2]   
        for i in range(0,3):
            if negspd[i]:
                if self.ms[i] > 0:
                    self.ms[i] = 0
            else:
                if self.ms[i] < 0:
                    self.ms[i] = 0

                    
    def changeori(self,alpha):
        self.o += alpha

    def changerot(self,ro):
        self.rs += ro
        if self.rs > self.max_rot:
            self.rs = self.max_rot
        if self.rs < -self.max_rot:
            self.rs = -self.max_rot

    def slowrot(self,ro):
        negspd = (self.rs < 0)
        self.rs -= ro
        if negspd:
            if self.rs > 0:
                self.rs = 0
        else:
            if self.rs < 0:
                self.rs = 0

    def simpleJump(self,strength):
        self.changemov([0,0,strength])

    def gravity(self):
        self.changemov([0,0,-0.0098])

    def isGrounded(self,groundlevel):
        if self.p[2] <= groundlevel:
            return True
        else: return False

    def groundLeveled(self,groundlevel):
        if self.p[2] < groundlevel:
            self.p[2] = groundlevel

    def behave(self,data):
        if data != None:
            self.changerot(data.angular)
            self.changemov(data.linear)

                
    ################ COMPORTAMIENTOS ############################

    def simpleWander(self):
        data = SteeringOutput([0,0,0], 0)
        data.linear[0] = self.max*math.cos(self.o*math.pi/180)
        data.linear[1] = self.max*math.sin(self.o*math.pi/180)
        data.angular = random.uniform(-1,1)*self.max_rot
        return data

        

    def arrive(self,target,targetradius = 0.5, slowradius = 1.5):

        
        timetotarget = 1


        data = SteeringOutput([0,0,0], 0)
        for i in range (0,3):
            data.linear[i] = target.p[i] - self.p[i]
        distance = data.norm()
 
        if distance > targetradius:     
        
            if distance > slowradius:
                targetspeed = self.max
            else:
                targetspeed = self.max*distance/slowradius

            data.normalize
            for i in range (0,3):
                data.linear[i] *= targetspeed
                data.linear[i] -= self.ms[i]
                data.linear[i] /= timetotarget

        return data

    def arrivePoint(self,point,targetradius = 0.5, slowradius = 1.5):

        
        timetotarget = 1


        data = SteeringOutput([0,0,0], 0)
        for i in range (0,3):
            data.linear[i] = point[i] - self.p[i]
        distance = data.norm()
 
        if distance > targetradius:     
        
            if distance > slowradius:
                targetspeed = self.max
            else:
                targetspeed = self.max*distance/slowradius

            data.normalize
            for i in range (0,3):
                data.linear[i] *= targetspeed
                data.linear[i] -= self.ms[i]
                data.linear[i] /= timetotarget

        return data
        


    def align(self,target,slowRadius = 20):
        maxAngularAxceleration = 0
        maxRotation = 0
        targetRadius = target.o
        targetRotation = 0


        data = SteeringOutput([0,0,0], 0)
        rota = target.o - self.o
        rotb = self.o - target.o
        if math.fabs(rota) > math.fabs(rotb):
            rotation = rotb
        else:
            rotation = rota
        rotation1 = math.fabs(rotation)
        
        if rotation1 > 0:
            if rotation1 > slowRadius:
                targetRotation = self.max_rot
            else:
                targetRotation = self.max_rot*rotation1/slowRadius

            targetRotation *= rotation/rotation1
            data.angular = targetRotation - self.rs

            angularAcceleration = math.fabs(data.angular)
            if angularAcceleration > self.max_ang_a:
                data.angular /= angularAcceleration
                data.angular *= self.max_ang_a

        return data

    def velocitymatch(self,target):
        data = SteeringOutput([0,0,0], 0)
        for i in range (0,3):
            data.linear[i] = target.ms[i] - self.ms[i]

        if data.linear.norm() > self.max_accel:
            data.normalize()
            data.linear *= self.max_accel
        return data

    def seek(self,target):
        if target is not None:
            data = SteeringOutput([0,0,0], 0)
            for i in range (0,3):
                data.linear[i] = target.p[i] - self.p[i]
            data.normalize
            for i in range (0,3):
                data.linear[i] *= self.max
            return data
        else:
            return None

    def seekPoint(self,point):
        data = SteeringOutput([0,0,0], 0)
        for i in range (0,3):
            data.linear[i] = point[i] - self.p[i]
        data.normalize
        for i in range (0,3):
            data.linear[i] *= self.max
        return data

    def seekPoint2d(self,point):
        data = SteeringOutput([0,0,0], 0)
        for i in range (0,2):
            data.linear[i] = point[i] - self.p[i]
        data.normalize
        for i in range (0,2):
            data.linear[i] *= self.max
        return data

    def seekInGraph(self,graph,point):
        n = graph.inwhatnode2d(self.p)
        if n:
            n_p = graph.inwhatnode2d(point)
            p = graph.pathfindastar(n,n_p)
            if (p != None) and (p[0].connection != None):
                return p 
                #return self.seekPoint(p[0].connection.nodeto.location)
        return None

    def seekRandNode(self,graph):
        n = graph.inwhatnode2d(self.p)
        if n:
            #print(n.name)
            l1 = graph.connections[n.name][random.randrange(0,len(graph.connections[n.name]))].nodeto.location
        else: 
            return None
        return self.seekPoint(l1)



    def pursue(self,target,maxPrediction):
        direction = SteeringOutput([0,0,0], 0)
        for i in range (0,3):
            direction.linear[i] = target.p[i] - self.p[i]
        distance = direction.norm()
        speed = self.speed()
        if speed <= distance/maxPrediction:
            prediction = maxPrediction
        else:
            prediction = distance / speed

        predicted = Kinematic([0,0,0],[0,0,0])
        for i in range (0,3):
            predicted.p[i] += target.p[i]*prediction
        data = self.seek(target)
        return data
        
    def flee(self,target,distance = 5):
        data = SteeringOutput([0,0,0], 0)
        for i in range (0,3):
            data.linear[i] = self.p[i] - target.p[i]

        aux = data.norm() < distance
        data.normalize()
        
        for i in range (0,3):
            data.linear[i] *= self.max

        if aux:
            return data
        else:
            return SteeringOutput([0,0,0], 0)

    def collisionAvoidance(self,targets,radius):
        shortestTime = float('inf')
        firstTarget = None
        firstMinSeparation = 0
        firstDistance = 0
        firstRelativePos = None
        firstRelativeVel = None

        for target in targets:
            relativePos = [0,0,0]
            relativeVel = [0,0,0]
            for i in range (0,3):
                relativePos[i] = target.p[i] - self.p[i]
                relativeVel[i] = target.ms[i] - self.ms[i]
            relativeSpd = self.relativeSpeed(target)
            dot = 0
            for i in range(0,3):
                dot += relativePos[i]*relativeVel[i]
            if relativeSpd != 0:
                timeToCollision = (dot) / (relativeSpd * relativeSpd)
            else:
                timeToCollision = 0
            xyz = relativePos[0]*relativePos[0]
            xyz += relativePos[1]*relativePos[1]
            xyz += relativePos[2]*relativePos[2]        
            distance= math.sqrt(xyz)
            minSeparation = distance-relativeSpd*shortestTime
            if minSeparation > 2*radius:
                continue
            if timeToCollision > 0 and timeToCollision < shortestTime:
                shortestTime = timeToCollision
                firstTarget = target
                firstMinSeparation = minSeparation
                firstRelativePos = relativePos
                firstRelativeVel = relativeVel

        if not firstTarget: return None
        if firstMinSeparation <= 0 or distance < 2*radius:
            for i in range (0,3):
                relativePos[i] = target.p[i] - self.p[i]
        else:
            for i in range (0,3):
                relativePos[i] = firstRelativePos[i] + firstRelativeVel[i]*shortestTime
                relativePos[i] /= numpy.linalg.norm(relativePos)
        final = [0,0,0]
        for i in range (0,3):
            final[i] = relativePos[i]*self.max_accel
        return SteeringOutput(final,0)


    def obstacleAvoidance(self,detector,avoidDistance,lookahead):
        rayVector = list(self.ms)
        norm = numpy.linalg.norm(self.ms)
        if norm !=0:
            
            for i in range(0,3):
                rayVector[i] /= norm
                rayVector[i] *= lookahead
            collision = detector.getCollision([self.p[0],self.p[1],self.p[2]],rayVector)
            if collision is None:
                return None
            target = [0,0,0]
            for i in range(0,3):
                target[i] = collision.point[i] + collision.normal[i]*avoidDistance
 
            data = Kinematic(target,[0,0,0])
            return self.seek(data)
        return None
    
    ##################### OTROS ##########################

    def blending(self,behaviors,weigths):
        data = SteeringOutput([0,0,0],0)
        j = 0
        for behavior in behaviors:
            behavior
            if behavior != None:
                for i in range(0,3):
                    data.linear[i] += weigths[j]*behavior.linear[i]
                data.angular += weigths[j]*behavior.angular
                j += 1
        
        #for i in range(0,3):
        #    data.linear[i] = max(data.linear[i],self.max_accel)
        #data.angular = max(data.angular,self.max_ang_a)
        return data
        
    def friction(self,m):
        if self.ms[1] < 0:
            self.slowspd([0,-m*0.5,0])
        else:
            if self.ms[1] > 0:
                self.slowspd([0,m*0.5,0])
                
        if self.ms[0] < 0:
            self.slowspd([-m*0.5,0,0])
        else:
            if self.ms[0] > 0:
                self.slowspd([m*0.5,0,0])


        if self.rs < 0:
            self.slowrot(-m*0.5)
        else:
            if self.rs > 0:
                self.slowrot(m*0.5)

    #### Dibuja un cubo ####
    def cubeman(self):
        glPushMatrix();
        glTranslatef(self.p[0],self.p[1],self.p[2])
        glRotatef(self.o, 0, 0, 1)        



        edges = [
            (0,1),
            (0,3),
            (0,4),
            (2,1),
            (2,3),
            (2,7),
            (6,3),
            (6,4),
            (6,7),
            (5,1),
            (5,4),
            (5,7),
            #rombo
            (8,10),
            (9,10),
            (8,11),
            (9,11)
            ]
        
        colors = [
            (1,1,0),
            (0,1,0),
            (1,1,0),
            (0,1,0),
            ]

        surfaces = (
            (0,1,2,3),
            (3,2,7,6),
            (6,7,5,4),
            (4,5,1,0),
            (1,5,7,2),
            (4,0,3,6)
            )

        glBegin(GL_QUADS)
        for surface in surfaces:
            x = 0
            for vertex in surface:
                glColor3fv(colors[x])
                glVertex3fv(self.verticies[vertex])
                x+=2
                x = x%3
        glEnd()

        
        glBegin(GL_LINES)
        glColor3fv((1,0,0))
        for edge in edges:
            for vertex in edge:
                glVertex3fv(self.verticies[vertex])
        glEnd()


        glRotatef(-self.o, 0, 0, 1)
        glTranslatef(-self.p[0],-self.p[1],self.p[2])
        glPopMatrix();

    #### Dibuja un NPC ####
    def drawnpc(self):
        glPushMatrix();
        glTranslatef(self.p[0],self.p[1],self.p[2])
        glRotatef(self.o, 0, 0, 1)        


        edges = [
            (0,1),
            (0,3),
            (0,4),
            (2,1),
            (2,3),
            (2,7),
            (6,3),
            (6,4),
            (6,7),
            (5,1),
            (5,4),
            (5,7),
            #rombo
            (8,10),
            (9,10),
            (8,11),
            (9,11)
            ]
        
        colors = [
            (0,1,1),
            (0,1,0),
            (0,1,1),
            (0,1,0),
            ]

        surfaces = (
            (0,1,2,3),
            (3,2,7,6),
            (6,7,5,4),
            (4,5,1,0),
            (1,5,7,2),
            (4,0,3,6)
            )

        glBegin(GL_QUADS)
        for surface in surfaces:
            x = 0
            for vertex in surface:
                glColor3fv(colors[x])
                glVertex3fv(self.verticies[vertex])
                x+=2
                x = x%3


        glEnd()
        
        glBegin(GL_LINES)
        glColor3fv((1,0,0))
        for edge in edges:
            for vertex in edge:
                glVertex3fv(self.verticies[vertex])
        glEnd()


        glRotatef(-self.o, 0, 0, 1)
        glTranslatef(-self.p[0],-self.p[1],-self.p[2])
        glPopMatrix();


