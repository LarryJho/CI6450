import numpy
from OpenGL.GL import *
from OpenGL.GLU import *

def trianglearea(a,b,c):
    u = []
    v = []
    for i in range(0,3):
        u.append(a[i]-b[i])
        v.append(c[i]-b[i])
    area = numpy.linalg.norm(numpy.cross(u,v)/2)
    return area

def incentro(a,b,c):
    u = []
    v = []
    w = []
    for i in range(0,3):
        u.append(a[i]-b[i])
        v.append(b[i]-c[i])
        w.append(c[i]-a[i])
    r = []
    cop = numpy.linalg.norm(u)
    aop = numpy.linalg.norm(v)
    bop = numpy.linalg.norm(w)
    perimeter = aop + bop + cop
    for i in range(0,3):
        r.append((aop*(a[i])+bop*(b[i])+cop*(c[i]))/perimeter)
    return r

def baricentro(a,b,c):
    u = []
    v = []
    w = []
    for i in range(0,3):
        u.append((a[i]+b[i])/2)
        v.append((b[i]+c[i])/2)
        w.append((c[i]+a[i])/2)
    r = []
    for i in range(0,3):
        r.append((u[i]+v[i]+w[i])/3)
    return r
    

class triangle:
    def __init__(self, v1,v2,v3):
        self.v1 = v1
        self.v2 = v2
        self.v3 = v3

    def area(self):
        return trianglearea(self.v1, self.v2, self.v3)

    def draw(self):
        d1 = list(self.v1)
        d2 = list(self.v2)
        d3 = list(self.v3)
        d1[2] += 1
        d2[2] += 1
        d3[2] += 1
        glBegin(GL_LINES)
        glColor3fv((1,1,1))    
        glVertex3fv(d1)
        glVertex3fv(d2)
        glVertex3fv(d2)
        glVertex3fv(d3)      
        glVertex3fv(d3)
        glVertex3fv(d1)
        glEnd()


    def isin(self, point):
        x = self.area()
        a1 = trianglearea(self.v1,self.v2,point)
        a2 = trianglearea(self.v1,point,self.v3)
        a3 = trianglearea(point,self.v2,self.v3)
        #print (x , ' == ', a1,'+',a2,'+0',a3)
        return (abs( a1+a2+a3-x) < 0.5)

    def incentro(self):
        return incentro(self.v1, self.v2, self.v3)

    def baricentro(self):
        return baricentro(self.v1,self.v2,self.v3)

    def share2(self,t):
        same = 0
        if (self.v1 == t.v1) or (self.v1 == t.v2) or (self.v1 == t.v3):
            same += 1
        if (self.v2 == t.v1) or (self.v2 == t.v2) or (self.v2 == t.v3):
            same += 1        
        if (self.v3 == t.v1) or (self.v3 == t.v2) or (self.v3 == t.v3):
            same += 1
        return (same == 2)
