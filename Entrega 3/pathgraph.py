import numpy
import math
import random
from trianglesmaps import *
from SteeringOutput import *
from OpenGL.GL import *
from OpenGL.GLU import *

############### GRAFO DE CAMINOS #################


class node:

    def __init__(self, location, triangle):
        self.location = location
        self.name = str(location)
        self.triangle = triangle

    def share2(self, n):
        return self.triangle.share2(n.triangle)

class connection:

    def __init__(self, nodefrom, nodeto, weight):
        self.nodefrom = nodefrom
        self.nodeto = nodeto
        self.weight = weight


    def addw(self):
        self.weight += 5

    def minusw(self):
        self.weight -= 5

#El conjunto de aristas se va a implementar como un diccionario cuya clave es
#la posicion hecha string y el valor es una lista de aristas que empiezan en
#esa posicion De esta manera se garantiza que no hayan dos o mas nodos en un
#mismo lugar y se utilizan las propiedades de tabla de hash del diccionario
#python para mejorar el tiempo de busqueda

class pathgraph:

    def __init__(self, nodes, connections):
        self.nodes = nodes
        self.connections = connections

    def addnode(self, node):
        self.nodes.append(node)
        self.connections[node.name] = []

    def addconnection(self,connection):
        self.connections[connection.nodefrom.name].append(connection)

    def getconnections(self,node):
        return self.connections[node.name]

    def getnodefrom(self,connection):
        return self.connection.nodefrom

    def getnodeto(self,connection):
        return self.connection.nodeto

    def getweight(self,connection):
        return self.connection.weight

    def connected(self, node1, node2):
        conns = self.connections[node1.name]
        for conn in conns:
            if conn.nodeto == node2:
                return True
        return False

    def ingraph(self, location):
        for node1 in self.nodes:
            if node1.triangle.isin(location):
                return True
        return False

    def inwhatnode2d(self, location):
        loc2d = [location[0],location[1],0]
        for node1 in self.nodes:
            if node1.triangle.isin(loc2d):
                return node1
        return None

#################################################

    def drawconnections(self,node1):
        lst = self.connections[node1.name]
        for connection in lst:
            fr = connection.nodefrom.location
            to = connection.nodeto.location
            glBegin(GL_LINES)
            glColor3fv((1,1,0))    
            glVertex3fv(to)
            glVertex3fv(fr)
            glEnd()

    def draw(self):
        for nodee in self.nodes:
            self.drawconnections(nodee)

################# Dijkstra #######################

    def pathfinddijkstra(self,start, end):

        class noderecord:
            def __init__(self,node,connection,cost = 0):
                self.node = node
                self.connection = connection
                self.cost = cost


        def smallest(recordlist):
            x = None
            if recordlist is not []:
                x = recordlist[0]
                for record in range(1,len(recordlist)):
                    if recordlist[record].cost < x.cost:
                        x = recordlist[record]
            return x       

        def findnode(recordlist, nodo):
            x = None
            if recordlist is not []:
                for record in recordlist:
                    if record.node == nodo:
                        x = record
                        break
            return x

        r = noderecord(start,None)

        openl = []
        openl.append(r)

        closedl = []

        while len(openl) > 0:
            current = smallest(openl)

            if current.node == end:
                break

            connections = self.getconnections(current.node)

            for connection in connections:

                endnode = connection.nodeto
                endnodecost = connection.weight + current.cost

                if endnode in closedl : continue
                elif endnode in openl:
                    endnoderecord = findnode(openl,endnode)

                    if endnoderecord.cost <= endnodecost:
                        continue

                else:
                    endnoderecord = noderecord(endnode, None) 

                endnoderecord.connection = connection
                endnoderecord.cost = endnodecost

                if endnode not in openl:
                    openl.append(endnoderecord)


            openl.remove(current)
            closedl.append(current)

        if current.node != end:
            return None

        else:
            path = []

            while current.node != start:
                path.append(current)
                current = findnode(closedl,current.connection.nodefrom)

            #for record in path:
            #   print(record.node.name)
            return list(reversed(path))



################ A* ###################

    def pathfindastar(self,start, end):

        class heuristic:
            def __init__(self, node):
                self.initial = node.location

            def estimate(self,goal):
                euclid = []
                x = goal.location
                for i in range(0,3):
                    euclid.append(x[i]-self.initial[i])
                euclid = numpy.linalg.norm(euclid)
            

        class noderecord:
            def __init__(self,node,connection,estimated,cost = 0):
                self.node = node
                self.connection = connection
                self.cost = cost
                self.estimated = estimated


        def smallest(recordlist):
            x = None
            if recordlist is not []:
                x = recordlist[0]
                for record in range(1,len(recordlist)):
                    if recordlist[record].cost < x.cost:
                        x = recordlist[record]
            #print(x)
            return x       

        def findnode(recordlist, nodo):
            x = None
            if recordlist is not []:
                for record in recordlist:
                    if record.node == nodo:
                        x = record
                        break
            return x

        def sameas(record1,record2):
            return (record1.node == record2.node) and (record1.connection == record2.connection)

        def inlist(record1, recordlist):
            for record2 in recordlist:
                if sameas(record1,record2):
                    return True
            return False

        heu = heuristic(start)
        r = noderecord(start,None,heu.estimate(start))
        endnodeheuristic =0

        openl = []
        openl.append(r)

        closedl = []

        while len(openl) > 0:
            #print(len(openl))
            current = smallest(openl)

            if current.node == end:
                break

            connections = self.getconnections(current.node)

            for connection in connections:

                endnode = connection.nodeto
                endnodecost = connection.weight + current.cost

                if endnode in closedl:
                    endnoderecord = findnode(closeldl,endnode)

                    if endnoderecord.cost <= endnodecost:
                        continue

                    closedl.remove(endnoderecord)

                    endnodeheuristic = endnodecost - endnoderecord.cost
                    #mosca error del libro aqui

                elif endnode in openl:
                    endnoderecord = findnode(openl,endnode)

                    if endnoderecord.cost <= endnodecost:
                        continue
                    
                    endnodeheuristic = endnodecost - endnoderecord.cost

                else:
                    endnoderecord = noderecord (endnode, None, heu.estimate(endnode))


                endnoderecord.connection = connection
                endnoderecord.cost = endnodecost
                endnoderecord.estimate = endnodecost + endnodeheuristic
                #print((endnoderecord not in openl),' BUT ',inlist(endnoderecord,openl))

                if  not inlist(endnoderecord,openl):
                    openl.append(endnoderecord)


            openl.remove(current)
            closedl.append(current)

        if current.node != end:
            return None



        else:
            pathshifter = [None]
            path = []





            while current.node != start:

                path.append(current)
                pathshifter.append(current.connection)
                current = findnode(closedl,current.connection.nodefrom)


            path.append(current)

            final = list(reversed(path))

            i = 0
            for record in path:
                record.connection = pathshifter[i]
                i +=1

            #for record in list(reversed(path)):
             #   print(record.node.name)
              #  if record.connection != None:
               #     print('from: ',record.connection.nodefrom.name,' to ', record.connection.nodeto.name)
                

            return list(reversed(path))


################ A* aumentando pesos para estrategia de emboscada ###################

    def pathfindastarplusw(self,start, end):

        class heuristic:
            def __init__(self, node):
                self.initial = node.location

            def estimate(self,goal):
                euclid = []
                x = goal.location
                for i in range(0,3):
                    euclid.append(x[i]-self.initial[i])
                euclid = numpy.linalg.norm(euclid)
            

        class noderecord:
            def __init__(self,node,connection,estimated,cost = 0):
                self.node = node
                self.connection = connection
                self.cost = cost
                self.estimated = estimated


        def smallest(recordlist):
            x = None
            if recordlist is not []:
                x = recordlist[0]
                for record in range(1,len(recordlist)):
                    if recordlist[record].cost < x.cost:
                        x = recordlist[record]
            #print(x)
            return x       

        def findnode(recordlist, nodo):
            x = None
            if recordlist is not []:
                for record in recordlist:
                    if record.node == nodo:
                        x = record
                        break
            return x

        def sameas(record1,record2):
            return (record1.node == record2.node) and (record1.connection == record2.connection)

        def inlist(record1, recordlist):
            for record2 in recordlist:
                if sameas(record1,record2):
                    return True
            return False

        heu = heuristic(start)
        r = noderecord(start,None,heu.estimate(start))
        endnodeheuristic =0

        openl = []
        openl.append(r)

        closedl = []

        while len(openl) > 0:
            #print(len(openl))
            current = smallest(openl)

            if current.node == end:
                break

            connections = self.getconnections(current.node)

            for connection in connections:

                endnode = connection.nodeto
                endnodecost = connection.weight + current.cost

                if endnode in closedl:
                    endnoderecord = findnode(closeldl,endnode)

                    if endnoderecord.cost <= endnodecost:
                        continue

                    closedl.remove(endnoderecord)

                    endnodeheuristic = endnodecost - endnoderecord.cost
                    #mosca error del libro aqui

                elif endnode in openl:
                    endnoderecord = findnode(openl,endnode)

                    if endnoderecord.cost <= endnodecost:
                        continue
                    
                    endnodeheuristic = endnodecost - endnoderecord.cost

                else:
                    endnoderecord = noderecord (endnode, None, heu.estimate(endnode))


                endnoderecord.connection = connection
                endnoderecord.cost = endnodecost
                endnoderecord.estimate = endnodecost + endnodeheuristic
                #print((endnoderecord not in openl),' BUT ',inlist(endnoderecord,openl))

                if  not inlist(endnoderecord,openl):
                    openl.append(endnoderecord)


            openl.remove(current)
            closedl.append(current)

        if current.node != end:
            return None



        else:
            pathshifter = [None]
            path = []





            while current.node != start:

                path.append(current)
                pathshifter.append(current.connection)
                current = findnode(closedl,current.connection.nodefrom)


            path.append(current)

            final = list(reversed(path))

            i = 0
            for record in path:
                record.connection = pathshifter[i]
                if record.connection != None:
                    record.connection.addw()
                i +=1

            #for record in list(reversed(path)):
             #   print(record.node.name)
              #  if record.connection != None:
               #     print('from: ',record.connection.nodefrom.name,' to ', record.connection.nodeto.name)
                

            return list(reversed(path))









        










        
