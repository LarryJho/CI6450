import math
import numpy

#Steering Output
class SteeringOutput:

    def __init__(self, lin , ang):
        self.linear = lin
        self.angular = ang

    def normalize(self):
        norm = self.norm()
        for i in range (0,3):
            self.linear[i] /= norm

    def norm(self):
        return numpy.linalg.norm(self.linear)

    def normalize2d(self):
        norm = self.norm2d()
        for i in range (0,2):
            self.linear[i] /= norm

    def norm2d(self):
        return numpy.linalg.norm(self.linear[0:2])      

