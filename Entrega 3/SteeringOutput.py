import math

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
        xyz = self.linear[0]*self.linear[0]
        xyz += self.linear[1]*self.linear[1]
        xyz += self.linear[2]*self.linear[2]        
        return math.sqrt(xyz)

    def normalize2d(self):
        norm = self.norm2d()
        for i in range (0,2):
            self.linear[i] /= norm

    def norm2d(self):
        xy = self.linear[0]*self.linear[0]
        xy += self.linear[1]*self.linear[1]       
        return math.sqrt(xy)
