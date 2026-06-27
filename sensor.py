import random

class Sensor:
    def __init__(self,noise=0):
        self.noise=noise
    
    def measure(self,drone):
        sensor_x= drone.x + random.randint(-self.noise,self.noise)
        sensor_y= drone.y + random.randint(-self.noise,self.noise)

        return(sensor_x,sensor_y)
    