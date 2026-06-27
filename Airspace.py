class Airspace:
    def __init__(self):
        self.drones=[]
    
    def add_drones(self,drone):
        self.drones.append(drone)

    def update(self):
        for drone in self.drones:
            drone.move()
