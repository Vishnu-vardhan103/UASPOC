from drone import Drone
from airspace import Airspace
from sensor import Sensor
from tracker import Tracker, Kalman_tracker
STRAIGHT = "Straight"
PATROL = "Patrol"
CIRCLE = "Circle"
INTRUDER="Intruder"
UNKNOWN="Unknown"

#specifying noise in sensor
sensor=Sensor(5)

airspace=Airspace()

# the following lines add drones based on simulator model

airspace.add_drones(
    Drone(1,STRAIGHT,100,50,10,5)
)

airspace.add_drones(
    Drone(2,PATROL,300,200,-10,10,270,330,170,230)
)

airspace.add_drones(
    Drone(3,INTRUDER,50,400,-4,3,target_x=70,target_y=370)
)

trackers = {}
kalman_trackers = {}
total_sensor_error=0
total_kalman_error=0

for drone in airspace.drones:

    kalman_trackers[drone.id] = Kalman_tracker()
    

# time is var here and i have assumed 1 second gap between each sensor measurement


for t in range(1,100):
    airspace.update()
    for drone in airspace.drones:
        reading=sensor.measure(drone)
        x,y=reading
    
        kalman_trackers[drone.id].predict()

        est_x, est_y = (
            kalman_trackers[drone.id].update(x,y)
        )


     

for drone in airspace.drones:
    print(f"Drone_id is {drone.id}")
    print(f"Kalaman predicts behaviour as {kalman_trackers[drone.id].get_state()}")
    print(f"kalman bounds is {kalman_trackers[drone.id].patrol_bounds()}")
    print(f"kalman predicts path is {kalman_trackers[drone.id].predict_path(3)}")
    print(f"_________________EOD___________________________\n\n")
    





