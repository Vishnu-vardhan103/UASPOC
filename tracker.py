STRAIGHT = "Straight"
PATROL = "Patrol"
CIRCLE = "Circle"
INTRUDER="Intruder"
UNKNOWN="Unknown"

import numpy as np

class Tracker:
    def __init__(self,drone_id,x,y):
        self.id=drone_id
        self.x=x
        self.y=y
        self.measurement=[]

    def get_pos(self,x,y):
        self.measurement.append((x,y))

    def calc_vel(self):
        if len(self.measurement)<2:
            return(0,0)
        else:
            x1,y1=self.measurement[-1]
            x2,y2=self.measurement[-2]
            vx=x1-x2
            vy=y1-y2
            
            return(vx,vy)
        
    def calc_avg_vel(self, window=5):
            n = min(window, len(self.measurement))

            if n < 2:
                return (0, 0)

            x1, y1 = self.measurement[-n]
            x2, y2 = self.measurement[-1]

            vx = (x2 - x1) / (n - 1)
            vy = (y2 - y1) / (n - 1)

            return (vx, vy)
    
    def predict_path(self,num):
        p_future=[]
        vx,vy=self.calc_avg_vel()
        x,y=self.measurement[-1]

        for _ in range(num):
                x+=vx
                y+=vy
                p_future.append((x,y))
                
        return p_future  

    def get_state(self,num):

        x, y = self.measurement[-1]
        vx, vy = self.calc_avg_vel(num)

        return {
            "id": self.id,
            "position": (round(x,2), round(y,2)),
            "velocity": (round(vx,2), round(vy,2)),
            "prediction": [
                (round(px,2), round(py,2))
                for px,py in self.predict_path(3)
            ]
        }
    
    def predict_motion(self,num):
         
        vx_avg, vy_avg = self.calc_avg_vel(num)
        vx_cur, vy_cur = self.calc_vel()
        threshold=0.5

        if abs(vx_cur - vx_avg) > threshold and abs(vy_cur - vy_avg) > threshold:
            return STRAIGHT

        return UNKNOWN
    

class Kalman_tracker:
        def __init__(self):
              
            self.x=np.array([
                   [0],
                   [0],
                   [0],
                   [0]

            ])

            self.P = np.eye(4) * 1000

            self.F = np.array([
                    [1,0,1,0],
                    [0,1,0,1],
                    [0,0,1,0],
                    [0,0,0,1]
            ])

            self.H = np.array([
                [1,0,0,0],
                [0,1,0,0]
            ])

            self.R = np.array([
                [25,0],
                [0,25]
            ])

            self.Q = np.eye(4) * 0.1

            self.kmeasurement=[]
            self.sensor_measurements=[]

            self.x_max_hits = []
            self.x_min_hits = []
            self.y_max_hits = []
            self.y_min_hits = []

            self.x_max_flag = False
            self.x_min_flag = False
            self.y_max_flag = False
            self.y_min_flag = False

        def predict(self):
            self.x = self.F @ self.x

            self.P = self.F @ self.P @ self.F.T + self.Q

            return self.x
        
        def update(self, measured_x, measured_y):
            
            self.sensor_measurements.append((measured_x,measured_y))
            z = np.array([
                [measured_x],
                [measured_y]
            ])

            y = z - (self.H @ self.x)

            S = self.H @ self.P @ self.H.T + self.R

            K = self.P @ self.H.T @ np.linalg.inv(S)
            self.x = self.x + (K @ y)

            I = np.eye(4)

            self.P = (I - K @ self.H) @ self.P


            self.kmeasurement.append((
                round(float(self.x[0,0]), 2),
                round(float(self.x[1,0]), 2)
            ))

          

            self.state=0
            
            
            
            return (
                self.x[0,0],
                self.x[1,0]
            )
        
       
            
        def avg_vel(self,window=5):
             
            n=min(len(self.kmeasurement),window)
            if n<2:
                  return(0,0)
             
            vel=[]
            vx=0
            vy=0
            for i in range(-n,-1):
                x1,y1=self.kmeasurement[i]
                x2,y2=self.kmeasurement[i+1]
                vel.append(x2-x1)
                vel.append(y2-y1)

            b=len(vel)/2 
            for a in range(len(vel)):
                  if a%2==0:
                    vx+=vel[a]

                  else:
                       vy+=vel[a]
            
            return(vx/b,vy/b)
        

        
        def is_stationary(self):
            vx,vy=self.avg_vel()

            speed = (vx**2 + vy**2)**0.5

            if speed < 2:
                return (True)
            
        def is_straight(self):
                # Average velocity over the last few measurements
            vx1, vy1 = self.avg_vel()

            # Average velocity over the entire journey
            vx2, vy2 = self.avg_vel(999999)

            mag1 = np.hypot(vx1, vy1)
            mag2 = np.hypot(vx2, vy2)

            # Drone isn't moving
            if mag1 < 1e-6 or mag2 < 1e-6:
                return ("Not Straight")

            # Cosine similarity between the two velocity vectors
            similarity = (vx1 * vx2 + vy1 * vy2) / (mag1 * mag2)

            THRESHOLD = 0.98

            return (similarity >= THRESHOLD)
        

        def is_patrol(self):

            if len(self.sensor_measurements) < 6:
                return False

            TOL = 5

            reversals = 0

            for i in range(-len(self.sensor_measurements)+2, 0):

                x1,y1 = self.sensor_measurements[i-2]
                x2,y2 = self.sensor_measurements[i-1]
                x3,y3 = self.sensor_measurements[i]

                vx1 = x2-x1
                vx2 = x3-x2

                vy1 = y2-y1
                vy2 = y3-y2
            
                if vx1*vx2 < 0:
                    TOL = 5

                    matches_max = sum(abs(x2 - x) <= TOL for x in self.x_max_hits)
                    matches_min = sum(abs(x2 - x) <= TOL for x in self.x_min_hits)

                    if matches_max >= len(self.x_max_hits) * 0.7 or matches_min >= len(self.x_min_hits) * 0.7:   # 70% agree
                        
                        reversals += 1

                    if vx1 > 0:
                        # hit right wall

                        if not self.x_max_flag:
                            self.x_max_hits.append(x2)
                            self.x_max_flag = True

                        self.x_min_flag = False

                    else:
                        # hit left wall

                        if not self.x_min_flag:
                            self.x_min_hits.append(x2)
                            self.x_min_flag = True

                        self.x_max_flag = False

                if vy1*vy2 < 0:

                    matches_max = sum(abs(y2 - y) <= TOL for y in self.y_max_hits)
                    matches_min = sum(abs(y2 - y) <= TOL for y in self.y_min_hits)


                    if matches_max >= len(self.y_max_hits) * 0.7 or matches_min >= len(self.y_min_hits) * 0.7:   # 70% agree
                        
                        reversals += 1
                    

                    if vy1 > 0:
                        # hit top wall

                        if not self.y_max_flag:
                            self.y_max_hits.append(y2)
                            self.y_max_flag = True

                        self.y_min_flag = False

                    else:
                        # hit bottom wall

                        if not self.y_min_flag:
                            self.y_min_hits.append(y2)
                            self.y_min_flag = True

                        self.y_max_flag = False

            if reversals >= 8:
                return True

            return False
        
        def patrol_bounds(self):

            if not self.is_patrol() or self.get_state()!=PATROL:
                return None

            bounds = {}

            if self.x_min_hits:
                bounds["xmin"] = sum(self.x_min_hits)/len(self.x_min_hits)

            if self.x_max_hits:
                bounds["xmax"] = sum(self.x_max_hits)/len(self.x_max_hits)

            if self.y_min_hits:
                bounds["ymin"] = sum(self.y_min_hits)/len(self.y_min_hits)

            if self.y_max_hits:
                bounds["ymax"] = sum(self.y_max_hits)/len(self.y_max_hits)

            return bounds
                

        def get_state(self):
            if self.is_stationary():
                self.state=1
                return ("Stationary")
            

            if self.is_patrol():
                self.state=2
                return PATROL
            

            if self.is_straight():
                self.state=3
                return STRAIGHT

            

            

            return ("Unknown")
                    
        def predict_path(self,num):
            

            if self.state==0:
                self.get_state()

            
            if self.state==1:
                return(self.predict_stationary(num))
            
            elif self.state==2:
                return(self.predict_patrol(num))

            elif self.state==3:
                return(self.predict_straight_path(num))
            
           

            

            
            
            else:
                return("Unknown")

        def predict_straight_path(self,num):
            path=[]
            vx,vy=self.avg_vel()
            
            x=self.x[0,0]
            y=self.x[1,0]

            for _ in range(num):
                 x+=vx
                 y+=vy

                 path.append((float(x),float(y)))

            return(path)   


        def predict_stationary(self,num):
            path=[]

            for i in range(num):
                path.append((float(self.x[0,0]),float(self.x[1,0])))
            
            return (path)
        
        def predict_patrol(self,num):

            path=[]

            bounds=self.patrol_bounds()

            if bounds is None:
                return("Not Patrol")

            x,y=self.sensor_measurements[-1]
            
            vx,vy=self.get_velocity_from_sensor()
            
            TOL=2
            for i in range (num):
                
                
                if x>=bounds["xmax"]-TOL: 
                    vx*=-1

                if y>=bounds["ymax"]-TOL:
                    vy*=-1
                
                x+=vx
                y+=vy
                path.append((float(x),float(y)))

            return(path)
            
        def get_velocity_from_sensor(self):
            
            velx=[]
            vely=[]
            if len(self.sensor_measurements)<6:
                return(0,0)
            
            for i in range(1,5):
                x1,y1=self.sensor_measurements[-i]
                x2,y2=self.sensor_measurements[-i-1]
                
                if x2>x1:
                    velx.append(x2-x1)
                else:
                    velx.append(x1-x2)

                if y2>y1:
                    vely.append(y2-y1)
                else:
                    vely.append(y1-y2)

            xlen=len(velx)
            ylen=len(vely)
            xtot=sum(velx)
            ytot=sum(vely)

            return((xtot/xlen,ytot/ylen))




            


            
             
            
             

             
                  

                    
                    
                       
                       


                  
             

             
             

            
             
                 





