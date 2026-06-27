STRAIGHT = "Straight"
PATROL = "Patrol"
CIRCLE = "Circle"
INTRUDER="Intruder"
UNKNOWN="Unknown"


class Drone:
    def __init__(self,drone_id,mode,x,y,vx,vy,x_leftlim=0,x_rightlim=0,y_leftlim=0,y_rightlim=0,target_x=None,target_y=None):
        self.id=drone_id
        self.x=x
        self.y=y
        self.vx=vx 
        self.vy=vy
        self.history=[(self.x,self.y)]
        self.measurement=[]
        self.mode=mode
        self.x_leftlim=x_leftlim
        self.x_rightlim=x_rightlim
        self.y_leftlim=y_leftlim
        self.y_rightlim=y_rightlim
        self.target_x=target_x
        self.target_y=target_y
        
       
    
    def move(self):
        if self.mode==STRAIGHT:
            self.x+=self.vx
            self.y+=self.vy
        
        elif self.mode==PATROL:
            if self.x >= self.x_rightlim and self.vx > 0:
                self.vx *= -1

            if self.x <= self.x_leftlim and self.vx < 0:
                self.vx *= -1

            if self.y >= self.y_rightlim and self.vy > 0:
                self.vy *= -1

            if self.y <= self.y_leftlim and self.vy < 0:
                self.vy *= -1

            self.x += self.vx
            self.y += self.vy
        
        elif self.mode==INTRUDER:
            speed=(self.vx**2+self.vy**2)**0.5
            dx=self.target_x - self.x
            dy=self.target_y - self.y
            distance=(dx**2+dy**2)**0.5

            if distance>0:
                V_x= (dx/distance)*speed
                V_y= (dy/distance)*speed

            if distance>speed:
                self.x+=V_x
                self.y+=V_y
            else:
                self.x=self.target_x
                self.y=self.target_y

        
        # elif self.mode=="Circle":




        self.history.append((self.x,self.y))
    
    def calc_velocity(self):
        if len(self.history)<2:
            return(0,0)
        else:
            x1,y1=self.history[-1]
            x2,y2=self.history[-2]

            vx=x1-x2
            vy=y1-y2

        return(vx,vy)

    def predict_next_pos(self):
        x,y = self.history[-1]
        vx,vy = self.calc_velocity()

        return (x+vx, y+vy)
    
    def predict_path(self,num):
        future=[]
        x1,y1=self.history[-1]
        vx1,vy1=self.calc_velocity()
        
        
        for _ in range(num):
            x1+=vx1
            y1+=vy1
            future.append((x1,y1))

        return future  

    # def measurement_error(self):

    #     true_x,true_y = self.history[-1]

    #     meas_x,meas_y = self.measurement[-1]

    #     return (
    #     meas_x-true_x,
    #     meas_y-true_y
    #     )  

    # def calc_measured_velocity(self):

    #     x1,y1=self.measurement[-1]
    #     x2,y2=self.measurement[-2]

    #     return(x1-x2,y1-y2)
    
    # def measured_prediction(self,num):
    #     p_future=[]
        
    #     if self.mode==STRAIGHT:
    #         x1,y1=self.measurement[-1]
    #         vx1,vy1=self.calc_avg_velocity()
            
            
    #         for _ in range(num):
    #             x1+=vx1
    #             y1+=vy1
    #             p_future.append((x1,y1))

        


    #     return p_future  
    
    
    def calc_avg_velocity(self, window=5):

        n = min(window, len(self.measurement))

        if n < 2:
            return (0, 0)

        x1, y1 = self.measurement[-n]
        x2, y2 = self.measurement[-1]

        vx = (x2 - x1) / (n - 1)
        vy = (y2 - y1) / (n - 1)

        return (vx, vy)



