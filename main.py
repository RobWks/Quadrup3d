__author__ = " Robert Weeks <robertweeks94@gmail.com>"

from collections import OrderedDict
import numpy as np
import math
import serial
import time

class quadrup3d:
    def __init__(self,properties):
        """
        Initialize the tool quadrup3d object.
        Parameters
        ----------s
        """ 
        #Global variables for translation and rotation functions
        xold = 0
        yold = 0
        zold = 0
        rollold = 0                       
        pitchold = 0
        yawold = 0

        self.properties = properties

        #Initialize leg positions
        #for i in range(4):   
          #self.positionLeg(i,properties['initial_x'],properties['initial_y'],properties['initial_z'])

    def run(self):
        """
        while True:
    	dif = float(raw_input('dif: '))
    	self.positionLeg(leg=0,x=0,y=0,z=dif)
            self.positionLeg(leg=1,x=0,y=0,z=dif)
            self.positionLeg(leg=2,x=0,y=0,z=dif)
            self.positionLeg(leg=3,x=0,y=0,z=dif)
            time.sleep(1)
        """
        for leg in range(4):
            self.positionLeg(leg=leg,x=0,y=0,z=0)
        self.positionLeg(leg=1,x=0,y=0,z=60)
	#time.sleep(5)
        #self.walk(5)

    def walk(self,num_cycles):
        pi = math.pi

        S = 70 #Step length in mm
        H = 60 #Step height in mm
        T = 2 #Gait cycle in seconds
        R = 0.05 #Resolution of gait cycle in seconds

        n = T/R+1 #Calculates number of steps to achieve resolution, must be int
        t = np.linspace(0,T,n)

        leg_x = 0 #Does not move in x direction
        leg_y = S*t/T-(S/(2*pi))*np.sin(2*pi*t/T) #Movement for t in [0,T/2]
        leg_z1 = 2*H*t/T - (H/(2*pi))*np.sin(4*pi*t/T) #Movement for t in [0,T/4]
        leg_z2 = 2*H - 2*H*t/T + (H/(2*pi))*np.sin(4*pi*t/T) #Movement in [T/4,T/2]
        leg_z1 = leg_z1[:round(len(t)/2)+1]
        leg_z2 = leg_z2[round(len(t)/2)+1:]
        leg_z = np.hstack((leg_z1,leg_z2))

        body_x = 0 #Does not move in x direction
        body_y = S*t/(2*T)-(S/(4*pi))*np.sin(2*pi*t/T) #Movement for t in [0,T/2]
        body_z = 0 #Does not move in z direction

        for _ in range(num_cycles):
            for leg in [1,0]: #move legs 1 then 0
                for i in range(int(n)):
                    self.positionLeg(leg=leg,x=0,y=leg_y[i],z=leg_z[i])
                    #time.sleep(R)

            #move body forward half motion
            for i in range(int(n)):
                    self.positionLeg(leg=0,x=0,y=-body_y[i]+S,z=0)
                    self.positionLeg(leg=1,x=0,y=-body_y[i]+S,z=0)
                    self.positionLeg(leg=2,x=0,y=-body_y[i],z=0)
                    self.positionLeg(leg=3,x=0,y=-body_y[i],z=0)
                    #time.sleep(R)

            for leg in [2,3]: #move legs 2 then 3
                for i in range(int(n)):
                    self.positionLeg(leg=leg,x=0,y=leg_y[i]-S/2,z=leg_z[i])
                    #time.sleep(R)
            
            #move body forward half motion
            for i in range(int(n)):
                    self.positionLeg(leg=0,x=0,y=-body_y[i]+S/2,z=0)
                    self.positionLeg(leg=1,x=0,y=-body_y[i]+S/2,z=0)
                    self.positionLeg(leg=2,x=0,y=-body_y[i]+S/2,z=0)
                    self.positionLeg(leg=3,x=0,y=-body_y[i]+S/2,z=0)
                    #time.sleep(R)

    def positionLeg(self,leg, x, y, z):
        """
        Position leg to 3D location in space relative
        to the base of the leg.
        """
        #Apply Zero positions
        p = self.properties

        z+=p['initial_z']

        if leg==0 or leg ==1: x+=p['initial_x'] 
        else: x-=p['initial_x'] 

        if leg==0 or leg==3: y+=p['initial_y'] 
        else: y-=p['initial_y'] 

        #To place in start postion for gait
        if leg==0 or leg ==1: y-=100

        #Local to global coordinate transform
        #Swap X and Y
        temp = x
        xt = y
        yt = temp
        z=-z

        rotAngle = leg*math.pi/2+math.pi/4
        x = xt*math.cos(rotAngle)+yt*math.sin(rotAngle)
        y = -xt*math.sin(rotAngle)+yt*math.cos(rotAngle)
        print x
        print y


        coxaLength = self.properties['coxaLength']
        tibiaLength = self.properties['tibiaLength']
        femurLength = self.properties['femurLength']

        
        legLength = math.sqrt(x**2+y**2)
        hf = math.sqrt((legLength-coxaLength)**2+z**2) 

        a1 = math.acos(z/hf)
        a2 = math.acos((tibiaLength**2-femurLength**2-hf**2)/(-2*femurLength*hf))
        a = a1+a2
        b1 = math.acos((hf**2-tibiaLength**2-femurLength**2)/(-2*femurLength*tibiaLength))
        c1 = math.atan2(y,x)

        fAngle = self.clamp(a-math.pi/2,-math.pi/2,math.pi/2)
        tAngle = self.clamp(b1-math.pi/2,-math.pi/2,math.pi/2)
        cAngle = self.clamp(c1,-math.pi/2,math.pi/2)

        print 'Coxa'+str(leg)+' Angle: ' + str(round(math.degrees(cAngle)))
        print 'femur'+str(leg)+' Angle: ' + str(round(math.degrees(fAngle)))
        print 'tibia'+str(leg)+' Angle: ' + str(round(math.degrees(cAngle)))

        fAngleSerial = int(round(self.map(fAngle,-math.pi/2,math.pi/2,500,2500)))
        tAngleSerial = int(round(self.map(tAngle,-math.pi/2,math.pi/2,500,2500)))
        cAngleSerial = int(round(self.map(cAngle,-math.pi/2,math.pi/2,500,2500)))


        ser = serial.Serial("/dev/ttyAMA0", 115200)
        legPinout = [12,21,0,9]
        speed = 100
        command = "#{} P{} #{} P{} #{} P{}\r".format(legPinout[leg],cAngleSerial,legPinout[leg]+1,fAngleSerial,legPinout[leg]+2,tAngleSerial)
        print command
        ser.write(command)

    def update(self):
        for i in range(4):
          self.positionLeg(i,currentLoc[0][i],currentLoc[1][i],currentLoc[2][i])

    def clamp(self,n, minn, maxn):
        if n < minn:
          return minn
        elif n > maxn:
          return maxn
        else:
          return n

    def map(self,oldValue,oldMin,oldMax,newMin,newMax):
        oldRange = (oldMax - oldMin)  
        newRange = (newMax - newMin)  
        newValue = (((oldValue - oldMin) * newRange) / oldRange) + newMin
        return newValue

ROBOT_PROPERTIES = OrderedDict([
    ('femurLength', 66.0),
    ('tibiaLength', 140.0),
    ('coxaLength', 46.0),
    ('speed', 16),
    ('initial_x', 160),
    ('initial_y', 80),
    ('initial_z', -100),
])

if __name__ == '__main__':
    import logging
    
    logger = logging.getLogger(__name__)
 
    quadrup3d = quadrup3d(ROBOT_PROPERTIES)
    quadrup3d.run()
