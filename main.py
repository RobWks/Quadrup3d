__author__ = " Robert Weeks <robertweeks94@gmail.com>"

from collections import OrderedDict
import numpy as np
import math
import serial
import time
from itertools import cycle

class quadrup3d:
  def __init__(self,properties):
    """
    Initialize the tool quadrup3d object.
    Parameters
    ----------
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
    while True:
        #height = int(raw_input('Position: ')) 
        height = int(raw_input('Move in x: '))
        self.positionLeg(leg=0,x=100,y=100+height,z=self.properties['initial_z'])
        self.positionLeg(leg=1,x=100,y=-100+height,z=self.properties['initial_z'])
        self.positionLeg(leg=2,x=-100,y=-100+height,z=self.properties['initial_z'])
        self.positionLeg(leg=3,x=-100,y=100+height,z=self.properties['initial_z'])
        #time.sleep(1)

  def positionLeg(self,leg, x, y, z):
    """
    Position leg to 3D location in space relative
    to the base of the leg.
    """

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


    ser = serial.Serial('COM3', 115200)
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
    ('initial_x', 100),
    ('initial_y', 100),
    ('initial_z', -80),
])

if __name__ == '__main__':
    import logging
    
    logger = logging.getLogger(__name__)
 
    quadrup3d = quadrup3d(ROBOT_PROPERTIES)
    quadrup3d.run()