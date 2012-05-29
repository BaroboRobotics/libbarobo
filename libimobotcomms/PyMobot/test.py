from mobot import *

robot1 = CMobot()
robot2 = CMobot()
robot1.connect()
robot2.connect()
angle1 = 0
angle2 = 0
angle3 = 0
angle4 = 0
while 1:
  ret,angle1,angle2,angle3,angle4 = robot1.getJointAngles()
  robot2.moveToPIDNB(angle1, angle2, angle3, angle4)
