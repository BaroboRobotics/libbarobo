from mobot import *

robot = CMobot()
robot.connect()
angle1 = 0
angle2 = 0
angle3 = 0
angle4 = 0
robot.getJointAngles(angle1, angle2, angle3, angle4)
print "The robot joint angles are %lf %lf %lf %lf" % angle1, angle2, angle3, angle4
