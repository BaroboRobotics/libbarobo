import signal
import sys
from mobot import *

def sigint_handler(signal, frame):
  sys.exit(0)

signal.signal(signal.SIGINT, sigint_handler)

robot1 = CMobot()
robot2 = CMobot()
robot1.connect()
robot2.connect()

while 1:
  ret,angle1,angle2,angle3,angle4 = robot1.getJointAngles()
  robot2.moveToPIDNB(angle1, angle2, angle3, angle4)
