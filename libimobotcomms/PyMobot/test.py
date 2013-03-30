import signal
import sys
from mobot import *

def sigint_handler(signal, frame):
  sys.exit(0)

signal.signal(signal.SIGINT, sigint_handler)

robot1 = CMobotI()
robot1.connect()

robot1.move(90, 90, 90)
