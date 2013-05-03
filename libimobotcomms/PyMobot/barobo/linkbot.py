import math
import threading
import time
import pylab
from mobot import *

def deg2rad(deg):
  return deg * math.pi / 180.0

def rad2deg(deg):
  return deg * 180.0 / math.pi

class LinkBot():
  def __init__(self):
    self._mobot = mobot_t()
    Mobot_init(self._mobot)

  def connect(self):
    Mobot_connect(self._mobot)

  def disconnect(self):
    Mobot_disconnect(self._mobot)

  def getJointAngles(self):
    r = Mobot_getJointAngles(self._mobot)
    if r[0] != 0:
      return -1
    else:
      return map(lambda x: rad2deg(x), r[1:4])

  def getJointAnglesTime(self):
    r = Mobot_getJointAnglesTime(self._mobot)
    if r[0] != 0:
      return None
    else:
      ret = [r[1]]
      ret.extend( map(lambda x: rad2deg(x), r[2:5]) )
      return ret

  def driveJointTo(self, joint, angle):
    """Drive joint 'joint' to 'angle' (degrees) using PID controller"""
    Mobot_driveJointTo(self._mobot, joint, deg2rad(angle))

  def driveJointToNB(self, joint, angle):
    Mobot_driveJointToNB(self._mobot, joint, deg2rad(angle))

  def move(self, angle1, angle2, angle3):
    self.moveNB(angle1, angle2, angle3)
    self.moveWait()

  def moveNB(self, angle1, angle2, angle3):
    Mobot_moveNB(self._mobot, deg2rad(angle1), deg2rad(angle2), deg2rad(angle3), 0)

  def moveTo(self, angle1, angle2, angle3):
    self.moveToNB(angle1, angle2, angle3)
    self.moveWait()

  def moveToNB(self, angle1, angle2, angle3):
    Mobot_moveToNB(self._mobot, 
        deg2rad(angle1),
        deg2rad(angle2),
        deg2rad(angle3), 
        0)

  def moveWait(self):
    while Mobot_isMoving(self._mobot):
      time.sleep(0.5)

  def recordAnglesBegin(self):
    self.recordThread = _LinkBotRecordThread(self)
    self.recordThread.start()

  def recordAnglesEnd(self):
    self.recordThread.runflag_lock.acquire()
    self.recordThread.runflag = False
    self.recordThread.runflag_lock.release()
    # Wait for recording to end
    while self.recordThread.isRunning:
      time.sleep(0.5)

  def recordAnglesPlot(self):
    pylab.plot(
        self.recordThread.time, 
        self.recordThread.angles[0],
        self.recordThread.time, 
        self.recordThread.angles[2])
    pylab.show()

  def setBuzzerFrequency(self, frequency):
    Mobot_setBuzzerFrequencyOn(self._mobot, int(frequency))

  def setColorRGB(self, r, g, b):
    Mobot_setColorRGB(self._mobot, int(r), int(g), int(b))

  def setJointSpeed(self, joint, speed):
    Mobot_setJointSpeed(self._mobot, int(joint), float(speed))

  def setMotorPower(self, joint, power):
    Mobot_setMotorPower(self._mobot, int(joint), int(power))
   
  def stop(self):
    Mobot_stop(self._mobot) 
    
class _LinkBotRecordThread(threading.Thread):
  def __init__(self, linkbot):
    self.linkbot = linkbot
    self.runflag = False
    self.isRunning = False;
    self.runflag_lock = threading.Lock()
    self.time = []
    self.angles = [ [], [], [] ]
    threading.Thread.__init__(self)

  def run(self):
    self.runflag = True
    self.isRunning = True
    while True:
      self.runflag_lock.acquire()
      if self.runflag == False:
        self.runflag_lock.release()
        break
      self.runflag_lock.release()
      # Get the joint angles and stick them into our data struct
      data = self.linkbot.getJointAnglesTime()
      self.time.append(data[0])
      self.angles[0].append(data[1])
      self.angles[1].append(data[2])
      self.angles[2].append(data[3])
      time.sleep(0.05)
    self.isRunning = False
