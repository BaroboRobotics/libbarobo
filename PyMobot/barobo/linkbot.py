import math
import threading
import time
import pylab
#from mobot import *
import mobot

def deg2rad(deg):
  return deg * math.pi / 180.0

def rad2deg(deg):
  return deg * 180.0 / math.pi

class Linkbot():
  """
  Each instance of the Linkbot class represents a physical Linkbot. This class
  is used to control and get data from a Linkbot.
  """
  def __init__(self):
    self._mobot = mobot.mobot_t()
    mobot.Mobot_init(self._mobot)

  def connect(self):
    """Connect to a Linkbot
  
    Use this function to control Linkbots that are currently 
    connected in BaroboLink. """
    if mobot.Mobot_connect(self._mobot) != 0:
      raise RuntimeError("Error connecting to robot.")

  def connectWithBluetoothAddress(self, address, channel=1):
    """Connect to a Bluetooth-enabled Linkbot"""
    if mobot.Mobot_connectWithBluetoothAddress(self._mobot, address, channel) != 0:
      raise RuntimeError("Error connecting to robot.")

  def disconnect(self):
    """Disconnect from a Linkbot"""
    mobot.Mobot_disconnect(self._mobot)

  def getBreakoutADC(self, adc):
    """Return ADC reading from the breakout board."""
    rc, value = mobot.Mobot_getBreakoutADC(self._mobot, adc)
    if rc:
      raise RuntimeError("Error communicating with robot. Return code {0}".format(rc))
    return value
  
  def getJointAngle(self, joint):
    rc, value = mobot.Mobot_getJointAngle(self._mobot, joint)
    if rc:
      raise RuntimeError("Error communicating with robot. Return code {0}".format(rc))
    return rad2deg(value)

  def getJointAngles(self):
    """Return a list of joint angles in degrees"""
    r = mobot.Mobot_getJointAngles(self._mobot)
    if r[0] != 0:
      raise RuntimeError("Error communicating with robot.")
    else:
      return map(lambda x: rad2deg(x), r[1:4])

  def getJointAnglesTime(self):
    """Returns a list: [millis, deg1, deg2, deg3]"""
    r = mobot.Mobot_getJointAnglesTime(self._mobot)
    if r[0] != 0:
      raise RuntimeError("Error communicating with robot.")
    else:
      ret = [r[1]]
      ret.extend( map(lambda x: rad2deg(x), r[2:5]) )
      return ret

  def driveJointTo(self, joint, angle):
    """Drive joint 'joint' to 'angle' (degrees) using PID controller"""
    rc = mobot.Mobot_driveJointTo(self._mobot, joint, deg2rad(angle))
    if rc < 0:
      raise RuntimeError("Error communicating with robot. Return code {0}".format(rc))

  def driveJointToNB(self, joint, angle):
    """Drive joint 'joint' to 'angle' (degrees) using PID controller"""
    rc = mobot.Mobot_driveJointToNB(self._mobot, joint, deg2rad(angle))
    if rc < 0:
      raise RuntimeError("Error communicating with robot. Return code {0}".format(rc))

  def isMoving(self):
    """Return True if moving."""
    return mobot.Mobot_isMoving(self._mobot)

  def moveJoint(self, joint, angle):
    """Move a joint from its current location."""
    rc = mobot.Mobot_moveJoint(self._mobot, joint, deg2rad(angle))
    if rc < 0:
      raise RuntimeError("Error communicating with robot. Return code {0}".format(rc))

  def moveJointNB(self, joint, angle):
    """Move a joint from its current location."""
    rc = mobot.Mobot_moveJointNB(self._mobot, joint, deg2rad(angle))
    if rc < 0:
      raise RuntimeError("Error communicating with robot. Return code {0}".format(rc))

  def moveJointTo(self, joint, angle):
    """Move a joint to 'angle'."""
    rc = mobot.Mobot_moveJointTo(self._mobot, joint, deg2rad(angle))
    if rc < 0:
      raise RuntimeError("Error communicating with robot. Return code {0}".format(rc))

  def moveJointToNB(self, joint, angle):
    """Move a joint to 'angle'."""
    rc = mobot.Mobot_moveJointToNB(self._mobot, joint, deg2rad(angle))
    if rc < 0:
      raise RuntimeError("Error communicating with robot. Return code {0}".format(rc))

  def move(self, angle1, angle2, angle3):
    """Move the joints on a Linkbot
      
    Move joints from current location by amount specified in arguments in
    degrees. If the joint is not a movable joint (e.g. Joint 2 on a
    Linkbot-I), the argument is ignored."""
    rc = self.moveNB(angle1, angle2, angle3)
    if rc < 0:
      raise RuntimeError("Error communicating with robot. Return code {0}".format(rc))
    rc = self.moveWait()
    if rc < 0:
      raise RuntimeError("Error communicating with robot. Return code {0}".format(rc))

  def moveNB(self, angle1, angle2, angle3):
    """Nonblocking version ofthe move() function"""
    rc = mobot.Mobot_moveNB(self._mobot, deg2rad(angle1), deg2rad(angle2), deg2rad(angle3), 0)
    if rc < 0:
      raise RuntimeError("Error communicating with robot. Return code {0}".format(rc))

  def moveTo(self, angle1, angle2, angle3):
    """Move the joints on a Linkbot

    Move the joints on a Linkbot to absolute positions."""
    rc = self.moveToNB(angle1, angle2, angle3)
    if rc < 0:
      raise RuntimeError("Error communicating with robot. Return code {0}".format(rc))
    rc = self.moveWait()
    if rc < 0:
      raise RuntimeError("Error communicating with robot. Return code {0}".format(rc))

  def moveToNB(self, angle1, angle2, angle3):
    """Nonblocking version of the moveTo() function"""
    rc = mobot.Mobot_moveToNB(self._mobot, 
        deg2rad(angle1),
        deg2rad(angle2),
        deg2rad(angle3), 
        0)
    if rc < 0:
      raise RuntimeError("Error communicating with robot. Return code {0}".format(rc))

  def moveWait(self):
    """Wait until a non-blocking movement function is finished moving"""
    while mobot.Mobot_isMoving(self._mobot):
      time.sleep(0.5)

  def recordAnglesBegin(self, delay=0.05):
    """Begin recording joint angles.

    Keyword arguments:
    delay -- Seconds to delay between recorded datapoints. 0 indicates no delay (as fast as possible)
    """
    self.recordThread = _LinkbotRecordThread(self, delay)
    self.recordThread.start()

  def recordAnglesEnd(self):
    """ End recording angles and return a list consisting of [time_values,
    joint1angles, joint2angles, joint3angles]"""
    self.recordThread.runflag_lock.acquire()
    self.recordThread.runflag = False
    self.recordThread.runflag_lock.release()
    # Wait for recording to end
    while self.recordThread.isRunning:
      time.sleep(0.5)
    return [map(lambda x: x-self.recordThread.time[0], self.recordThread.time), 
        self.recordThread.angles[0], 
        self.recordThread.angles[1], 
        self.recordThread.angles[2]]

  def recordAnglesPlot(self):
    """Plot recorded angles.

    See recordAnglesBegin() and recordAnglesEnd() to record joint motions.
    """
    pylab.plot(
        self.recordThread.time, 
        self.recordThread.angles[0],
        self.recordThread.time, 
        self.recordThread.angles[2])
    pylab.show()

  def reset(self):
    rc = mobot.Mobot_reset(self._mobot)
    if rc < 0:
      raise RuntimeError("Error communicating with robot. Return code {0}".format(rc))

  def resetToZero(self):
    """Reset the Linkbot to its zero positions."""
    rc = mobot.Mobot_resetToZero(self._mobot)
    if rc < 0:
      raise RuntimeError("Error communicating with robot. Return code {0}".format(rc))

  def resetToZeroNB(self):
    """Reset the Linkbot to its zero positions."""
    rc = mobot.Mobot_resetToZeroNB(self._mobot)
    if rc < 0:
      raise RuntimeError("Error communicating with robot. Return code {0}".format(rc))

  def setBuzzerFrequency(self, frequency):
    """Sets the buzzer to buz at a frequency specified in Hz.

    Set frequency to 0 to silence the buzzer"""
    rc = mobot.Mobot_setBuzzerFrequencyOn(self._mobot, int(frequency))
    if rc < 0:
      raise RuntimeError("Error communicating with robot. Return code {0}".format(rc))

  def setColorRGB(self, r, g, b):
    """Set the multicolor LED on a linkbot.

    Arguments should be in the range [0, 255].
    """
    rc = mobot.Mobot_setColorRGB(self._mobot, int(r), int(g), int(b))
    if rc < 0:
      raise RuntimeError("Error communicating with robot. Return code {0}".format(rc))

  def setJointSpeed(self, joint, speed):
    """Set the constant-velocity speed of a joint to "speed", in deg/sec."""
    rc = mobot.Mobot_setJointSpeed(self._mobot, int(joint), float(deg2rad(speed)))
    if rc < 0:
      raise RuntimeError("Error communicating with robot. Return code {0}".format(rc))

  def setMotorPower(self, joint, power):
    """Set the power of a joint.

    Power is a value in the range [0, 255]
    """
    rc = mobot.Mobot_setMotorPower(self._mobot, int(joint), int(power))
    if rc < 0:
      raise RuntimeError("Error communicating with robot. Return code {0}".format(rc))

  def setMovementState(self, state1, state2, state3):
    """Set the movement state for the 3 joints of a Linkbot.

    Valid states are ROBOT_NEUTRAL, ROBOT_BACKWARD, ROBOT_FORWARD, and ROBOT_HOLD
    """
    rc = mobot.Mobot_setMovementStateNB(
        self._mobot,
        state1,
        state2,
        state3,
        0)
    if rc < 0:
      raise RuntimeError("Error communicating with robot. Return code {0}".format(rc))
   
  def stop(self):
    """Stop all motors."""
    rc = mobot.Mobot_stop(self._mobot) 
    if rc < 0:
      raise RuntimeError("Error communicating with robot. Return code {0}".format(rc))

    
class _LinkbotRecordThread(threading.Thread):
  def __init__(self, linkbot, delay):
    self.delay = delay
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
      time.sleep(self.delay)
    self.isRunning = False
