from barobo.linkbot import *

l = Linkbot()
l.connectWithSerialID('2HNJ')
l.setJointSpeeds(120, 120, 120)

while True:
  l.move(180, 0, -180)
  l.move(-180, 0, 180)

