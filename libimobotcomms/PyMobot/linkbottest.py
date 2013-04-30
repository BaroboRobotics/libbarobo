from linkbot import *
from pylab import *
import time

mybot = LinkBot()
mybot.connect()
mybot.recordAnglesBegin()
mybot.move(90, 90, 90)
mybot.move(-90, -90, -90)
mybot.recordAnglesEnd()
mybot.recordAnglesPlot()
