from barobo.linkbot import *

linkbot_6PV8 = LinkBot()
linkbot_6PV8.connect()

linkbot_6PV8.recordAnglesBegin()
linkbot_6PV8.move(90, 90, 90)
linkbot_6PV8.move(-90, -90, -90)
linkbot_6PV8.recordAnglesEnd()
linkbot_6PV8.recordAnglesPlot()
