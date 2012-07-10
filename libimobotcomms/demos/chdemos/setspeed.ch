/* Filename: setspeed.ch 
  Move the two wheeled mobot with different speed. */
#include <mobot.h>
#include <math.h>
CMobot mobot;

/* Connect to the paired Mobot */
mobot.connect();

/* Set the mobot to "home" position, where all joint angles are 0 degrees. */
mobot.resetToZero();

double speed, radius;
mobot.getJointMaxSpeed(MOBOT_JOINT1, speed);
printf("The maximum speed is %lf degrees/s\n", speed);

mobot.setJointSpeed(MOBOT_JOINT1, 60);
mobot.setJointSpeed(MOBOT_JOINT4, 60);

//mobot.setJointSpeedRatio(MOBOT_JOINT1, 0.5);
//mobot.setJointSpeedRatio(MOBOT_JOINT4, 0.5);

//mobot.setJointSpeeds(60, 0, 0, 60);

printf("Roll forward 360 degrees.\n");
mobot.motionRollForward(360);

speed = 1.83; // = (3.5/2) * M_PI * 60/180 (inch/s)
radius = 3.5/2;             // radius is 1.75 
mobot.setTwoWheelRobotSpeed(speed, radius);

printf("Move 360 degrees.\n");
mobot.move(360, 0, 0, 360);

/* move at 1.83inch/sec with the radius 3.5 inches for 3 seconds */
printf("Move continuously for 3 seconds.\n");
mobot.moveContinuousTime(MOBOT_FORWARD, MOBOT_HOLD, MOBOT_HOLD, MOBOT_FORWARD, 3);
