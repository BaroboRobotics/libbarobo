/* Filename: setspeed.ch 
  Move the two wheeled robot with different speed. */
#include <mobot.h>
#include <math.h>
CMobot robot;

/* Connect to the paired Mobot */
robot.connect();

/* Set the robot to "home" position, where all joint angles are 0 degrees. */
robot.moveToZero();

double speed, radius;
robot.getJointMaxSpeed(MOBOT_JOINT1, speed);
printf("The maximum speed is %lf degrees/s\n", speed);

robot.setJointSpeed(MOBOT_JOINT1, 60);
robot.setJointSpeed(MOBOT_JOINT4, 60);

//robot.setJointSpeedRatio(MOBOT_JOINT1, 0.5);
//robot.setJointSpeedRatio(MOBOT_JOINT4, 0.5);

//robot.setJointSpeeds(60, 0, 0, 60);

printf("Roll forward 360 degrees.\n");
robot.motionRollForward(360);

speed = 1.83; // = (3.5/2) * M_PI * 60/180 (inch/s)
radius = 3.5/2;             // radius is 1.75 
robot.setTwoWheelRobotSpeed(speed, radius);

printf("Move 360 degrees.\n");
robot.move(360, 0, 0, 360);

/* move at 1.83inch/sec with the radius 3.5 inches for 3 seconds */
printf("Move continuously for 3 seconds.\n");
robot.moveContinuousTime(MOBOT_FORWARD, MOBOT_HOLD, MOBOT_HOLD, MOBOT_FORWARD, 3);
