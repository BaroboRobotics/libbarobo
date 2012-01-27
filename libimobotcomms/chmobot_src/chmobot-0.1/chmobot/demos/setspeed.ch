/* Filename: setspeed.ch 
  Move the two wheeled robot with different speed. */

#include <mobot.h>
#include <math.h>

CMobot robot;

/* Connect to the paired MoBot */
robot.connect();

/* Set the robot to "home" position, where all joint angles are 0 degrees. */
robot.moveToZero();

double speed, radius;
robot.getJointMaxSpeed(ROBOT_JOINT1, speed);
printf("The maximum speed is %lf degrees/s\n", rad2deg(speed));

robot.setJointSpeed(ROBOT_JOINT1, deg2rad(90));
robot.setJointSpeed(ROBOT_JOINT4, deg2rad(90));

//robot.setJointSpeedRatio(ROBOT_JOINT1, 0.5);
//robot.setJointSpeedRatio(ROBOT_JOINT4, 0.5);

//robot.setJointSpeeds(deg2rad(90), 0, 0, deg2rad(90));

printf("Roll forward 360 degrees.\n");
robot.motionRollForward(deg2rad(360));

speed = (3.5/2) * M_PI / 2;      // 2.75 inch/s 
radius = 3.5/2;     // radius is 1.75 
robot.setTwoWheelRobotSpeed(speed, radius);

printf("Move 360 degrees.\n");
robot.move(deg2rad(360), 0, 0, deg2rad(360));

/* move at 2.75inch/sec with the radius 3.5 inches for 3 seconds */
printf("Move continuously for 3 seconds.\n");
robot.moveContinuousTime(ROBOT_FORWARD, ROBOT_HOLD, 
                         ROBOT_HOLD, ROBOT_FORWARD, 3000);
