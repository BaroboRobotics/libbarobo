#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <imobot.h>

void moveJoint(CiMobot &robot, iMobotJointId_t id, double angle)
{
  double cur_angle;
  robot.getJointAngle(id, cur_angle);
  robot.moveJointTo(id, cur_angle + angle);
}

void move_forward(CiMobot &robot)
{
  moveJoint(robot, IMOBOT_JOINT1, -90);
  moveJoint(robot, IMOBOT_JOINT4, 90);
}

void move_backward(CiMobot &robot)
{
  moveJoint(robot, IMOBOT_JOINT1, 90);
  moveJoint(robot, IMOBOT_JOINT4, -90);
}

void arch(CiMobot &robot)
{
  robot.moveJointTo(IMOBOT_JOINT2, -30);
  robot.moveJointTo(IMOBOT_JOINT3, 30);
}

void inch_right(CiMobot &robot)
{
  robot.moveJointTo(IMOBOT_JOINT2, -50);
  robot.moveJointTo(IMOBOT_JOINT3, 50);
  robot.moveJointTo(IMOBOT_JOINT2, 0);
  robot.moveJointTo(IMOBOT_JOINT3, 0);
}

void inch_left(CiMobot &robot)
{
  robot.moveJointTo(IMOBOT_JOINT3, 50);
  robot.moveJointTo(IMOBOT_JOINT2, -50);
  robot.moveJointTo(IMOBOT_JOINT3, 0);
  robot.moveJointTo(IMOBOT_JOINT2, 0);
}

void rotate_right(CiMobot &robot)
{
  moveJoint(robot, IMOBOT_JOINT1, 90);
  moveJoint(robot, IMOBOT_JOINT4, 90);
}

void rotate_left(CiMobot &robot)
{
  moveJoint(robot, IMOBOT_JOINT1, -90);
  moveJoint(robot, IMOBOT_JOINT4, -90);
}

void stand(CiMobot &robot)
{
  int i;
  double speed;
  /* Go to home position first */
  robot.moveToZero();
  sleep(1);

  /* Arch the robot */
  robot.moveJointToNB(IMOBOT_JOINT2, -85);
  robot.moveJointToNB(IMOBOT_JOINT3, 80);
  robot.moveWait();

  /* Twist the bottom face */
  robot.moveJointTo(IMOBOT_JOINT1, 45);

  /* Stand the robot up slowly */
  /* First, save the speed */
  robot.getJointSpeed(IMOBOT_JOINT2, speed);
  robot.setJointSpeed(IMOBOT_JOINT2, 30);
  robot.moveJointTo(IMOBOT_JOINT2, 20);
  /* Reset the old speed */
  robot.setJointSpeed(IMOBOT_JOINT2, speed);
}

int main()
{
  int i;
  CiMobot robot;

  /* Set the robot to "home" position, where all joint angles are 0 degrees. */
  robot.moveToZero();
  robot.moveWait();

  /* Roll the robot forward */
  move_forward(robot);
  robot.moveWait();

  /* Rotate left */
  rotate_left(robot);
  robot.moveWait();

  /* Roll the robot backward */
  move_backward(robot);
  robot.moveWait();

  /* Rotate right */
  rotate_right(robot);
  robot.moveWait();

  /* Stand the robot up */
  stand(robot);
  robot.moveWait();

  return 0;
}


