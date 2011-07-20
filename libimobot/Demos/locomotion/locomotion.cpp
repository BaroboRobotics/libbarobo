#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <imobot.h>

void move_forward(CiMobot &robot)
{
  robot.moveJoint(2, -90);
  robot.moveJoint(3, 90);
}

void move_backward(CiMobot &robot)
{
  robot.moveJoint(2, 90);
  robot.moveJoint(3, -90);
}

void arch(CiMobot &robot)
{
  robot.poseJoint(0, -30);
  robot.poseJoint(1, 30);
}

void inch_right(CiMobot &robot)
{
  robot.poseJoint(0, -50);
  robot.waitMotor(0);
  robot.poseJoint(1, 50);
  robot.waitMotor(1);
  robot.poseJoint(0, 0);
  robot.waitMotor(0);
  robot.poseJoint(1, 0);
  robot.waitMotor(1);
}

void inch_left(CiMobot &robot)
{
  robot.poseJoint(1, 50);
  robot.waitMotor(1);
  robot.poseJoint(0, -50);
  robot.waitMotor(0);
  robot.poseJoint(1, 0);
  robot.waitMotor(1);
  robot.poseJoint(0, 0);
  robot.waitMotor(0);
}

void rotate_right(CiMobot &robot)
{
  robot.moveJoint(2, 90);
  robot.moveJoint(3, 90);
}

void rotate_left(CiMobot &robot)
{
  robot.moveJoint(2, -90);
  robot.moveJoint(3, -90);
}

void stand(CiMobot &robot)
{
  int i;
  int speed;
  /* Go to home position first */
  for(i = 0; i < 4; i++) {
    robot.poseJoint(i, 0);
  }
  robot.moveWait();
  sleep(2);

  /* Arch the robot */
  robot.poseJoint(0, -85);
  robot.poseJoint(1, 80);
  robot.moveWait();

  /* Twist the bottom face */
  robot.poseJoint(2, 45);
  robot.waitMotor(2);

  /* Stand the robot up slowly */
  /* First, save the speed */
  robot.getMotorSpeed(0, speed);
  robot.setMotorSpeed(0, 30);
  robot.poseJoint(0, 20);
  robot.waitMotor(0);
  /* Reset the old speed */
  robot.setMotorSpeed(0, speed);
}

int main()
{
  int i;
  CiMobot robot;

  /* Set the robot to "home" position, where all joint angles are 0 degrees. */
  robot.poseZero();
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
