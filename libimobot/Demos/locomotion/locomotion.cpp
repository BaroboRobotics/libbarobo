#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <imobot.h>

void move_forward(CiMobot* robot)
{
  robot->moveJoint(2, -90);
  robot->moveJoint(3, 90);
}

void move_backward(CiMobot* robot)
{
  robot->moveJoint(2, 90);
  robot->moveJoint(3, -90);
}

void arch(CiMobot* robot)
{
  robot->poseJoint(0, -30);
  robot->poseJoint(1, 30);
}

void inch_right(CiMobot* robot)
{
  robot->poseJoint(0, -50);
  robot->waitMotor(0);
  robot->poseJoint(1, 50);
  robot->waitMotor(1);
  robot->poseJoint(0, 0);
  robot->waitMotor(0);
  robot->poseJoint(1, 0);
  robot->waitMotor(1);
}

void inch_left(CiMobot* robot)
{
  robot->poseJoint(1, 50);
  robot->waitMotor(1);
  robot->poseJoint(0, -50);
  robot->waitMotor(0);
  robot->poseJoint(1, 0);
  robot->waitMotor(1);
  robot->poseJoint(0, 0);
  robot->waitMotor(0);
}

void rotate_right(CiMobot* robot)
{
  robot->moveJoint(2, 90);
  robot->moveJoint(3, 90);
}

void rotate_left(CiMobot* robot)
{
  robot->moveJoint(2, -90);
  robot->moveJoint(3, -90);
}

void stand(CiMobot* robot)
{
  int i;
  int speed;
  /* Go to home position first */
  for(i = 0; i < 4; i++) {
    robot->poseJoint(i, 0);
  }
  robot->moveWait();
  printf("Home.\n");
  sleep(2);

  /* Arch the robot */
  robot->poseJoint(0, -85);
  robot->poseJoint(1, 80);
  robot->moveWait();

  /* Twist the bottom face */
  robot->poseJoint(2, 45);
  robot->waitMotor(2);

  /* Stand the robot up slowly */
  /* First, save the speed */
  robot->getMotorSpeed(0, speed);
  robot->setMotorSpeed(0, 25);
  robot->poseJoint(0, 20);
  robot->waitMotor(0);
  /* Reset the old speed */
  robot->setMotorSpeed(0, speed);
}

int main()
{
  int i;
  CiMobot robot;

  /* Set the robot to "home" position, where all joint angles are 0 degrees. */
  for(i = 0; i < 4; i++) {
    robot.poseJoint(i, 0);
  }
  robot.moveWait();
  sleep(2);
  stand(&robot);
  sleep(2);
  return 0;
  /* Rotate each of the faceplates by 90 degrees */
  robot.poseJoint(2, 90);
  robot.poseJoint(3, 90);
  /* Wait for the movement to complete */
  robot.waitMotor(2);
  robot.waitMotor(3);
  /* Move the motors back to where they were */
  robot.poseJoint(2, 0);
  robot.poseJoint(3, 0);
  robot.waitMotor(2);
  robot.waitMotor(3);

  /* Stand the robot up */
  stand(&robot);

  /* Terminate control of the robot */
  robot.terminate();

  return 0;
}
