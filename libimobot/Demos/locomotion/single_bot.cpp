#include <iostream>
#include "imobot.h"

using namespace std;

int move_forward(CiMobot *robot)
{
  float m[IMOBOT_NUM_JOINTS];
  /* First, lets get the encoder counts for the 3rd and 4th motors, since we do not want
   * to move them for this gait. */
  m[IMOBOT_JOINT3] = robot->getJointAngle(IMOBOT_JOINT3);
  m[IMOBOT_JOINT4] = robot->getJointAngle(IMOBOT_JOINT4);
  
  m[IMOBOT_JOINT1] = 0;
  m[IMOBOT_JOINT2] = 0;
  robot->drive(m);
  robot->moveWait();
  m[IMOBOT_JOINT1] = 60;
  robot->drive(m); 
  robot->moveWait();
  m[IMOBOT_JOINT2] = -60;
  robot->drive(m); 
  robot->moveWait();
  m[IMOBOT_JOINT1] = 0;
  robot->drive(m);
  robot->moveWait();
  m[IMOBOT_JOINT2] = 0;
  robot->drive(m); 
  robot->moveWait();
  return 0;
}

int move_backward(CiMobot *robot)
{
  float m[IMOBOT_NUM_JOINTS];
  /* First, lets get the encoder counts for the 3rd and 4th motors, since we do not want
   * to move them for this gait. */
  m[IMOBOT_JOINT3] = robot->getJointAngle(IMOBOT_JOINT3);
  m[IMOBOT_JOINT4] = robot->getJointAngle(IMOBOT_JOINT4);

  m[IMOBOT_JOINT1] = 0;
  m[IMOBOT_JOINT2] = 0;
  robot->drive(m);
  robot->moveWait();
  m[IMOBOT_JOINT2] = -60;
  robot->drive(m);
  robot->moveWait();
  m[IMOBOT_JOINT1] = 60;
  robot->drive(m);
  robot->moveWait();
  m[IMOBOT_JOINT2] = 0;
  robot->drive(m);
  robot->moveWait();
  m[IMOBOT_JOINT1] = 0;
  robot->drive(m);
  robot->moveWait();
  return 0;
}

int move_right(CiMobot *robot)
{
  int m[IMOBOT_NUM_JOINTS] = {0, 0, 120, -120};
  robot->driveRelativeEncoder(m);
  robot->moveWait();
  return 0;
}

int move_left(CiMobot *robot)
{
  int m[IMOBOT_NUM_JOINTS] = {0, 0, -120, 120};
  robot->driveRelativeEncoder(m);
  robot->moveWait();
  return 0;
}

int turn_left(CiMobot *robot, int amount)
{
  float angle[4] = {0, 0, 90*amount, 90*amount};
  robot->driveRelative(angle);
  robot->moveWait();
  return 0;
}

int sweep_left(CiMobot *robot, int amount)
{
  float angle[4] = {0, 0, 0, 90*amount};
  robot->driveRelative(angle);
  robot->moveWait();
  return 0;
}

int stand(CiMobot *robot)
{
  float m[4];
  /* First, lets get the encoder counts for the 3rd and 4th motors */
  m[IMOBOT_JOINT3] = robot->getJointAngle(IMOBOT_JOINT3);
  m[IMOBOT_JOINT4] = robot->getJointAngle(IMOBOT_JOINT4);
  m[IMOBOT_JOINT1] = 0;
  m[IMOBOT_JOINT2] = 0;
  robot->drive(m);
  robot->moveWait();
  /* Get into fetal position */
  m[IMOBOT_JOINT1] = 90;
  m[IMOBOT_JOINT2] = -85;
  robot->drive(m);
  robot->moveWait();
  /* Twist endcap */
  m[IMOBOT_JOINT3] += 45;
  robot->drive(m);
  robot->moveWait();
  /* Lisf body slowly */
  robot->setJointPower(IMOBOT_JOINT1, 200);
  m[IMOBOT_JOINT1] = -20;
  robot->drive(m);
  robot->moveWait();
  robot->setJointPower(IMOBOT_JOINT1, 254);
  return 0;
}

int main()
{
  CiMobot robot;
  int i;
  int m[4] = {0, 0, 0, 0};

  for(i = 0; i < 8; i++) {
    move_forward(&robot);
  }
  for(i = 0; i < 8; i++) {
    move_right(&robot);
  }
  for(i = 0; i < 8; i++) {
    move_left(&robot);
  }
  for(i = 0; i < 4; i++) {
    move_backward(&robot);
  }
  turn_left(&robot, 8);
  sweep_left(&robot, 8);
  stand(&robot);
  //robot.moveHome();

  return 0;
}

