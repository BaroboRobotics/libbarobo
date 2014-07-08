/* Filename: simple.cpp
 * Rotate the faceplates by 90 degrees */
#include <mobot.h>
#include <linkbot.h>
#include <stdlib.h>
#include <iostream>
#include <unistd.h>

using namespace std;
int main()
{
  CLinkbotL robot;
  
  robot.connectWithAddress("LM64", 1);

  robot.setJointSpeed(ROBOT_JOINT1, 120);
  robot.setJointMovementStateNB(ROBOT_JOINT1, ROBOT_BACKWARD);
  robot.accelJointTimeNB(ROBOT_JOINT1, 60, 4);
  sleep(5);
  return 0;
}
