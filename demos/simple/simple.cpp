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
  char color[20],color2[20]="red";
  int r, g, b;
  int pin=2;
  CLinkbotI robot;
  
  robot.connectWithAddress("XM8V");
  robot.setJointSpeeds(-90, -90, -90);
  robot.moveJoint(ROBOT_JOINT1, 90);
  robot.setJointSpeeds(90,90, 90);
  robot.moveJoint(ROBOT_JOINT1, 90);
  /*
  robot.move(90, 90, 90);
  robot.setJointSpeeds(-90, -90, -90);
  robot.move(90, 90, 90);
  */
  /*
  robot.accelAngularToVelocityNB(ROBOT_JOINT1, 45, 180);
  double speed;
  for(int i = 0; i < 50; i++) {
    robot.getJointSpeed(ROBOT_JOINT1, speed);
    cout << speed << endl;
    usleep(100000);
  }
  */
  /*
  cout << "forward" << endl;
  robot.moveTime(3);
  robot.setJointSpeeds(-90,-90,-90);
  cout << "backward" << endl;
  robot.moveTime(3);
  cout << "backward" << endl;
  robot.moveTime(3);
  robot.setJointSpeeds(90,90,90);
  cout << "forward" << endl;
  robot.moveTime(3);
  */
  /*
  robot.setJointSpeeds(-90,-90,-90);
  cout << "backward" << endl;
  robot.moveJointForeverNB(ROBOT_JOINT1);
  sleep(2);
  cout << "backward" << endl;
  robot.moveJointForeverNB(ROBOT_JOINT1);
  sleep(2);
  robot.setJointSpeeds(90,90,90);
  cout << "forward" << endl;
  robot.moveJointForeverNB(ROBOT_JOINT1);
  sleep(2);
  robot.stop();
  */

  return 0;
}
