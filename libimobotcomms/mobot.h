#ifndef _MOBOTCOMMS_H_
#define _MOBOTCOMMS_H_
#include <math.h>

#ifdef _CH_
#pragma package <chmobot>
#ifdef _WIN32_
#define _WIN32
#include <stdint.h>
#define UINT8 uint8_t
#else
#pragma package <chbluetooth>
#endif
#endif

#include <stdio.h>

#ifndef _CH_
#ifndef _WIN32
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#else
//#include <types.h>
#include <winsock2.h>
#include <Ws2bth.h>
#endif
#endif /* Not if CH */

#ifndef _CH_
#include "mobot_internal.h"
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MOTORMAXSPEED (90.0*M_PI/180.0)

extern int g_numConnected;

#ifndef ROBOT_JOINTS_E
#define ROBOT_JOINTS_E
typedef enum robotJoints_e {
  ROBOT_ZERO,
  ROBOT_JOINT1,
  ROBOT_JOINT2,
  ROBOT_JOINT3,
  ROBOT_JOINT4,
  ROBOT_NUM_JOINTS = 4
} robotJointId_t;
#endif

#ifndef ROBOT_JOINT_STATE_E
#define ROBOT_JOINT_STATE_E
typedef enum robotJointState_e
{
    ROBOT_JOINT_IDLE = 0,
    ROBOT_JOINT_MOVING,
    ROBOT_JOINT_GOALSEEK,
} robotJointState_t;
#endif

#ifndef ROBOT_JOINT_DIRECTION_T
#define ROBOT_JOINT_DIRECTION_T
typedef enum robotJointDirection_e
{
  ROBOT_NEUTRAL,
  ROBOT_FORWARD,
  ROBOT_BACKWARD
} robotJointDirection_t;
#endif

#ifndef _CH_
extern "C" {
  DLLIMPORT double deg2rad(double deg);
  DLLIMPORT double rad2deg(double rad);
}
#endif

#ifndef C_ONLY
#if defined (__cplusplus) || defined (_CH_)
class CMobot {
  public:
    CMobot();
    ~CMobot();
    int connect();
    int connectWithAddress(const char address[], int channel);
    int disconnect();
    int isConnected();
    int isMoving();
    int getJointAngle(robotJointId_t id, double &angle);
    int getJointMaxSpeed(robotJointId_t id, double &maxSpeed);
    int getJointSpeed(robotJointId_t id, double &speed);
    int getJointSpeedRatio(robotJointId_t id, double &ratio);
    int getJointSpeeds(double speeds[4]);
    int getJointSpeedRatios(double ratios[4]);
    int getJointState(robotJointId_t id, robotJointState_t &state);
    int move(double angle1, double angle2, double angle3, double angle4);
    int moveNB(double angle1, double angle2, double angle3, double angle4);
    int moveContinuousNB(robotJointDirection_t dir1, 
                       robotJointDirection_t dir2, 
                       robotJointDirection_t dir3, 
                       robotJointDirection_t dir4);
    int moveContinuousTime(robotJointDirection_t dir1, 
                           robotJointDirection_t dir2, 
                           robotJointDirection_t dir3, 
                           robotJointDirection_t dir4, 
                           int msecs);
    int moveJointContinuousNB(robotJointId_t id, robotJointDirection_t dir);
    int moveJointContinuousTime(robotJointId_t id, robotJointDirection_t dir, int msecs);
    int moveJoint(robotJointId_t id, double angle);
    int moveJointNB(robotJointId_t id, double angle);
    int moveJointTo(robotJointId_t id, double angle);
    int moveJointToNB(robotJointId_t id, double angle);
    int moveJointToPIDNB(robotJointId_t id, double angle);
    int moveJointWait(robotJointId_t id);
    int moveTo(double angle1, double angle2, double angle3, double angle4);
    int moveToNB(double angle1, double angle2, double angle3, double angle4);
    int moveWait();
    int moveToZero();
    int moveToZeroNB();
    int setJointSpeed(robotJointId_t id, double speed);
    int setJointSpeeds(double speeds[4]);
    int setJointSpeedRatio(robotJointId_t id, double ratio);
    int setJointSpeedRatios(double ratios[4]);
    int setTwoWheelRobotSpeed(double speed, double radius, char unit[]);
    int stop();

    int motionInchwormLeft();
    int motionInchwormRight();
    int motionRollBackward();
    int motionRollForward();
    int motionStand();
    int motionTurnLeft();
    int motionTurnRight();

    /* Non-Blocking motion functions */
    int motionInchwormLeftNB();
    int motionInchwormRightNB();
    int motionRollBackwardNB();
    int motionRollForwardNB();
    int motionStandNB();
    int motionTurnLeftNB();
    int motionTurnRightNB();
    int motionWait();
#ifndef _CH_
  private:
    int getJointDirection(robotJointId_t id, robotJointDirection_t &dir);
    int setJointDirection(robotJointId_t id, robotJointDirection_t dir);
    br_comms_t _comms;
#else
  public:
    static void *g_chmobot_dlhandle;
    static int g_chmobot_dlcount;
#endif /* Not _CH_*/
};

class CMobotGroup
{
  public:
    CMobotGroup();
    ~CMobotGroup();
    int addRobot(CMobot& robot);
    int isMoving();
    int move(double angle1, double angle2, double angle3, double angle4);
    int moveNB(double angle1, double angle2, double angle3, double angle4);
    int moveContinuousNB(robotJointDirection_t dir1, 
                       robotJointDirection_t dir2, 
                       robotJointDirection_t dir3, 
                       robotJointDirection_t dir4);
    int moveContinuousTime(robotJointDirection_t dir1, 
                           robotJointDirection_t dir2, 
                           robotJointDirection_t dir3, 
                           robotJointDirection_t dir4, 
                           int msecs);
    int moveJointContinuousNB(robotJointId_t id, robotJointDirection_t dir);
    int moveJointContinuousTime(robotJointId_t id, robotJointDirection_t dir, int msecs);
    int moveJointTo(robotJointId_t id, double angle);
    int moveJointToNB(robotJointId_t id, double angle);
    int moveJointWait(robotJointId_t id);
    int moveTo(double angle1, double angle2, double angle3, double angle4);
    int moveToNB(double angle1, double angle2, double angle3, double angle4);
    int moveWait();
    int moveToZero();
    int moveToZeroNB();
    int setJointSpeed(robotJointId_t id, double speed);
    int setJointSpeeds(robotJointId_t id, double speeds[4]);
    int setJointSpeedRatio(robotJointId_t id, double ratio);
    int setJointSpeedRatios(robotJointId_t id, double ratios[4]);
    int setTwoWheelRobotSpeed(double speed, double radius, char unit[]);
    int stop();

    int motionInchwormLeft();
    int motionInchwormLeftNB();
    int motionInchwormRight();
    int motionInchwormRightNB();
    int motionRollBackward();
    int motionRollBackwardNB();
    int motionRollForward();
    int motionRollForwardNB();
    int motionStand();
    int motionStandNB();
    int motionTurnLeft();
    int motionTurnLeftNB();
    int motionTurnRight();
    int motionTurnRightNB();
    int motionWait();

  private:
    int _numRobots;
    CMobot *_robots[64];
};

#endif /* If C++ or CH */
#endif /* C_ONLY */

#ifdef _CH_
void * CMobot::g_chmobot_dlhandle = NULL;
int CMobot::g_chmobot_dlcount = 0;
#pragma importf "chmobot.chf"
#endif

#endif /* Header Guard */
