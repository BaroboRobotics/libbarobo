#ifndef _MOBOTCOMMS_H_
#define _MOBOTCOMMS_H_

#ifdef _CH_
#pragma package <chbarobo>
#ifdef _WIN32_
#define _WIN32
#include <stdint.h>
#define UINT8 uint8_t
#else
#pragma package <chbluetooth>
#endif
#endif

#include <stdio.h>

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

#ifndef _CH_
#include "mobot_internal.h"
#endif

#ifndef MOBOT_JOINTS_E
#define MOBOT_JOINTS_E
typedef enum mobot_joints_e {
  MOBOT_JOINT1 = 0,
  MOBOT_JOINT2,
  MOBOT_JOINT3,
  MOBOT_JOINT4,
  MOBOT_NUM_JOINTS 
} mobot_joints_t;
#endif

#ifndef MOBOT_JOINT_STATE_E
#define MOBOT_JOINT_STATE_E
enum mobot_joint_state_e
{
    MOBOT_JOINT_IDLE = 0,
    MOBOT_JOINT_MOVING,
    MOBOT_JOINT_GOALSEEK,
};
#endif

typedef enum mobot_joint_direction_e
{
  MOBOT_NEUTRAL,
  MOBOT_FORWARD,
  MOBOT_BACKWARD
} mobotDirection_t;

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
    int getJointAngle(int id, double &angle);
    int getJointSpeed(int id, double &speed);
    int getJointState(int id, int &state);
    int move(double angle1, double angle2, double angle3, double angle4);
    int moveContinuous(int dir1, int dir2, int dir3, int dir4);
    int moveContinuousTime(int dir1, int dir2, int dir3, int dir4, int msecs);
    int moveJointTo(int id, double angle);
    int moveJointWait(int id);
    int moveTo(double angle1, double angle2, double angle3, double angle4);
    int moveWait();
    int moveToZero();
    int setJointSpeed(int id, double speed);
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
#ifndef _CH_
  private:
    int getJointDirection(int id, int &dir);
    int setJointDirection(int id, int dir);
    br_comms_t _comms;
#else
  private:
    static void *g_chmobot_dlhandle;
    static int g_chmobot_dlcount;
#endif /* Not _CH_*/
};
#endif /* If C++ or CH */
#endif /* C_ONLY */

#ifdef _CH_
void * CMobot::g_chmobot_dlhandle = NULL;
int CMobot::g_chmobot_dlcount = 0;
#pragma importf "chmobot.chf"
#endif

#endif /* Header Guard */
