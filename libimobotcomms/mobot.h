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
  MOBOT_JOINT1 = 1,
  MOBOT_JOINT2,
  MOBOT_JOINT3,
  MOBOT_JOINT4,
  MOBOT_NUM_JOINTS = 4
} mobotJointId_t;
#endif

#ifndef MOBOT_JOINT_STATE_E
#define MOBOT_JOINT_STATE_E
typedef enum mobot_joint_state_e
{
    MOBOT_JOINT_IDLE = 0,
    MOBOT_JOINT_MOVING,
    MOBOT_JOINT_GOALSEEK,
} mobotJointState_t;
#endif

#ifndef MOBOT_JOINT_DIRECTION_T
#define MOBOT_JOINT_DIRECTION_T
typedef enum mobot_joint_direction_e
{
  MOBOT_NEUTRAL,
  MOBOT_FORWARD,
  MOBOT_BACKWARD
} mobotJointDirection_t;
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
    int getJointAngle(mobotJointId_t id, double &angle);
    int getJointSpeed(mobotJointId_t id, double &speed);
    int getJointState(mobotJointId_t id, mobotJointState_t &state);
    int move(double angle1, double angle2, double angle3, double angle4);
    int moveContinuous(mobotJointDirection_t dir1, 
                       mobotJointDirection_t dir2, 
                       mobotJointDirection_t dir3, 
                       mobotJointDirection_t dir4);
    int moveContinuousTime(mobotJointDirection_t dir1, 
                           mobotJointDirection_t dir2, 
                           mobotJointDirection_t dir3, 
                           mobotJointDirection_t dir4, 
                           int msecs);
    int moveJointTo(mobotJointId_t id, double angle);
    int moveJointWait(mobotJointId_t id);
    int moveTo(double angle1, double angle2, double angle3, double angle4);
    int moveWait();
    int moveToZero();
    int setJointSpeed(mobotJointId_t id, double speed);
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
    int getJointDirection(mobotJointId_t id, mobotJointDirection_t &dir);
    int setJointDirection(mobotJointId_t id, mobotJointDirection_t dir);
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
