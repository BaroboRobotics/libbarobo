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
#ifndef __MACH__
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#endif
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

#define angle2distance(radius, angle) ((radius) * ((angle) * M_PI / 180.0))
#define distance2angle(radius, distance) (((distance) / (radius)) * 180 / M_PI)

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
    ROBOT_NEUTRAL = 0,
    ROBOT_FORWARD,
    ROBOT_BACKWARD,
    ROBOT_HOLD,
} robotJointState_t;
#endif

#ifndef _CH_

#ifdef __cplusplus
extern "C" {
#endif
  DLLIMPORT double deg2rad(double deg);
  DLLIMPORT double rad2deg(double rad);
#ifdef __cplusplus
}
#endif

typedef struct recordAngleArg_s 
{
  br_comms_t* comms;
  robotJointId_t id;
  double *time;
  double *angle;
  double *angle2;
  double *angle3;
  double *angle4;
  int num;
  int msecs;
} recordAngleArg_t;
#endif

#ifndef C_ONLY
#if defined (__cplusplus) || defined (_CH_)
class CMobot {
  public:
    CMobot();
    ~CMobot();
    int connect();
    int connectWithAddress(const char address[], int channel);
#ifndef _WIN32
    int connectWithTTY(const char ttyfilename[]);
#endif
    int disconnect();
    int isConnected();
    int isMoving();
    int getJointAngle(robotJointId_t id, double &angle);
    int getJointAngles(double &angle1, double &angle2, double &angle3, double &angle4);
    int getJointMaxSpeed(robotJointId_t id, double &maxSpeed);
    int getJointSafetyAngle(double &angle);
    int getJointSafetyAngleTimeout(double &seconds);
    int getJointSpeed(robotJointId_t id, double &speed);
    int getJointSpeedRatio(robotJointId_t id, double &ratio);
    int getJointSpeeds(double &speed1, double &speed2, double &speed3, double &speed4);
    int getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3, double &ratio4);
    int getJointState(robotJointId_t id, robotJointState_t &state);
    int move(double angle1, double angle2, double angle3, double angle4);
    int moveNB(double angle1, double angle2, double angle3, double angle4);
    int moveContinuousNB(robotJointState_t dir1, 
                       robotJointState_t dir2, 
                       robotJointState_t dir3, 
                       robotJointState_t dir4);
    int moveContinuousTime(robotJointState_t dir1, 
                           robotJointState_t dir2, 
                           robotJointState_t dir3, 
                           robotJointState_t dir4, 
                           double seconds);
    int moveJointContinuousNB(robotJointId_t id, robotJointState_t dir);
    int moveJointContinuousTime(robotJointId_t id, robotJointState_t dir, double seconds);
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
#ifdef _CH_
    int recordAngle(robotJointId_t id, double time[:], double angle[:], int num, double seconds);
    int recordAngles(double time[:], 
                     double angle1[:], 
                     double angle2[:], 
                     double angle3[:], 
                     double angle4[:], 
                     int num, 
                     double seconds);
#else
    int recordAngle(robotJointId_t id, double time[], double angle[], int num, double seconds);
    int recordAngles(double time[], 
                     double angle1[], 
                     double angle2[], 
                     double angle3[], 
                     double angle4[], 
                     int num, 
                     double seconds);
#endif
    int recordWait();
    int setJointSafetyAngle(double angle);
    int setJointSafetyAngleTimeout(double seconds);
    int setJointSpeed(robotJointId_t id, double speed);
    int setJointSpeeds(double speed1, double speed2, double speed3, double speed4);
    int setJointSpeedRatio(robotJointId_t id, double ratio);
    int setJointSpeedRatios(double ratios1, double ratios2, double ratios3, double ratios4);
    int setMotorPower(robotJointId_t id, int power);
    int setTwoWheelRobotSpeed(double speed, double radius);
    int stop();

    int motionArch(double angle);
    int motionInchwormLeft(int num);
    int motionInchwormRight(int num);
    int motionRollBackward(double angle);
    int motionRollForward(double angle);
    int motionSkinny(double angle);
    int motionStand();
    int motionTurnLeft(double angle);
    int motionTurnRight(double angle);
    int motionTumbleRight(int num);
    int motionTumbleLeft(int num);
    int motionUnstand();

    /* Non-Blocking motion functions */
    int motionArchNB(double angle);
    int motionInchwormLeftNB(int num);
    int motionInchwormRightNB(int num);
    int motionRollBackwardNB(double angle);
    int motionRollForwardNB(double angle);
    int motionSkinnyNB(double angle);
    int motionStandNB();
    int motionTurnLeftNB(double angle);
    int motionTurnRightNB(double angle);
    int motionTumbleRightNB(int num);
    int motionTumbleLeftNB(int num);
    int motionUnstandNB();
    int motionWait();
#ifndef _CH_
  private:
    int getJointDirection(robotJointId_t id, robotJointState_t &dir);
    int setJointDirection(robotJointId_t id, robotJointState_t dir);
    br_comms_t _comms;
    void (*buttonCallback)(CMobot *robot, int button, int buttonDown);
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
    int moveContinuousNB(robotJointState_t dir1, 
                       robotJointState_t dir2, 
                       robotJointState_t dir3, 
                       robotJointState_t dir4);
    int moveContinuousTime(robotJointState_t dir1, 
                           robotJointState_t dir2, 
                           robotJointState_t dir3, 
                           robotJointState_t dir4, 
                           double seconds);
    int moveJointContinuousNB(robotJointId_t id, robotJointState_t dir);
    int moveJointContinuousTime(robotJointId_t id, robotJointState_t dir, double seconds);
    int moveJointTo(robotJointId_t id, double angle);
    int moveJointToNB(robotJointId_t id, double angle);
    int moveJointWait(robotJointId_t id);
    int moveTo(double angle1, double angle2, double angle3, double angle4);
    int moveToNB(double angle1, double angle2, double angle3, double angle4);
    int moveWait();
    int moveToZero();
    int moveToZeroNB();
    int setJointSpeed(robotJointId_t id, double speed);
    int setJointSpeeds(double speed1, double speed2, double speed3, double speed4);
    int setJointSpeedRatio(robotJointId_t id, double ratio);
    int setJointSpeedRatios(double ratio1, double ratio2, double ratio3, double ratio4);
    int setTwoWheelRobotSpeed(double speed, double radius);
    int stop();

    int motionArch(double angle);
    int motionArchNB(double angle);
    static void* motionArchThread(void*);
    int motionInchwormLeft(int num);
    int motionInchwormLeftNB(int num);
    static void* motionInchwormLeftThread(void*);
    int motionInchwormRight(int num);
    int motionInchwormRightNB(int num);
    static void* motionInchwormRightThread(void*);
    int motionRollBackward(double angle);
    int motionRollBackwardNB(double angle);
    static void* motionRollBackwardThread(void*);
    int motionRollForward(double angle);
    int motionRollForwardNB(double angle);
    static void* motionRollForwardThread(void*);
    int motionSkinny(double angle);
    int motionSkinnyNB(double angle);
    static void* motionSkinnyThread(void*);
    int motionStand();
    int motionStandNB();
    static void* motionStandThread(void*);
    int motionTurnLeft(double angle);
    int motionTurnLeftNB(double angle);
    static void* motionTurnLeftThread(void*);
    int motionTurnRight(double angle);
    int motionTurnRightNB(double angle);
    static void* motionTurnRightThread(void*);
    int motionTumbleRight(int num);
    int motionTumbleRightNB(int num);
    static void* motionTumbleRightThread(void*);
    int motionTumbleLeft(int num);
    int motionTumbleLeftNB(int num);
    static void* motionTumbleLeftThread(void*);
    int motionUnstand();
    int motionUnstandNB();
    static void* motionUnstandThread(void*);
    int motionWait();

  private:
    int _numRobots;
    CMobot *_robots[64];
    int argInt;
    double argDouble;
#ifndef _CH_
    THREAD_T* _thread;
#else
    void* _thread;
#endif
    int _motionInProgress;
};

#endif /* If C++ or CH */
#endif /* C_ONLY */

#ifdef _CH_
void * CMobot::g_chmobot_dlhandle = NULL;
int CMobot::g_chmobot_dlcount = 0;
#pragma importf "chmobot.chf"
#endif

#endif /* Header Guard */
