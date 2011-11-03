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

#include "thread_macros.h"

#ifndef _WIN32
typedef struct sockaddr_rc sockaddr_t;
#else
#define AF_BLUETOOTH AF_BTH
#define BTPROTO_RFCOMM BTHPROTO_RFCOMM
typedef SOCKADDR_BTH sockaddr_t;
#endif

#define DEGREE2ENC(angles, enc) \
enc[0] = angles[0] * 10; \
enc[1] = angles[1] * 10; \
enc[2] = angles[2] * 10; \
enc[3] = angles[3] * 10

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

#ifndef MOBOT_JOINT_DIRECTION_E
#define MOBOT_JOINT_DIRECTION_E
typedef enum mobot_motor_direction_e
{
  MOBOT_JOINT_DIR_AUTO,
  MOBOT_JOINT_DIR_FORWARD,
  MOBOT_JOINT_DIR_BACKWARD
} motorDirection_t;
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

typedef struct br_comms_s
{
  int socket;
  int connected;
#ifdef _WIN32
  HANDLE hSerial;
#endif
  sockaddr_t addr;
  double jointSpeeds[4];
  THREAD_T thread;
} br_comms_t;

#define DEF_MOTOR_SPEED 0.50

#ifndef C_ONLY
#ifdef __cplusplus
extern "C" {
#endif
#endif

#ifdef _WIN32

#ifdef _CH_
#define DLLIMPORT
#else
#define DLLIMPORT __declspec(dllexport)
#endif

#else /* Not _WIN32 */
#define DLLIMPORT
#endif /* _WIN32 */

DLLIMPORT int Mobot_init(br_comms_t* comms);
DLLIMPORT int Mobot_connect(br_comms_t* comms);
DLLIMPORT int Mobot_connectWithAddress(
    br_comms_t* comms, const char* address, int channel);
DLLIMPORT int Mobot_disconnect(br_comms_t* comms);
DLLIMPORT int Mobot_isConnected(br_comms_t* comms);
DLLIMPORT int Mobot_setJointDirection(br_comms_t* comms, int id, int dir);
DLLIMPORT int Mobot_getJointDirection(br_comms_t* comms, int id, int *dir);
DLLIMPORT int Mobot_setJointSpeed(br_comms_t* comms, int id, double speed);
DLLIMPORT int Mobot_getJointSpeed(br_comms_t* comms, int id, double *speed);
DLLIMPORT int Mobot_moveJointTo(br_comms_t* comms, int id, double angle);
DLLIMPORT int Mobot_getJointAngle(br_comms_t* comms, int id, double *angle);
DLLIMPORT int Mobot_getJointState(br_comms_t* comms, int id, int *state);
DLLIMPORT int Mobot_move(br_comms_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4);
DLLIMPORT int Mobot_moveContinuous(br_comms_t* comms,
                                  int dir1,
                                  int dir2,
                                  int dir3,
                                  int dir4);
DLLIMPORT int Mobot_moveContinuousTime(br_comms_t* comms,
                                  int dir1,
                                  int dir2,
                                  int dir3,
                                  int dir4,
                                  int msecs);
DLLIMPORT int Mobot_moveTo(br_comms_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4);
DLLIMPORT int Mobot_moveToZero(br_comms_t* comms);
DLLIMPORT int Mobot_moveJointWait(br_comms_t* comms, int id);
DLLIMPORT int Mobot_moveWait(br_comms_t* comms);
DLLIMPORT int Mobot_stop(br_comms_t* comms);

/* compound motion functions */
DLLIMPORT int Mobot_motionInchwormLeft(br_comms_t* comms);
DLLIMPORT int Mobot_motionInchwormRight(br_comms_t* comms);
DLLIMPORT int Mobot_motionRollBackward(br_comms_t* comms);
DLLIMPORT int Mobot_motionRollForward(br_comms_t* comms);
DLLIMPORT int Mobot_motionStand(br_comms_t* comms);
DLLIMPORT int Mobot_motionTurnLeft(br_comms_t* comms);
DLLIMPORT int Mobot_motionTurnRight(br_comms_t* comms);

/* Non-Blocking compound motion functions */
DLLIMPORT int Mobot_motionInchwormLeftNB(br_comms_t* comms);
DLLIMPORT int Mobot_motionInchwormRightNB(br_comms_t* comms);
DLLIMPORT int Mobot_motionRollBackwardNB(br_comms_t* comms);
DLLIMPORT int Mobot_motionRollForwardNB(br_comms_t* comms);
DLLIMPORT int Mobot_motionStandNB(br_comms_t* comms);
DLLIMPORT int Mobot_motionTurnLeftNB(br_comms_t* comms);
DLLIMPORT int Mobot_motionTurnRightNB(br_comms_t* comms);

#ifdef _WIN32
typedef struct bdaddr_s {
  UINT8 b[6];
} bdaddr_t;
int str2ba(const char *str, bdaddr_t *ba);
void baswap(bdaddr_t *dst, const bdaddr_t *src);
int str2ba(const char *str, bdaddr_t *ba);
#endif

#ifndef C_ONLY
#ifdef __cplusplus
}
#endif
#endif

/* Utility Functions */
int SendToIMobot(br_comms_t* comms, const char* str, int len);
int RecvFromIMobot(br_comms_t* comms, char* buf, int size);

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
  private:
    int getJointDirection(int id, int &dir);
    int setJointDirection(int id, int dir);
    br_comms_t _comms;
};
#endif /* If C++ or CH */
#endif /* C_ONLY */

#ifdef _CH_
#pragma importf "mobot.cpp"
#endif

#endif /* Header Guard */
