#ifndef _MOBOT_INTERNAL_H_
#define _MOBOT_INTERNAL_H_
#include "mobot.h"
#ifndef _CH_
#include "thread_macros.h"
#endif

#ifndef _WIN32
typedef struct sockaddr_rc sockaddr_t;
#else
#define AF_BLUETOOTH AF_BTH
#define BTPROTO_RFCOMM BTHPROTO_RFCOMM
typedef SOCKADDR_BTH sockaddr_t;
#endif

#ifndef BR_COMMS_S
#define BR_COMMS_S
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
#endif

#ifndef MOBOT_JOINTS_E
#define MOBOT_JOINTS_E
typedef enum mobot_joints_e {
  MOBOT_JOINT1 = 0,
  MOBOT_JOINT2,
  MOBOT_JOINT3,
  MOBOT_JOINT4,
  MOBOT_NUM_JOINTS 
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


#ifndef MOBOT_JOINT_DIRECTION_E
#define MOBOT_JOINT_DIRECTION_E
typedef enum mobot_motor_direction_e
{
  MOBOT_JOINT_DIR_AUTO,
  MOBOT_JOINT_DIR_FORWARD,
  MOBOT_JOINT_DIR_BACKWARD
} motorDirection_t;
#endif

#ifndef _CH_

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
DLLIMPORT int Mobot_isMoving(br_comms_t* comms);
DLLIMPORT int Mobot_setJointDirection(br_comms_t* comms, mobotJointId_t id, mobotJointDirection_t dir);
DLLIMPORT int Mobot_getJointDirection(br_comms_t* comms, mobotJointId_t id, mobotJointDirection_t *dir);
DLLIMPORT int Mobot_setJointSpeed(br_comms_t* comms, mobotJointId_t id, double speed);
DLLIMPORT int Mobot_getJointSpeed(br_comms_t* comms, mobotJointId_t id, double *speed);
DLLIMPORT int Mobot_moveJointTo(br_comms_t* comms, mobotJointId_t id, double angle);
DLLIMPORT int Mobot_moveJointToNB(br_comms_t* comms, mobotJointId_t id, double angle);
DLLIMPORT int Mobot_getJointAngle(br_comms_t* comms, mobotJointId_t id, double *angle);
DLLIMPORT int Mobot_getJointState(br_comms_t* comms, mobotJointId_t id, mobotJointState_t *state);
DLLIMPORT int Mobot_move(br_comms_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4);
DLLIMPORT int Mobot_moveNB(br_comms_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4);
DLLIMPORT int Mobot_moveContinuousNB(br_comms_t* comms,
                                  mobotJointDirection_t dir1,
                                  mobotJointDirection_t dir2,
                                  mobotJointDirection_t dir3,
                                  mobotJointDirection_t dir4);
DLLIMPORT int Mobot_moveContinuousTime(br_comms_t* comms,
                                  mobotJointDirection_t dir1,
                                  mobotJointDirection_t dir2,
                                  mobotJointDirection_t dir3,
                                  mobotJointDirection_t dir4,
                                  int msecs);
DLLIMPORT int Mobot_moveTo(br_comms_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4);
DLLIMPORT int Mobot_moveToNB(br_comms_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4);
DLLIMPORT int Mobot_moveToZero(br_comms_t* comms);
DLLIMPORT int Mobot_moveToZeroNB(br_comms_t* comms);
DLLIMPORT int Mobot_moveJointWait(br_comms_t* comms, mobotJointId_t id);
DLLIMPORT int Mobot_moveWait(br_comms_t* comms);
DLLIMPORT int Mobot_stop(br_comms_t* comms);
DLLIMPORT int Mobot_moveJointToPIDNB(br_comms_t* comms, mobotJointId_t id, double angle);

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

/* Utility Functions */
int SendToIMobot(br_comms_t* comms, const char* str, int len);
int RecvFromIMobot(br_comms_t* comms, char* buf, int size);

#endif /* Not _CH_ */

#ifdef _WIN32
typedef struct bdaddr_s {
  UINT8 b[6];
} bdaddr_t;
int str2ba(const char *str, bdaddr_t *ba);
void baswap(bdaddr_t *dst, const bdaddr_t *src);
int str2ba(const char *str, bdaddr_t *ba);
#endif

/* Hide all of the C-style structs and API from CH */
#ifndef C_ONLY
#ifdef __cplusplus
}
#endif
#endif


#endif
