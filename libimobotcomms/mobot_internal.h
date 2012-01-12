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

#define DEG2RAD(x) ((x) * M_PI / 180.0)
#define RAD2DEG(x) ((x) * 180.0 / M_PI)

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
  double maxSpeed[4];
  THREAD_T thread;
} br_comms_t;
#endif

#ifndef ROBOT_JOINTS_E
#define ROBOT_JOINTS_E
typedef enum mobot_joints_e {
  ROBOT_JOINT1 = 1,
  ROBOT_JOINT2 = 2,
  ROBOT_JOINT3 = 3,
  ROBOT_JOINT4 = 4,
  ROBOT_NUM_JOINTS 
} robotJointId_t;
#endif

#ifndef ROBOT_JOINT_STATE_E
#define ROBOT_JOINT_STATE_E
typedef enum mobot_joint_state_e
{
    ROBOT_JOINT_IDLE = 0,
    ROBOT_JOINT_MOVING,
    ROBOT_JOINT_GOALSEEK,
} robotJointState_t;
#endif

#ifndef ROBOT_JOINT_DIRECTION_T
#define ROBOT_JOINT_DIRECTION_T
typedef enum mobot_joint_direction_e
{
  ROBOT_NEUTRAL,
  ROBOT_FORWARD,
  ROBOT_BACKWARD
} robotJointDirection_t;
#endif


#ifndef ROBOT_JOINT_DIRECTION_E
#define ROBOT_JOINT_DIRECTION_E
typedef enum mobot_motor_direction_e
{
  ROBOT_JOINT_DIR_AUTO,
  ROBOT_JOINT_DIR_FORWARD,
  ROBOT_JOINT_DIR_BACKWARD
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
DLLIMPORT int Mobot_setJointDirection(br_comms_t* comms, robotJointId_t id, robotJointDirection_t dir);
DLLIMPORT int Mobot_getJointDirection(br_comms_t* comms, robotJointId_t id, robotJointDirection_t *dir);
DLLIMPORT int Mobot_getJointMaxSpeed(br_comms_t* comms, robotJointId_t, double *maxSpeed);
DLLIMPORT int Mobot_setJointSpeed(br_comms_t* comms, robotJointId_t id, double speed);
DLLIMPORT int Mobot_setJointSpeedRatio(br_comms_t* comms, robotJointId_t id, double ratio);
DLLIMPORT int Mobot_getJointSpeed(br_comms_t* comms, robotJointId_t id, double *speed);
DLLIMPORT int Mobot_getJointSpeedRatio(br_comms_t* comms, robotJointId_t id, double *ratio);
DLLIMPORT int Mobot_getJointSpeeds(br_comms_t* comms, double speeds[4]);
DLLIMPORT int Mobot_setJointSpeeds(br_comms_t* comms, double speeds[4]);
DLLIMPORT int Mobot_getJointSpeedRatios(br_comms_t* comms, double ratios[4]);
DLLIMPORT int Mobot_setJointSpeedRatios(br_comms_t* comms, double ratios[4]);
DLLIMPORT int Mobot_setTwoWheelRobotSpeed(br_comms_t* comms, double speed, double radius, char unit[]);
DLLIMPORT int Mobot_moveJointTo(br_comms_t* comms, robotJointId_t id, double angle);
DLLIMPORT int Mobot_moveJointToNB(br_comms_t* comms, robotJointId_t id, double angle);
DLLIMPORT int Mobot_getJointAngle(br_comms_t* comms, robotJointId_t id, double *angle);
DLLIMPORT int Mobot_getJointState(br_comms_t* comms, robotJointId_t id, robotJointState_t *state);
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
                                  robotJointDirection_t dir1,
                                  robotJointDirection_t dir2,
                                  robotJointDirection_t dir3,
                                  robotJointDirection_t dir4);
DLLIMPORT int Mobot_moveContinuousTime(br_comms_t* comms,
                                  robotJointDirection_t dir1,
                                  robotJointDirection_t dir2,
                                  robotJointDirection_t dir3,
                                  robotJointDirection_t dir4,
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
DLLIMPORT int Mobot_moveJointContinuousNB(br_comms_t* comms, robotJointId_t id, robotJointDirection_t dir);
DLLIMPORT int Mobot_moveJointContinuousTime(br_comms_t* comms, robotJointId_t id, robotJointDirection_t dir, int msecs);
DLLIMPORT int Mobot_moveJointWait(br_comms_t* comms, robotJointId_t id);
DLLIMPORT int Mobot_moveWait(br_comms_t* comms);
DLLIMPORT int Mobot_stop(br_comms_t* comms);
DLLIMPORT int Mobot_moveJointToPIDNB(br_comms_t* comms, robotJointId_t id, double angle);

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
