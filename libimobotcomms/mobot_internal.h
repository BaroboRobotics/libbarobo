#ifndef _MOBOT_INTERNAL_H_
#define _MOBOT_INTERNAL_H_
#include "mobot.h"
#ifndef _CH_
#include "thread_macros.h"
#endif

#ifndef _WIN32
#ifndef _llvm_
#include <stdint.h>
#else
typedef unsigned char uint8_t;
#endif // _llvm_
typedef struct sockaddr_rc sockaddr_t;
#else
typedef unsigned char uint8_t;
typedef unsigned __int32 uint32_t;
#define AF_BLUETOOTH AF_BTH
#define BTPROTO_RFCOMM BTHPROTO_RFCOMM
typedef SOCKADDR_BTH sockaddr_t;
#endif

#define DEG2RAD(x) ((x) * M_PI / 180.0)
#define RAD2DEG(x) ((x) * 180.0 / M_PI)

#ifndef BR_COMMS_S
#define BR_COMMS_S
typedef struct mobot_s
{
  int socket;
  int connected;
#ifndef __MACH__
  sockaddr_t *addr;
#endif
  double jointSpeeds[4];
  double maxSpeed[4];
  THREAD_T* thread;
  MUTEX_T* commsLock;
  int motionInProgress;
  int motionArgInt;
  double motionArgDouble;
  int recordingInProgress[4];

  THREAD_T* commsThread;
  uint8_t recvBuf[64];
  int recvBuf_ready;
  MUTEX_T* recvBuf_lock;
  COND_T*  recvBuf_cond;
  int recvBuf_bytes;
  int commsBusy;
  MUTEX_T* commsBusy_lock;
  COND_T* commsBusy_cond;

  MUTEX_T* callback_lock;
  int callbackEnabled;
  void (*buttonCallback)(void* robot, int button, int buttonDown);
  void* mobot;
} mobot_t;
#endif

#ifndef CALLBACK_ARG_S
#define CALLBACK_ARG_S
typedef struct callbackArg_s
{
  mobot_t* comms;
  int button;
  int buttonDown;
} callbackArg_t;
#endif

#ifndef ROBOT_JOINTS_E
#define ROBOT_JOINTS_E
typedef enum mobot_joints_e {
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

#define DEF_MOTOR_SPEED 45
#define DEF_MOTOR_MAXSPEED 120

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

DLLIMPORT int Mobot_blinkLED(mobot_t* comms, double delay, int numBlinks);
DLLIMPORT int Mobot_connect(mobot_t* comms);
#ifndef _WIN32
DLLIMPORT int Mobot_connectWithTTY(mobot_t* comms, const char* ttyfilename);
#endif
DLLIMPORT int Mobot_connectWithAddress(
    mobot_t* comms, const char* address, int channel);
DLLIMPORT int Mobot_disconnect(mobot_t* comms);
DLLIMPORT int Mobot_enableButtonCallback(mobot_t* comms, void* mobot, void (*buttonCallback)(void* mobot, int button, int buttonDown));
DLLIMPORT int Mobot_init(mobot_t* comms);
DLLIMPORT int Mobot_isConnected(mobot_t* comms);
DLLIMPORT int Mobot_isMoving(mobot_t* comms);
DLLIMPORT int Mobot_getButtonVoltage(mobot_t* comms, double *voltage);
DLLIMPORT int Mobot_getEncoderVoltage(mobot_t* comms, int pinNumber, double *voltage);
DLLIMPORT int Mobot_getJointAngle(mobot_t* comms, robotJointId_t id, double *angle);
DLLIMPORT int Mobot_getJointAngleAbs(mobot_t* comms, robotJointId_t id, double *angle);
DLLIMPORT int Mobot_getJointAngleTime(mobot_t* comms, robotJointId_t id, double *time, double *angle);
DLLIMPORT int Mobot_getJointAnglesTime(mobot_t* comms, 
                                       double *time, 
                                       double *angle1, 
                                       double *angle2, 
                                       double *angle3, 
                                       double *angle4);
DLLIMPORT int Mobot_getJointAngles(mobot_t* comms, 
                                       double *angle1, 
                                       double *angle2, 
                                       double *angle3, 
                                       double *angle4);
DLLIMPORT int Mobot_getJointAnglesAbsTime(mobot_t* comms, 
                                       double *time, 
                                       double *angle1, 
                                       double *angle2, 
                                       double *angle3, 
                                       double *angle4);
DLLIMPORT int Mobot_getJointAnglesAbs(mobot_t* comms, 
                                       double *angle1, 
                                       double *angle2, 
                                       double *angle3, 
                                       double *angle4);
DLLIMPORT int Mobot_getJointDirection(mobot_t* comms, robotJointId_t id, robotJointState_t *dir);
DLLIMPORT int Mobot_getJointMaxSpeed(mobot_t* comms, robotJointId_t, double *maxSpeed);
DLLIMPORT int Mobot_getJointSafetyAngle(mobot_t* comms, double *angle);
DLLIMPORT int Mobot_getJointSafetyAngleTimeout(mobot_t* comms, double *seconds);
DLLIMPORT int Mobot_getJointSpeed(mobot_t* comms, robotJointId_t id, double *speed);
DLLIMPORT int Mobot_getJointSpeedRatio(mobot_t* comms, robotJointId_t id, double *ratio);
DLLIMPORT int Mobot_getJointSpeedRatios(mobot_t* comms, 
                                        double *ratio1, 
                                        double *ratio2, 
                                        double *ratio3, 
                                        double *ratio4);
DLLIMPORT int Mobot_getJointState(mobot_t* comms, robotJointId_t id, robotJointState_t *state);
DLLIMPORT int Mobot_getStatus(mobot_t* comms);
DLLIMPORT int Mobot_getVersion(mobot_t* comms);
DLLIMPORT int Mobot_move(mobot_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4);
DLLIMPORT int Mobot_moveNB(mobot_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4);
DLLIMPORT int Mobot_moveContinuousNB(mobot_t* comms,
                                  robotJointState_t dir1,
                                  robotJointState_t dir2,
                                  robotJointState_t dir3,
                                  robotJointState_t dir4);
DLLIMPORT int Mobot_moveContinuousTime(mobot_t* comms,
                                  robotJointState_t dir1,
                                  robotJointState_t dir2,
                                  robotJointState_t dir3,
                                  robotJointState_t dir4,
                                  double seconds);
DLLIMPORT int Mobot_moveJoint(mobot_t* comms, robotJointId_t id, double angle);
DLLIMPORT int Mobot_moveJointNB(mobot_t* comms, robotJointId_t id, double angle);
DLLIMPORT int Mobot_moveJointContinuousNB(mobot_t* comms, robotJointId_t id, robotJointState_t dir);
DLLIMPORT int Mobot_moveJointContinuousTime(mobot_t* comms, 
                                            robotJointId_t id, 
                                            robotJointState_t dir, 
                                            double seconds);
DLLIMPORT int Mobot_moveJointTo(mobot_t* comms, robotJointId_t id, double angle);
DLLIMPORT int Mobot_moveJointToAbs(mobot_t* comms, robotJointId_t id, double angle);
DLLIMPORT int Mobot_moveJointToDirect(mobot_t* comms, robotJointId_t id, double angle);
DLLIMPORT int Mobot_moveJointToNB(mobot_t* comms, robotJointId_t id, double angle);
DLLIMPORT int Mobot_moveJointToAbsNB(mobot_t* comms, robotJointId_t id, double angle);
DLLIMPORT int Mobot_moveJointToDirectNB(mobot_t* comms, robotJointId_t id, double angle);
DLLIMPORT int Mobot_moveJointWait(mobot_t* comms, robotJointId_t id);
DLLIMPORT int Mobot_moveTo(mobot_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4);
DLLIMPORT int Mobot_moveToAbs(mobot_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4);
DLLIMPORT int Mobot_moveToDirect(mobot_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4);
DLLIMPORT int Mobot_moveToPID(mobot_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4);
DLLIMPORT int Mobot_moveToNB(mobot_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4);
DLLIMPORT int Mobot_moveToAbsNB(mobot_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4);
DLLIMPORT int Mobot_moveToDirectNB(mobot_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4);
DLLIMPORT int Mobot_moveToPIDNB(mobot_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4);
DLLIMPORT int Mobot_moveToZero(mobot_t* comms);
DLLIMPORT int Mobot_moveToZeroNB(mobot_t* comms);
DLLIMPORT int Mobot_moveWait(mobot_t* comms);
DLLIMPORT int Mobot_recordAngle(mobot_t* comms, 
                                robotJointId_t id, 
                                double* time, 
                                double* angle, 
                                int num, 
                                double timeInterval);
DLLIMPORT int Mobot_recordAngles(mobot_t* comms, 
                                 double *time, 
                                 double* angle1, 
                                 double* angle2,
                                 double* angle3,
                                 double* angle4,
                                 int num,
                                 double timeInterval);
DLLIMPORT int Mobot_recordWait(mobot_t* comms);
DLLIMPORT int Mobot_setJointDirection(mobot_t* comms, robotJointId_t id, robotJointState_t dir);
DLLIMPORT int Mobot_setJointSafetyAngle(mobot_t* comms, double angle);
DLLIMPORT int Mobot_setJointSafetyAngleTimeout(mobot_t* comms, double seconds);
DLLIMPORT int Mobot_setJointSpeed(mobot_t* comms, robotJointId_t id, double speed);
DLLIMPORT int Mobot_setJointSpeedRatio(mobot_t* comms, robotJointId_t id, double ratio);
DLLIMPORT int Mobot_setJointSpeeds(mobot_t* comms, 
                                   double speeds1, 
                                   double speeds2, 
                                   double speeds3, 
                                   double speeds4);
DLLIMPORT int Mobot_setJointSpeedRatios(mobot_t* comms, 
                                        double ratio1, 
                                        double ratio2, 
                                        double ratio3, 
                                        double ratio4);
DLLIMPORT int Mobot_getJointSpeeds(mobot_t* comms, 
                                   double *speed1, 
                                   double *speed2, 
                                   double *speed3, 
                                   double *speed4);
DLLIMPORT int Mobot_setMotorPower(mobot_t* comms, robotJointId_t id, int power);
DLLIMPORT int Mobot_setTwoWheelRobotSpeed(mobot_t* comms, double speed, double radius, char unit[]);
DLLIMPORT int Mobot_stop(mobot_t* comms);
DLLIMPORT int Mobot_moveJointToPIDNB(mobot_t* comms, robotJointId_t id, double angle);

/* compound motion functions */
DLLIMPORT int Mobot_motionArch(mobot_t* comms, double angle);
DLLIMPORT int Mobot_motionInchwormLeft(mobot_t* comms, int num);
DLLIMPORT int Mobot_motionInchwormRight(mobot_t* comms, int num);
DLLIMPORT int Mobot_motionRollBackward(mobot_t* comms, double angle);
DLLIMPORT int Mobot_motionRollForward(mobot_t* comms, double angle);
DLLIMPORT int Mobot_motionSkinny(mobot_t* comms, double angle);
DLLIMPORT int Mobot_motionStand(mobot_t* comms);
DLLIMPORT int Mobot_motionTumbleRight(mobot_t* comms, int num);
DLLIMPORT int Mobot_motionTumbleLeft(mobot_t* comms, int num);
DLLIMPORT int Mobot_motionTurnLeft(mobot_t* comms, double angle);
DLLIMPORT int Mobot_motionTurnRight(mobot_t* comms, double angle);
DLLIMPORT int Mobot_motionUnstand(mobot_t* comms);

/* Non-Blocking compound motion functions */
DLLIMPORT int Mobot_motionArchNB(mobot_t* comms, double angle);
DLLIMPORT int Mobot_motionInchwormLeftNB(mobot_t* comms, int num);
DLLIMPORT int Mobot_motionInchwormRightNB(mobot_t* comms, int num);
DLLIMPORT int Mobot_motionRollBackwardNB(mobot_t* comms, double angle);
DLLIMPORT int Mobot_motionRollForwardNB(mobot_t* comms, double angle);
DLLIMPORT int Mobot_motionSkinnyNB(mobot_t* comms, double angle);
DLLIMPORT int Mobot_motionStandNB(mobot_t* comms);
DLLIMPORT int Mobot_motionTumbleRightNB(mobot_t* comms, int num);
DLLIMPORT int Mobot_motionTumbleLeftNB(mobot_t* comms, int num);
DLLIMPORT int Mobot_motionTurnLeftNB(mobot_t* comms, double angle);
DLLIMPORT int Mobot_motionTurnRightNB(mobot_t* comms, double angle);
DLLIMPORT int Mobot_motionUnstandNB(mobot_t* comms);
DLLIMPORT int Mobot_motionWait(mobot_t* comms);

/* Utility Functions */
int SendToIMobot(mobot_t* comms, uint8_t cmd, const void* data, int datasize);
int RecvFromIMobot(mobot_t* comms, uint8_t* buf, int size);
void* commsEngine(void* arg);
void* callbackThread(void* arg);

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
