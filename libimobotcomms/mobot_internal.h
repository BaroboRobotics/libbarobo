#ifndef _MOBOT_INTERNAL_H_
#define _MOBOT_INTERNAL_H_
#include "mobot.h"
#ifndef _CH_
#include "thread_macros.h"
#endif

/*
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
*/
#define DEG2RAD(x) ((x) * M_PI / 180.0)
#define RAD2DEG(x) ((x) * 180.0 / M_PI)

#ifndef CALLBACK_ARG_S
#define CALLBACK_ARG_S
typedef struct callbackArg_s
{
  mobot_t* comms;
  int button;
  int buttonDown;
} callbackArg_t;
#endif

#ifndef MOBOT_JOINTS_E
#define MOBOT_JOINTS_E
typedef enum mobotJoints_e{
  MOBOT_ZERO,
  MOBOT_JOINT1,
  MOBOT_JOINT2,
  MOBOT_JOINT3,
  MOBOT_JOINT4,
  ROBOT_NUM_JOINTS = 4
} mobotJointId_t;
#endif

#ifndef MOBOT_JOINT_STATE_E
#define MOBOT_JOINT_STATE_E
typedef enum mobotJointState_e
{
    MOBOT_NEUTRAL = 0,
    MOBOT_FORWARD,
    MOBOT_BACKWARD,
    MOBOT_HOLD,
} mobotJointState_t;
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

/* Utility Functions */
//int SendToIMobot(mobot_t* comms, uint8_t cmd, const void* data, int datasize);
//int RecvFromIMobot(mobot_t* comms, uint8_t* buf, int size);
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
