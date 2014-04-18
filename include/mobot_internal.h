/*
   Copyright 2013 Barobo, Inc.

   This file is part of libbarobo.

   BaroboLink is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   BaroboLink is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with BaroboLink.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _ROBOT_INTERNAL_H_
#define _ROBOT_INTERNAL_H_
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

#ifndef ROBOT_JOINTS_E
#define ROBOT_JOINTS_E
typedef enum robotJoints_e{
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
#define LINKBOT_MAX_SPEED DEG2RAD(240)

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

#define MAX_RETRIES 3
//int MobotMsgTransaction(mobot_t* comms, uint8_t cmd, /*IN&OUT*/ void* buf, int sendsize);
int Mobot_waitForReportedSerialID(mobot_t* comms, char* id);
#endif /* Not _CH_ */

int finishConnect (mobot_t* comms);
int finishConnectWithoutCommsThread(mobot_t* comms);
int getFormFactor(mobot_t* comms, int* form);

/* Hide all of the C-style structs and API from CH */
#ifndef C_ONLY
#ifdef __cplusplus
}
#endif
#endif


#endif
