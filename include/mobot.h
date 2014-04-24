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

#ifndef _MOBOTCOMMS_H_
#define _MOBOTCOMMS_H_

#ifdef SWIG
#define DLLIMPORT
%module mobot
%feature("autodoc", "1");
%{
#include "mobot.h"
%}
%apply double & OUTPUT {double &angle};
%apply double & OUTPUT {double &angle1, double &angle2, double &angle3, double &angle4};
//%apply double & OUTPUT {double &angle1, double &angle2, double &angle3};
%apply double & OUTPUT {double &maxSpeed};
%apply double & OUTPUT {double &seconds};
%apply double & OUTPUT {double &speed};
%apply double & OUTPUT {double &ratio};
%apply double & OUTPUT {double &speed1, double &speed2, double &speed3, double &speed4};
%apply double & OUTPUT {double &ratio1, double &ratio2, double &ratio3, double &ratio4};
%apply double & OUTPUT {robotJointState_t &state};
#endif

#ifndef _CH_
#include "donglefwd.h"
#endif

#include <math.h>

#ifdef _CH_
#pragma package <chbarobo>
#ifdef _WIN32_
#define _WIN32
#include <stdint.h>
#define UINT8 uint8_t
#endif
#include <array.h>
#endif

/* 23 April 2014, hlh: Including MinGW C++ headers before stdio.h causes a
 * crash in dongle_get_tty_win32.cpp upon the first call to
 * SetupDiEnumDeviceInfo. Specifically, MinGW C++ headers include:
 *   bits/c++config.h, which includes
 *   bits/os_defines.h, which defines
 *   ___USE_MINGW_ANSI_STDIO 1
 *
 * [This next paragraph is partly conjecture.]
 *
 * If __USE_MINGW_ANSI_STDIO is defined before stdio.h is included, then MinGW
 * uses C99-conformant Standard I/O from libmingwex.a (see [1]). If not, then
 * it uses Microsoft's non-C99-conformant MSVCRT runtime. Somehow, delay-loaded
 * DLLs (i.e., setupapi.dll) get mixed up in this. If __USE_MINGW_ANSI_STDIO
 * is defined, but a Standard I/O call is made inside Mobot_dongleGetTTY
 * before the call to SetupDiEnumDeviceInfo, then Windows loads its libraries
 * in the correct order and everything is fine.
 *
 * Solutions/Workarounds:
 *   - Put a dummy printf in Mobot_dongleGetTTY.
 *   - Ensure that no C++ headers are included before stdio.h.
 *
 * A couple other clues I picked up: when __USE_MINGW_ANSI_STDIO is 0 or
 * undefined, setupapi.dll loads wintrust.dll, which has something to do with
 * application privileges. When the symbol is 1, setupapi.dll does not load
 * wintrust.dll, and the aforementioned crash occurs.
 *
 * [1] http://en.wikipedia.org/wiki/MinGW#Programming_language_support
 */

#include <stdio.h>

#if __cplusplus
#include <string>
#endif

#if defined (NONRELEASE) || defined (SWIG)
#ifndef _CH_
#ifdef __cplusplus
#include "eventqueue.h"
#endif
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
#endif
#endif /* Not if CH */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifdef _WIN32
#ifndef _STDINT
typedef signed char int8_t;
typedef short int16_t;
typedef int int32_t;

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;
#endif
#include <ws2tcpip.h>
/* Order-dependent headers here: basetyps.h should be included before
 * ws2bth.h. This is technically only necessary if WIN32_LEAN_AND_MEAN is
 * defined, which is a reasonable thing to do (it resolves a different header
 * issue, the conflict between winsock.h and winsock2.h). */
#include <basetyps.h>
#include <ws2bth.h>
#endif
#ifndef _CH_
#include "thread_macros.h"
#endif
#else // Release
#define DLLIMPORT
#define THREAD_T void
#define MUTEX_T void
#define COND_T void
#define sockaddr_t void
#ifdef _WIN32
#ifndef _CH_
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
#endif
#endif
#endif

#ifndef _WIN32
#ifndef _llvm_
#include <stdint.h>
#else
typedef unsigned char uint8_t;
#endif // _llvm_
#ifdef NONRELEASE
typedef struct sockaddr_rc sockaddr_t;
#endif
#else
#ifndef _CH_
typedef unsigned char uint8_t;
//typedef unsigned __int32 uint32_t;
//typedef __int32 int32_t;
#endif
#define AF_BLUETOOTH AF_BTH
#define BTPROTO_RFCOMM BTHPROTO_RFCOMM
#ifndef _CH_
#ifdef NONRELEASE
typedef SOCKADDR_BTH sockaddr_t;
#endif
#endif
#endif

typedef double* robotRecordData_t;
#define angle2distance(radius, angle) ((radius) * ((angle) * M_PI / 180.0))
#define distance2angle(radius, distance) (((distance) / (radius)) * 180 / M_PI)

typedef enum mobotConnectionMode_e
{
  MOBOTCONNECT_NONE,
  MOBOTCONNECT_BLUETOOTH,
  MOBOTCONNECT_TCP,
  MOBOTCONNECT_TTY,
  MOBOTCONNECT_ZIGBEE,
  MOBOTCONNECT_NUMMODES
} mobotConnectionMode_t;

#ifndef ROBOT_JOINT_STATE_E
#define ROBOT_JOINT_STATE_E
/* DEPRECATED */
enum mobotJointState_e
{
    MOBOT_NEUTRAL = 0,
    MOBOT_FORWARD,
    MOBOT_BACKWARD,
    MOBOT_HOLD,
    MOBOT_POSITIVE,
    MOBOT_NEGATIVE
};
#define mobotJointState_t robotJointState_t
/* end deprecated */

typedef enum robotJointState_e
{
    ROBOT_NEUTRAL = 0,
    ROBOT_FORWARD,
    ROBOT_BACKWARD,
    ROBOT_HOLD,
    ROBOT_POSITIVE,
    ROBOT_NEGATIVE,
    ROBOT_ACCEL,
} robotJointState_t;
#endif

#ifndef ROBOT_FORMFACTOR_E
typedef enum mobotFormFactor_e
{
  MOBOTFORM_NULL,
  MOBOTFORM_ORIGINAL,
  MOBOTFORM_I,
  MOBOTFORM_L,
  MOBOTFORM_T,
}mobotFormFactor_t;
#endif

/* Defines for Breakout Board */
#define AREF_DEFAULT  0x00
#define AREF_INTERNAL  0x01
#define AREF_INTERNAL1V1  0x02
#define AREF_INTERNAL2V56  0x03
#define AREF_EXTERNAL  0x04

#define PINMODE_INPUT  0x00
#define PINMODE_OUTPUT  0x01
#define PINMODE_INPUTPULLUP  0x02

//new
#define DIGITAL_LOW	  0x00
#define DIGITAL_HIGH  0x01

#define SENDBUF_SIZE 512

#ifndef BR_COMMS_S
#define BR_COMMS_S

typedef void (*Mobot_progressCallbackFunc)(double progress, void* user_data);
typedef void (*Mobot_completionCallbackFunc)(int complete, void* user_data);

struct mobot_s;
typedef struct mobotInfo_s
{
  uint16_t zigbeeAddr;
  char serialID[5];
  struct mobot_s* parent;
  struct mobot_s* mobot; 
  struct mobotInfo_s* next;
} mobotInfo_t;

typedef struct mobot_s
{
  int socket;
  int connected;
  mobotConnectionMode_t connectionMode;
#ifndef __MACH__
  sockaddr_t *addr;
#endif
  double jointSpeeds[4];
  double maxSpeed[4];
  robotJointState_t exitState;
  double wheelRadius;
  double distanceOffset;
  THREAD_T* thread;
  MUTEX_T* commsLock;
  int motionInProgress;
  MUTEX_T* recordingLock;
  int recordingEnabled[4];
  int recordingNumValues[4];
  MUTEX_T* recordingActive_lock;
  COND_T* recordingActive_cond;
  int recordingActive[4];
  double** recordedAngles[4];
  double** recordedTimes;
  int shiftData;
  int shiftDataGlobalEnable;
  int shiftDataGlobal;

  THREAD_T* commsThread;
  uint8_t recvBuf[256];
  int recvBuf_ready;
  MUTEX_T* recvBuf_lock;
  COND_T*  recvBuf_cond;
  int recvBuf_bytes;
  int commsEngine_bytes;
  int commsWaitingForMessage;
  MUTEX_T* commsWaitingForMessage_lock;
  //MUTEX_T* socket_lock;

#ifndef _CH_
  MOBOTdongle *dongle;
#else 
  void *dongle;
#endif

#if 0
  /* deprecated by libsfp */

  THREAD_T* commsOutThread;
  uint8_t *sendBuf;
  int sendBuf_index;
  int sendBuf_N;
  MUTEX_T* sendBuf_lock;
  COND_T* sendBuf_cond;
#endif

  MUTEX_T* callback_lock;
  int callbackEnabled;
  void (*buttonCallback)(void* mobot, int button, int buttonDown);
  void (*jointCallback)(int, double, double, double, double, int, void*);
  void* jointCallbackData;
  void (*accelCallback)(int, double, double, double, void*);
  void* accelCallbackData;
  void* mobot;
  void (*eventCallback)(const uint8_t* buf, int size, void* userdata);
  void* eventCallbackData;
  char* configFilePath;
  void* itemsToFreeOnExit[64];
  int numItemsToFreeOnExit;
  char* lockfileName;
  /* If the connection mode is zigbee, we must have a parent mobot directly
   * connected. We must talk through the parent mobot to reach this mobot. */
  uint16_t zigbeeAddr;
  char serialID[5];
  mobotFormFactor_t formFactor;

  MUTEX_T* mobotTree_lock;
  COND_T* mobotTree_cond;
  struct mobot_s* parent;
  /* If a Mobot is a dedicated dongle, there still must be a child mobot
   * instance associated with the dongle in order to control it as a robot. */
  struct mobot_s* child; 
  struct mobotInfo_s* children;
  MUTEX_T* scan_callback_lock;
  void (*scan_callback) (const char* serialID);
#if defined (__cplusplus) && defined (NONRELEASE)
  RingBuf<event_t*> *eventqueue;
#else
  void *eventqueue;
#endif
  THREAD_T *eventthread;
} mobot_t;
#endif

#ifndef MOTION_ARG_S
#define MOTION_ARG_S
typedef struct motionArg_s
{
  int i;
  double d;
  mobot_t* mobot;
}motionArg_t;
#endif

#ifndef ROBOT_MELODY_NOTE
#define ROBOT_MELODY_NOTE
typedef struct mobotMelodyNote_s
{
  int tempo;
  /* Bits 15-8 -- note value divider */
  /* Bits 7-3 -- octave */
  /* Bits 2-0 -- note index */
  uint8_t notedata[2]; 
  struct mobotMelodyNote_s *next;
} mobotMelodyNote_t;
#endif

#ifdef NONRELEASE
#ifndef _CH_
#include "mobot_internal.h"
#endif
#endif

#if defined(_WIN32) && defined(__GNUC__) && !defined(strdup)
#define strdup(x) mc_strdup(x)
#ifdef __cplusplus
extern "C" {
#endif
DLLIMPORT char* mc_strdup(const char* str);
#ifdef __cplusplus
}
#endif
#endif

#ifndef ROBOT_JOINTS_E
#define ROBOT_JOINTS_E
/* DEPRECATED */
#if 0
typedef enum mobotJoints_e {
  MOBOT_ZERO,
  MOBOT_JOINT1,
  MOBOT_JOINT2,
  MOBOT_JOINT3,
  MOBOT_JOINT4,
  MOBOT_NUM_JOINTS = 4
} mobotJoints_t;
#define mobotJointId_t robotJointId_t
#endif
#define MOBOT_ZERO 0
#define MOBOT_JOINT1 1
#define MOBOT_JOINT2 2
#define MOBOT_JOINT3 3
#define MOBOT_JOINT4 4
/* End deprecated */

typedef enum robotJoints_e {
  ROBOT_ZERO,
  ROBOT_JOINT1,
  ROBOT_JOINT2,
  ROBOT_JOINT3,
  ROBOT_JOINT4,
  ROBOT_NUM_JOINTS = 4
} robotJointId_t;
#endif

#ifndef _CH_

#ifdef __cplusplus
extern "C" {
#endif
  DLLIMPORT double deg2rad(double deg);
  DLLIMPORT double rad2deg(double rad);
  DLLIMPORT double degree2radian(double deg);
  DLLIMPORT double radian2degree(double rad);
#ifndef SWIG
  DLLIMPORT void delay(double seconds);
#endif
#ifdef __cplusplus
}
#endif

typedef struct recordAngleArg_s 
{
  mobot_t* comms;
  robotJointId_t id;
  double *time;
  double *angle;
  double *angle2;
  double *angle3;
  double *angle4;
  double **time_p;
  double **angle_p;
  double **angle2_p;
  double **angle3_p;
  double **angle4_p;
  int num;
  int i; // Number of recorded items
  int msecs;
} recordAngleArg_t;
#endif

#ifndef SWIG
#ifndef C_ONLY
#if defined (__cplusplus) || defined (_CH_) 
#ifdef _CH_
double systemTime();
class CMobot 
{
  public:
    CMobot();
    ~CMobot();
    int blinkLED(double delay, int numBlinks);
    bool canFlashFirmware ();
    int flashFirmwareAsync (std::string hexfile,
                            Mobot_progressCallbackFunc progressCallback,
                            Mobot_completionCallbackFunc completionCallback,
                            void* user_data);
/* connect() Return Error Codes:
   -1 : General Error
   -2 : Lockfile Exists
   -3 : Address Format Incorrect
   -4 : Not enough entries in the configuration file
   -5 : Bluetooth device not found
   -6 : Protocol version mismatch
   */
    int connect();
    int connectWithAddress(const char address[], ...);
    int connectWithBluetoothAddress(const char address[], ...);
    int connectWithIPAddress(const char address[], ...);
#ifndef _WIN32
    int connectWithTTY(const char ttyfilename[]);
#endif
    int delaySeconds(int seconds);
    int disconnect();
    int driveJointToDirect(robotJointId_t id, double angle);
    int driveJointTo(robotJointId_t id, double angle);
    int driveJointToDirectNB(robotJointId_t id, double angle);
    int driveJointToNB(robotJointId_t id, double angle);
    int driveToDirect(double angle1, double angle2, double angle3, double angle4);
    int driveTo(double angle1, double angle2, double angle3, double angle4);
    int driveToDirectNB(double angle1, double angle2, double angle3, double angle4);
    int driveToNB(double angle1, double angle2, double angle3, double angle4);
    int enableButtonCallback(void* userdata, void (*buttonCallback)(void* data, int button, int buttonDown));
    int disableButtonCallback();
    int enableRecordDataShift();
    int disableRecordDataShift();
    int isConnected();
    int isMoving();
    int getDistance(double &distance, double radius);
    int getFormFactor(int &formFactor);
    static const char* getConfigFilePath();
    int getJointAngle(robotJointId_t id, double &angle);
    int getJointAngleAverage(robotJointId_t id, double &angle, ... );
    int getJointAngles(double &angle1, double &angle2, double &angle3, double &angle4);
    int getJointAnglesAverage(double &angle1, double &angle2, double &angle3, double &angle4, ...);
    int getJointMaxSpeed(robotJointId_t id, double &maxSpeed);
    int getJointSafetyAngle(double &angle);
    int getJointSafetyAngleTimeout(double &seconds);
    int getJointSpeed(robotJointId_t id, double &speed);
    int getJointSpeedRatio(robotJointId_t id, double &ratio);
    int getJointSpeeds(double &speed1, double &speed2, double &speed3, double &speed4);
    int getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3, double &ratio4);
    int getJointState(robotJointId_t id, robotJointState_t &state);
    mobot_t* getMobotObject();
    int getVersion();
    int move(double angle1, double angle2, double angle3, double angle4);
    int moveNB(double angle1, double angle2, double angle3, double angle4);
    int moveBackward(double angle);
    int moveBackwardNB(double angle);
    int moveContinuousNB(robotJointState_t dir1, 
                       robotJointState_t dir2, 
                       robotJointState_t dir3, 
                       robotJointState_t dir4);
    int moveContinuousTime(robotJointState_t dir1, 
                           robotJointState_t dir2, 
                           robotJointState_t dir3, 
                           robotJointState_t dir4, 
                           double seconds);
    int moveDistance(double distance, double radius);
    int moveDistanceNB(double distance, double radius);
    int moveForward(double angle);
    int moveForwardNB(double angle);
    int moveJointContinuousNB(robotJointId_t id, robotJointState_t dir);
    int moveJointContinuousTime(robotJointId_t id, robotJointState_t dir, double seconds);
    int moveJoint(robotJointId_t id, double angle);
    int moveJointNB(robotJointId_t id, double angle);
    int moveJointTo(robotJointId_t id, double angle);
    int moveJointToDirect(robotJointId_t id, double angle);
    int moveJointToNB(robotJointId_t id, double angle);
    int moveJointToDirectNB(robotJointId_t id, double angle);
    int moveJointWait(robotJointId_t id);
    int moveTo(double angle1, double angle2, double angle3, double angle4);
    int moveToDirect(double angle1, double angle2, double angle3, double angle4);
    int moveToNB(double angle1, double angle2, double angle3, double angle4);
    int moveToDirectNB(double angle1, double angle2, double angle3, double angle4);
    int moveWait();
    int moveToZero();
    int moveToZeroNB();
    int movexy(double x, double y, double radius, double trackwidth);
    int movexyNB(double x, double y, double radius, double trackwidth);
    int recordAngle(robotJointId_t id, double time[:], double angle[:], int num, double seconds, ...);
    int recordAngles(double time[:], 
                     double angle1[:], 
                     double angle2[:], 
                     double angle3[:], 
                     double angle4[:], 
                     int num, 
                     double seconds,
                     ...);
    int recordAngleBegin(robotJointId_t id, robotRecordData_t &time, robotRecordData_t &angle, double seconds, ...);
    int recordDistanceBegin(robotJointId_t id, robotRecordData_t &time, robotRecordData_t &distance, double radius, double seconds, ...);
    int recordAngleEnd(robotJointId_t id, int &num);
    int recordDistanceEnd(robotJointId_t id, int &num);
    int recordDistanceOffset(double distance);
    int recordAnglesBegin(robotRecordData_t &time, 
                          robotRecordData_t &angle1, 
                          robotRecordData_t &angle2, 
                          robotRecordData_t &angle3, 
                          robotRecordData_t &angle4, 
                          double seconds,
                          ...);
    int recordDistancesBegin(robotRecordData_t &time, 
                          robotRecordData_t &distance1, 
                          robotRecordData_t &distance2, 
                          robotRecordData_t &distance3, 
                          robotRecordData_t &distance4, 
                          double radius,
                          double seconds,
                          ...);
    int recordAnglesEnd(int &num);
    int recordDistancesEnd(int &num);
    int recordWait();
    int reset();
    int resetToZero();
    int resetToZeroNB();
    int setExitState(robotJointState_t exitState);
    int setJointMovementStateNB(robotJointId_t id, robotJointState_t dir);
    int setJointMovementStateTime(robotJointId_t id, robotJointState_t dir, double seconds);
    int setJointSafetyAngle(double angle);
    int setJointSafetyAngleTimeout(double seconds);
    int setJointSpeed(robotJointId_t id, double speed);
    int setJointSpeeds(double speed1, double speed2, double speed3, double speed4);
    int setJointSpeedRatio(robotJointId_t id, double ratio);
    int setJointSpeedRatios(double ratios1, double ratios2, double ratios3, double ratios4);
    int setMotorPower(robotJointId_t id, int power);
    int setMovementStateNB( robotJointState_t dir1,
        robotJointState_t dir2,
        robotJointState_t dir3,
        robotJointState_t dir4);
    int setMovementStateTime( robotJointState_t dir1,
        robotJointState_t dir2,
        robotJointState_t dir3,
        robotJointState_t dir4,
        double seconds);
    int setMovementStateTimeNB( robotJointState_t dir1,
        robotJointState_t dir2,
        robotJointState_t dir3,
        robotJointState_t dir4,
        double seconds);
    int setTwoWheelRobotSpeed(double speed, double radius);
    int stop();
    int stopOneJoint(robotJointId_t id);
    int stopTwoJoints(robotJointId_t id1, robotJointId_t id2);
    int stopThreeJoints(robotJointId_t id1, robotJointId_t id2, robotJointId_t id3);
    int stopAllJoints();
    int turnLeft(double angle, double radius, double tracklength);
    int turnLeftNB(double angle, double radius, double tracklength);
    int turnRight(double angle, double radius, double tracklength);
    int turnRightNB(double angle, double radius, double tracklength);

    int motionArch(double angle);
    int motionDistance(double distance, double radius);
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
    int motionDistanceNB(double distance, double radius);
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
    int systemTime(double &time);
  public:
    static void *g_chmobot_dlhandle;
    static int g_chmobot_dlcount;
};
#else
class DLLIMPORT CMobot 
{
  public:
    CMobot();
    virtual ~CMobot();
    virtual int accelTimeNB(double radius, double acceleration, double time);
    virtual int accelToVelocityNB(double radius, double acceleration, double velocity);
    virtual int accelToMaxSpeedNB(double radius, double acceleration);
    virtual int accelAngularTimeNB(robotJointId_t id, double acceleration, double time);
    virtual int accelAngularToVelocityNB(robotJointId_t id, double acceleration, double speed);
    virtual int accelAngularAngleNB(robotJointId_t id, double acceleration, double angle);
    virtual int smoothMoveToNB(
        robotJointId_t id,
        double accel0,
        double accelf,
        double vmax,
        double angle);
    virtual int blinkLED(double delay, int numBlinks);

    virtual bool canFlashFirmware ();
    virtual int flashFirmwareAsync (std::string hexfile,
                                    Mobot_progressCallbackFunc progressCallback,
                                    Mobot_completionCallbackFunc completionCallback,
                                    void* user_data);

/* connect() Return Error Codes:
   -1 : General Error
   -2 : Lockfile Exists
   -3 : Address Format Incorrect
   -4 : Not enough entries in the configuration file
   -5 : Bluetooth device not found
   -6 : Protocol version mismatch
   */
    virtual int connect();
    virtual int connectWithAddress(const char address[], int channel = 1);
    virtual int connectWithBluetoothAddress(const char address[], int channel = 1);
    virtual int connectWithIPAddress(const char address[], const char port[] = "5768");
    virtual int connectWithTTY(const char ttyfilename[]);
    virtual int delaySeconds(int seconds);
    virtual int disconnect();
    virtual int driveJointToDirect(robotJointId_t id, double angle);
    virtual int driveJointTo(robotJointId_t id, double angle);
    virtual int driveJointToDirectNB(robotJointId_t id, double angle);
    virtual int driveJointToNB(robotJointId_t id, double angle);
    virtual int driveToDirect(double angle1, double angle2, double angle3, double angle4);
    virtual int driveTo(double angle1, double angle2, double angle3, double angle4);
    virtual int driveToDirectNB(double angle1, double angle2, double angle3, double angle4);
    virtual int driveToNB(double angle1, double angle2, double angle3, double angle4);
    virtual int enableButtonCallback(void* userdata, void (*buttonCallback)(void* data, int button, int buttonDown));
    virtual int disableButtonCallback();
    virtual int enableJointEventCallback(void *userdata,
        void (*jointCallback)(int millis, double j1, double j2, double j3, double j4, int mask, void *userdata));
    virtual int disableJointEventCallback();
    virtual int enableEventCallback(void (*eventCallback)(const uint8_t *buf, int size, void* userdata),
        void* userdata);
    virtual int disableEventCallback();
    virtual int enableRecordDataShift();
    virtual int disableRecordDataShift();
    virtual int isConnected();
    virtual int isMoving();
    virtual int getFormFactor(int &formFactor);
    static const char* getConfigFilePath();
    virtual int getDistance(double &distance, double radius);
#ifndef SWIG
    int getJointAngle(robotJointId_t id, double &angle);
#else
    virtual void getJointAngle(robotJointId_t id, double &angle);
#endif
    virtual int getJointAngles(double &angle1, double &angle2, double &angle3, double &angle4);
    virtual int getJointAngleAverage(robotJointId_t id, double &angle, int numReadings=10);
    virtual int getJointAnglesAverage(double &angle1, double &angle2, double &angle3, double &angle4, int numReadings=10);
    virtual int getJointMaxSpeed(robotJointId_t id, double &maxSpeed);
    virtual int getJointSafetyAngle(double &angle);
    virtual int getJointSafetyAngleTimeout(double &seconds);
    virtual int getJointSpeed(robotJointId_t id, double &speed);
    virtual int getJointSpeedRatio(robotJointId_t id, double &ratio);
    virtual int getJointSpeeds(double &speed1, double &speed2, double &speed3, double &speed4);
    virtual int getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3, double &ratio4);
    virtual int getJointState(robotJointId_t id, robotJointState_t &state);
    virtual mobot_t* getMobotObject();
    virtual int getVersion();
    virtual int move(double angle1, double angle2, double angle3, double angle4);
    virtual int moveNB(double angle1, double angle2, double angle3, double angle4);
    virtual int moveBackward(double angle);
    virtual int moveBackwardNB(double angle);
    virtual int moveContinuousNB(robotJointState_t dir1, 
                       robotJointState_t dir2, 
                       robotJointState_t dir3, 
                       robotJointState_t dir4);
    virtual int moveContinuousTime(robotJointState_t dir1, 
                           robotJointState_t dir2, 
                           robotJointState_t dir3, 
                           robotJointState_t dir4, 
                           double seconds);
    virtual int moveDistance(double distance, double radius);
    virtual int moveDistanceNB(double distance, double radius);
    virtual int moveForward(double angle);
    virtual int moveForwardNB(double angle);
    virtual int moveJointContinuousNB(robotJointId_t id, robotJointState_t dir);
    virtual int moveJointContinuousTime(robotJointId_t id, robotJointState_t dir, double seconds);
    virtual int moveJoint(robotJointId_t id, double angle);
    virtual int moveJointNB(robotJointId_t id, double angle);
    virtual int moveJointTo(robotJointId_t id, double angle);
    virtual int moveJointToDirect(robotJointId_t id, double angle);
    virtual int moveJointToNB(robotJointId_t id, double angle);
    virtual int moveJointToDirectNB(robotJointId_t id, double angle);
    virtual int moveJointWait(robotJointId_t id);
    virtual int moveTo(double angle1, double angle2, double angle3, double angle4);
    virtual int moveToDirect(double angle1, double angle2, double angle3, double angle4);
    virtual int moveToNB(double angle1, double angle2, double angle3, double angle4);
    virtual int moveToDirectNB(double angle1, double angle2, double angle3, double angle4);
    virtual int moveWait();
    virtual int moveToZero();
    virtual int moveToZeroNB();
    virtual int movexy(double x, double y, double radius, double trackwidth);
    virtual int movexyNB(double x, double y, double radius, double trackwidth);
    virtual int recordAngle(robotJointId_t id, double time[], double angle[], int num, double seconds, int shiftData = 1);
    virtual int recordAngles(double time[], 
                     double angle1[], 
                     double angle2[], 
                     double angle3[], 
                     double angle4[], 
                     int num, 
                     double seconds,
                     int shiftData = 1);
    virtual int recordAngleBegin(robotJointId_t id, robotRecordData_t &time, robotRecordData_t &angle, double seconds, int shiftData = 1);
    virtual int recordDistanceBegin(robotJointId_t id, robotRecordData_t &time, robotRecordData_t &distance, double radius, double seconds, int shiftData = 1);
    virtual int recordAngleEnd(robotJointId_t id, int &num);
    virtual int recordDistanceEnd(robotJointId_t id, int &num);
    virtual int recordDistanceOffset(double distance);
    virtual int recordAnglesBegin(robotRecordData_t &time, 
                          robotRecordData_t &angle1, 
                          robotRecordData_t &angle2, 
                          robotRecordData_t &angle3, 
                          robotRecordData_t &angle4, 
                          double seconds,
                          int shiftData = 1);
    virtual int recordDistancesBegin(robotRecordData_t &time, 
                          robotRecordData_t &distance1, 
                          robotRecordData_t &distance2, 
                          robotRecordData_t &distance3, 
                          robotRecordData_t &distance4, 
                          double radius,
                          double seconds,
                          int shiftData = 1);
    virtual int recordAnglesEnd(int &num);
    virtual int recordDistancesEnd(int &num);
    virtual int recordWait();
    virtual int reset();
    virtual int resetToZero();
    virtual int resetToZeroNB();
    virtual int setAccelEventThreshold(double threshold);
    virtual int setJointEventThreshold(int joint, double threshold);
    virtual int setExitState(robotJointState_t exitState);
    virtual int setJointMovementStateNB(robotJointId_t id, robotJointState_t dir);
    virtual int setJointMovementStateTime(robotJointId_t id, robotJointState_t dir, double seconds);
    virtual int setJointSafetyAngle(double angle);
    virtual int setJointSafetyAngleTimeout(double seconds);
    virtual int setJointSpeed(robotJointId_t id, double speed);
    virtual int setJointSpeeds(double speed1, double speed2, double speed3, double speed4);
    virtual int setJointSpeedRatio(robotJointId_t id, double ratio);
    virtual int setJointSpeedRatios(double ratios1, double ratios2, double ratios3, double ratios4);
    virtual int setMotorPower(robotJointId_t id, int power);
    virtual int setMovementStateNB( robotJointState_t dir1,
        robotJointState_t dir2,
        robotJointState_t dir3,
        robotJointState_t dir4);
    virtual int setMovementStateTime( robotJointState_t dir1,
        robotJointState_t dir2,
        robotJointState_t dir3,
        robotJointState_t dir4,
        double seconds);
    virtual int setMovementStateTimeNB( robotJointState_t dir1,
        robotJointState_t dir2,
        robotJointState_t dir3,
        robotJointState_t dir4,
        double seconds);
    virtual int setTwoWheelRobotSpeed(double speed, double radius);
    virtual int stop();
    virtual int stopOneJoint(robotJointId_t id);
    virtual int stopTwoJoints(robotJointId_t id1, robotJointId_t id2);
    virtual int stopThreeJoints(robotJointId_t id1, robotJointId_t id2, robotJointId_t id3);
    virtual int stopAllJoints();
    virtual int turnLeft(double angle, double radius, double tracklength);
    virtual int turnLeftNB(double angle, double radius, double tracklength);
    virtual int turnRight(double angle, double radius, double tracklength);
    virtual int turnRightNB(double angle, double radius, double tracklength);

    virtual int motionArch(double angle);
    virtual int motionDistance(double distance, double radius);
    virtual int motionInchwormLeft(int num);
    virtual int motionInchwormRight(int num);
    virtual int motionRollBackward(double angle);
    virtual int motionRollForward(double angle);
    virtual int motionSkinny(double angle);
    virtual int motionStand();
    virtual int motionTurnLeft(double angle);
    virtual int motionTurnRight(double angle);
    virtual int motionTumbleRight(int num);
    virtual int motionTumbleLeft(int num);
    virtual int motionUnstand();

    /* Non-Blocking motion functions */
    virtual int motionArchNB(double angle);
    virtual int motionDistanceNB(double distance, double radius);
    virtual int motionInchwormLeftNB(int num);
    virtual int motionInchwormRightNB(int num);
    virtual int motionRollBackwardNB(double angle);
    virtual int motionRollForwardNB(double angle);
    virtual int motionSkinnyNB(double angle);
    virtual int motionStandNB();
    virtual int motionTurnLeftNB(double angle);
    virtual int motionTurnRightNB(double angle);
    virtual int motionTumbleRightNB(int num);
    virtual int motionTumbleLeftNB(int num);
    virtual int motionUnstandNB();
    virtual int motionWait();
    virtual int systemTime(double &time);
    int transactMessage(int cmd, void* buf, int size);
  protected:
    int getJointDirection(robotJointId_t id, robotJointState_t &dir);
    int setJointDirection(robotJointId_t id, robotJointState_t dir);
    mobot_t *_comms;
    void (*buttonCallback)(CMobot *mobot, int button, int buttonDown);
};
#endif

class CMobotGroup
{
  public:
    CMobotGroup();
    virtual ~CMobotGroup();
    int addRobot(CMobot& mobot);
#ifdef _CH_
    int addRobots(array CMobot mobots[], ...);
#else
    int addRobots(CMobot mobots[], int numMobots);
#endif
    int connect();
    int driveJointToDirect(robotJointId_t id, double angle);
    int driveJointTo(robotJointId_t id, double angle);
    int driveJointToDirectNB(robotJointId_t id, double angle);
    int driveJointToNB(robotJointId_t id, double angle);
    int driveToDirect(double angle1, double angle2, double angle3, double angle4);
    int driveTo(double angle1, double angle2, double angle3, double angle4);
    int driveToDirectNB(double angle1, double angle2, double angle3, double angle4);
    int driveToNB(double angle1, double angle2, double angle3, double angle4);
    int isMoving();
    int move(double angle1, double angle2, double angle3, double angle4);
    int moveNB(double angle1, double angle2, double angle3, double angle4);
    int moveBackward(double angle);
    int moveBackwardNB(double angle);
    int moveContinuousNB(robotJointState_t dir1, 
                       robotJointState_t dir2, 
                       robotJointState_t dir3, 
                       robotJointState_t dir4);
    int moveContinuousTime(robotJointState_t dir1, 
                           robotJointState_t dir2, 
                           robotJointState_t dir3, 
                           robotJointState_t dir4, 
                           double seconds);
    int moveDistance(double distance, double radius);
    int moveDistanceNB(double distance, double radius);
    int moveForward(double angle);
    int moveForwardNB(double angle);
    int moveJointContinuousNB(robotJointId_t id, robotJointState_t dir);
    int moveJointContinuousTime(robotJointId_t id, robotJointState_t dir, double seconds);
    int moveJointTo(robotJointId_t id, double angle);
    int moveJointToDirect(robotJointId_t id, double angle);
    int moveJointToNB(robotJointId_t id, double angle);
    int moveJointToDirectNB(robotJointId_t id, double angle);
    int moveJointWait(robotJointId_t id);
    int moveTo(double angle1, double angle2, double angle3, double angle4);
    int moveToDirect(double angle1, double angle2, double angle3, double angle4);
    int moveToNB(double angle1, double angle2, double angle3, double angle4);
    int moveToDirectNB(double angle1, double angle2, double angle3, double angle4);
    int moveWait();
    int moveToZero();
    int moveToZeroNB();
    int reset();
    int resetToZero();
    int resetToZeroNB();
    int setExitState(robotJointState_t exitState);
    int setJointMovementStateNB(robotJointId_t id, robotJointState_t dir);
    int setJointMovementStateTime(robotJointId_t id, robotJointState_t dir, double seconds);
    int setJointMovementStateTimeNB(robotJointId_t id, robotJointState_t dir, double seconds);
    int setJointSafetyAngle(double angle);
    int setJointSafetyAngleTimeout(double angle);
    int setJointSpeed(robotJointId_t id, double speed);
    int setJointSpeeds(double speed1, double speed2, double speed3, double speed4);
    int setJointSpeedRatio(robotJointId_t id, double ratio);
    int setJointSpeedRatios(double ratio1, double ratio2, double ratio3, double ratio4);
    int setMovementStateNB(robotJointState_t dir1, 
        robotJointState_t dir2, 
        robotJointState_t dir3, 
        robotJointState_t dir4);
    int setMovementStateTime(robotJointState_t dir1, 
        robotJointState_t dir2, 
        robotJointState_t dir3, 
        robotJointState_t dir4, 
        double seconds);
    int setMovementStateTimeNB(robotJointState_t dir1, 
        robotJointState_t dir2, 
        robotJointState_t dir3, 
        robotJointState_t dir4, 
        double seconds);
    int setTwoWheelRobotSpeed(double speed, double radius);
    int stopAllJoints();
    int stopOneJoint(robotJointId_t id);
    int stopTwoJoints(robotJointId_t id1, robotJointId_t id2);
    int stopThreeJoints(robotJointId_t id1, robotJointId_t id2, robotJointId_t id3);
    int turnLeft(double angle, double radius, double tracklength);
    int turnLeftNB(double angle, double radius, double tracklength);
    int turnRight(double angle, double radius, double tracklength);
    int turnRightNB(double angle, double radius, double tracklength);

    int motionArch(double angle);
    int motionArchNB(double angle);
    static void* motionArchThread(void*);
    int motionDistance(double distance, double radius);
    int motionDistanceNB(double distance, double radius);
    static void* motionDistanceThread(void*);
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

  protected:
    CMobot **_robots;
    int _numRobots;
    int argInt;
    double argDouble;
    int _numAllocated;
#ifndef _CH_
    THREAD_T* _thread;
#else
    void* _thread;
#endif
    int _motionInProgress;
};

#endif /* If C++ or CH */
#endif /* C_ONLY */
#endif /* SWIG */

#ifdef __cplusplus
extern "C" {
#endif

extern int g_numConnected;

DLLIMPORT int Mobot_accelTimeNB(mobot_t* comms, double radius, double acceleration, double time);
DLLIMPORT int Mobot_accelToVelocityNB(mobot_t* comms, double radius, double acceleration, double velocity);
DLLIMPORT int Mobot_accelToMaxSpeedNB(mobot_t* comms, double radius, double acceleration);
DLLIMPORT int Mobot_accelAngularTimeNB(mobot_t* comms, robotJointId_t id, double acceleration, double time);
DLLIMPORT int Mobot_accelAngularToVelocityNB(mobot_t* comms, robotJointId_t id, double acceleration, double speed);
DLLIMPORT int Mobot_accelAngularAngleNB(mobot_t* comms, robotJointId_t id, double acceleration, double angle);
DLLIMPORT int Mobot_blinkLED(mobot_t* comms, double delay, int numBlinks);
DLLIMPORT int Mobot_connect(mobot_t* comms);
DLLIMPORT int Mobot_connectWithTCP(mobot_t* comms);
DLLIMPORT int Mobot_connectWithIPAddress(mobot_t* comms, const char address[], const char port[]);
DLLIMPORT int Mobot_connectWithAddressTTY(mobot_t* comms, const char* address);
DLLIMPORT int Mobot_connectWithTTY(mobot_t* comms, const char* ttyfilename);
DLLIMPORT int Mobot_connectWithTTYBaud(mobot_t* comms, const char* ttyfilename, unsigned long baud);
DLLIMPORT int Mobot_connectWithSerialID(mobot_t* comms, const char address[]);
DLLIMPORT int Mobot_connectChild(mobot_t* parent, mobot_t* child);
DLLIMPORT int Mobot_connectChildID(mobot_t* parent, mobot_t* child, const char* childSerialID);
DLLIMPORT int Mobot_connectWithAddress(
    mobot_t* comms, const char* address, int channel);
DLLIMPORT int Mobot_connectWithBluetoothAddress(
    mobot_t* comms, const char* address, int channel);
DLLIMPORT int Mobot_disableJointEventCallback(mobot_t* comms);
DLLIMPORT int Mobot_disableAccelEventCallback(mobot_t* comms);

/* Find the serial device the robot is plugged into. The device's
 * fully-qualified path is stored in the tty output parameter as a
 * null-terminated string. Example device paths for various OSes are
 *    \\.\COM3                Windows
 *    /dev/ttyACM0            Linux
 *    /dev/cu.usbmodem3d11    OS X
 * Mobot_dongleGetTTY will write at most len bytes to tty, including the
 * terminating null byte. Returns -1 on error, 0 on success. */
DLLIMPORT int Mobot_dongleGetTTY(char* tty, size_t len);

/* True if it is possible to flash this robot's firmware, false otherwise.
 * Under the hood, this checks to see if the robot is connected via TTY (i.e.,
 * USB). */
DLLIMPORT int Mobot_canFlashFirmware (mobot_t* comms);

/* Begin flashing the firmware of a robot connected by TTY. Parameters include
 * the hexfile filename (not the raw data), and two callbacks. Where progress
 * is measured from 0.0 to 1.0, the thread on which the flash operation runs
 * will call progressCallback every time there is a change in progress. Once
 * the operation is complete, completionCallback is called with the value of
 * 1 for success, 0 for failure. The user_data parameter gets passed to the
 * user's callbacks. */
DLLIMPORT int Mobot_flashFirmwareAsync (mobot_t* comms, const char* hexfile,
    Mobot_progressCallbackFunc progressCallback,
    Mobot_completionCallbackFunc completionCallback,
    void* userData);

DLLIMPORT int Mobot_connectWithZigbeeAddress(mobot_t* comms, uint16_t addr);
DLLIMPORT int Mobot_enableAccelEventCallback(mobot_t* comms, void* data,
    void (*accelCallback)(int millis, double x, double y, double z, void* data));
DLLIMPORT int Mobot_enableJointEventCallback(mobot_t* comms, void* data, 
    void (*jointCallback)(int millis, double j1, double j2, double j3, double j4, int mask, void* data));
DLLIMPORT int Mobot_findMobot(mobot_t* parent, const char* childSerialID);
DLLIMPORT mobotMelodyNote_t* Mobot_createMelody(int tempo);
DLLIMPORT int Mobot_melodyAddNote(mobotMelodyNote_t* melody, const char* note, int divider);
DLLIMPORT int Mobot_loadMelody(mobot_t* comms, int id, mobotMelodyNote_t* melody);
DLLIMPORT int Mobot_playMelody(mobot_t* comms, int id);
DLLIMPORT int Mobot_getAddress(mobot_t* comms);
DLLIMPORT mobot_t* Mobot_getDongle();
DLLIMPORT int Mobot_queryAddresses(mobot_t* comms);
DLLIMPORT int Mobot_clearQueriedAddresses(mobot_t* comms);
DLLIMPORT int Mobot_getQueriedAddresses(mobot_t* comms);
DLLIMPORT int Mobot_setRFChannel(mobot_t* comms, uint8_t channel);
DLLIMPORT int Mobot_getID(mobot_t* comms);
DLLIMPORT int Mobot_setID(mobot_t* comms, const char* id);
DLLIMPORT int Mobot_reboot(mobot_t* comms);
DLLIMPORT int Mobot_disconnect(mobot_t* comms);
DLLIMPORT int Mobot_driveJointToDirect(mobot_t* comms, robotJointId_t id, double angle);
DLLIMPORT int Mobot_driveJointTo(mobot_t* comms, robotJointId_t id, double angle);
DLLIMPORT int Mobot_driveJointToDirectNB(mobot_t* comms, robotJointId_t id, double angle);
DLLIMPORT int Mobot_driveJointToNB(mobot_t* comms, robotJointId_t id, double angle);
DLLIMPORT int Mobot_driveToDirect(mobot_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4);
DLLIMPORT int Mobot_driveTo(mobot_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4);
DLLIMPORT int Mobot_driveToDirectNB(mobot_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4);
DLLIMPORT int Mobot_driveToNB(mobot_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4);
DLLIMPORT int Mobot_enableButtonCallback(mobot_t* comms, void* data, void (*buttonCallback)(void* mobot, int button, int buttonDown));
DLLIMPORT int Mobot_disableButtonCallback(mobot_t* comms);
DLLIMPORT int Mobot_enableEventCallback(mobot_t* comms, 
    void (*eventCallback)(const uint8_t* buf, int size, void* userdata), void* data);
DLLIMPORT int Mobot_disableEventCallback(mobot_t* comms);
DLLIMPORT int Mobot_enableRecordDataShift(mobot_t* comms);
DLLIMPORT int Mobot_disableRecordDataShift(mobot_t* comms);
DLLIMPORT int Mobot_init(mobot_t* comms);
DLLIMPORT void Mobot_initDongle();
DLLIMPORT int Mobot_isConnected(mobot_t* comms);
DLLIMPORT int Mobot_isMoving(mobot_t* comms);
DLLIMPORT int Mobot_pair(mobot_t* mobot);
DLLIMPORT int Mobot_unpair(mobot_t* mobot);
DLLIMPORT int Mobot_protocolVersion();
#ifdef SWIG
%apply double *OUTPUT{double *accel_x, double *accel_y, double *accel_z};
%apply double *OUTPUT{double *voltage};
%apply double *OUTPUT{double *angle};
%apply double *OUTPUT{double *time,
                      double *angle1,
                      double *angle2,
                      double *angle3,
                      double *angle4};
%apply double *OUTPUT{double *angle1,
                      double *angle2,
                      double *angle3,
                      double *angle4};
%apply int* OUTPUT {int* value};
%apply int* OUTPUT {int* r, int* g, int* b};
#endif
DLLIMPORT int Mobot_getAccelerometerData(mobot_t* comms, double *accel_x, double *accel_y, double *accel_z);
DLLIMPORT int Mobot_getBatteryVoltage(mobot_t* comms, double *voltage);
DLLIMPORT int Mobot_LinkPodAnalogRead(mobot_t* comms, int adc, int* value);
DLLIMPORT int Mobot_LinkPodAnalogReadVolts(mobot_t* comms, int adc, double *volts);
DLLIMPORT int Mobot_LinkPodDigitalRead(mobot_t* comms, int pin, int *value);
DLLIMPORT int Mobot_getButtonVoltage(mobot_t* comms, double *voltage);
DLLIMPORT int Mobot_getChildrenInfo(mobot_t* comms, mobotInfo_t **mobotInfo, int *numChildren );
DLLIMPORT int Mobot_getColorRGB(mobot_t* comms, int *r, int *g, int *b);
DLLIMPORT int Mobot_getColor(mobot_t* comms, char color[]);		/* Added by Dawn 1/7/2014 */
DLLIMPORT const char* Mobot_getConfigFilePath();
DLLIMPORT int Mobot_getDistance(mobot_t* comms, double *distance, double radius);
DLLIMPORT int Mobot_getEncoderVoltage(mobot_t* comms, int pinNumber, double *voltage);
DLLIMPORT int Mobot_getFormFactor(mobot_t* comms, int* form);
DLLIMPORT int Mobot_getHWRev(mobot_t* comms, int* rev);
DLLIMPORT int Mobot_getJointAngle(mobot_t* comms, robotJointId_t id, double *angle);
DLLIMPORT int Mobot_getJointAngleAverage(mobot_t* comms, robotJointId_t id, double *angle, int numReadings);
DLLIMPORT int Mobot_getJointAnglesTime(mobot_t* comms, 
                                       double *time, 
                                       double *angle1, 
                                       double *angle2, 
                                       double *angle3, 
                                       double *angle4);
DLLIMPORT int Mobot_getJointAnglesTimeIsMoving(mobot_t* comms,
                             double *time, 
                             double *angle1,
                             double *angle2,
                             double *angle3,
                             double *angle4,
                             int *isMoving);
DLLIMPORT int Mobot_getJointAnglesTimeState(mobot_t* comms,
                             double *time, 
                             double *angle1,
                             double *angle2,
                             double *angle3,
                             double *angle4,
                             robotJointState_t* state1,
                             robotJointState_t* state2,
                             robotJointState_t* state3,
                             robotJointState_t* state4);
DLLIMPORT int Mobot_getJointAngles(mobot_t* comms, 
                                       double *angle1, 
                                       double *angle2, 
                                       double *angle3, 
                                       double *angle4);
DLLIMPORT int Mobot_getJointAnglesAverage(mobot_t* comms, 
                                       double *angle1, 
                                       double *angle2, 
                                       double *angle3, 
                                       double *angle4,
                                       int numReadings);
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
DLLIMPORT int Mobot_getPoseData(mobot_t* comms, uint8_t index, double *angle1, double *angle2, double *angle3, double *angle4);
DLLIMPORT int Mobot_getMasterAddress(mobot_t* comms, uint16_t* addr);
DLLIMPORT int Mobot_getNumPoses(mobot_t* comms, int* num);
DLLIMPORT int Mobot_getNumSlaves(mobot_t* comms, int* num);
DLLIMPORT int Mobot_getSlaveAddr(mobot_t* comms, uint8_t index, uint16_t* addr);
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
DLLIMPORT int Mobot_moveBackward(mobot_t* comms, double angle);
DLLIMPORT int Mobot_moveBackwardNB(mobot_t* comms, double angle);
DLLIMPORT int Mobot_moveDistance(mobot_t* comms, double distance, double radius);
DLLIMPORT int Mobot_moveDistanceNB(mobot_t* comms, double distance, double radius);
DLLIMPORT int Mobot_moveForward(mobot_t* comms, double angle);
DLLIMPORT int Mobot_moveForwardNB(mobot_t* comms, double angle);
DLLIMPORT int Mobot_moveJoint(mobot_t* comms, robotJointId_t id, double angle);
DLLIMPORT int Mobot_moveJointNB(mobot_t* comms, robotJointId_t id, double angle);
DLLIMPORT int Mobot_moveJointContinuousNB(mobot_t* comms, robotJointId_t id, robotJointState_t dir);
DLLIMPORT int Mobot_moveJointContinuousTime(mobot_t* comms, 
                                            robotJointId_t id, 
                                            robotJointState_t dir, 
                                            double seconds);
DLLIMPORT int Mobot_moveJointTo(mobot_t* comms, robotJointId_t id, double angle);
DLLIMPORT int Mobot_moveJointToDirect(mobot_t* comms, robotJointId_t id, double angle);
DLLIMPORT int Mobot_moveJointToNB(mobot_t* comms, robotJointId_t id, double angle);
DLLIMPORT int Mobot_moveJointToDirectNB(mobot_t* comms, robotJointId_t id, double angle);
DLLIMPORT int Mobot_moveJointWait(mobot_t* comms, robotJointId_t id);
DLLIMPORT int Mobot_moveTo(mobot_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4);
DLLIMPORT int Mobot_moveToDirect(mobot_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4);
DLLIMPORT int Mobot_moveToNB(mobot_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4);
DLLIMPORT int Mobot_moveToDirectNB(mobot_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4);
DLLIMPORT int Mobot_moveToZero(mobot_t* comms);
DLLIMPORT int Mobot_moveToZeroNB(mobot_t* comms);
DLLIMPORT int Mobot_moveWait(mobot_t* comms);
DLLIMPORT int Mobot_movexy(mobot_t* comms, double x, double y, double radius, double trackwidth);
DLLIMPORT void* Mobot_movexyThread(void*);
DLLIMPORT int Mobot_movexyNB(mobot_t* comms, double x, double y, double radius, double trackwidth);
DLLIMPORT int Mobot_recordAngle(mobot_t* comms, 
                                robotJointId_t id, 
                                double* time, 
                                double* angle, 
                                int num, 
                                double timeInterval,
                                int shiftData);
DLLIMPORT int Mobot_recordAngleBegin(mobot_t* comms,
                                     robotJointId_t id,
                                     double **time,
                                     double **angle,
                                     double timeInterval, 
                                     int shiftData);
DLLIMPORT int Mobot_recordAngleEnd(mobot_t* comms, robotJointId_t id, int *num);
DLLIMPORT int Mobot_recordAngles(mobot_t* comms, 
                                 double *time, 
                                 double* angle1, 
                                 double* angle2,
                                 double* angle3,
                                 double* angle4,
                                 int num,
                                 double timeInterval,
                                 int shiftData);
DLLIMPORT int Mobot_recordAnglesBegin(mobot_t* comms,
                                     double **time,
                                     double **angle1,
                                     double **angle2,
                                     double **angle3,
                                     double **angle4,
                                     double timeInterval,
                                     int shiftData);
DLLIMPORT int Mobot_recordAnglesEnd(mobot_t* comms, int* num);
DLLIMPORT int Mobot_recordDistanceBegin(mobot_t* comms,
                                     robotJointId_t id,
                                     double **time,
                                     double **distance,
                                     double radius,
                                     double timeInterval, 
                                     int shiftData);
DLLIMPORT int Mobot_recordDistanceEnd(mobot_t* comms, robotJointId_t id, int *num);
DLLIMPORT int Mobot_recordDistanceOffset(mobot_t* comms, double distance);
DLLIMPORT int Mobot_recordDistancesBegin(mobot_t* comms,
                               double **time,
                               double **distance1,
                               double **distance2,
                               double **distance3,
                               double **distance4,
                               double radius, 
                               double timeInterval,
                               int shiftData);
DLLIMPORT int Mobot_recordDistancesEnd(mobot_t* comms, int *num);
DLLIMPORT int Mobot_recordWait(mobot_t* comms);
DLLIMPORT int Mobot_registerScanCallback(mobot_t* comms, void (*cb) (const char* id));
DLLIMPORT int Mobot_reset(mobot_t* comms);
DLLIMPORT int Mobot_resetToZero(mobot_t* comms);
DLLIMPORT int Mobot_resetToZeroNB(mobot_t* comms);
DLLIMPORT int Mobot_setAccelEventThreshold(mobot_t* comms, double threshold);
DLLIMPORT int Mobot_setJointEventThreshold(mobot_t* comms, int joint, double threshold);
DLLIMPORT int Mobot_setExitState(mobot_t* comms, robotJointState_t exitState);
DLLIMPORT int Mobot_setFourierCoefficients(mobot_t* comms, robotJointId_t id, double* a, double* b);
DLLIMPORT int Mobot_beginFourierControl(mobot_t* comms, uint8_t motorMask);
DLLIMPORT int Mobot_setHWRev(mobot_t* comms, uint8_t rev);
DLLIMPORT int Mobot_LinkPodAnalogWrite(mobot_t* comms, int pin, int value);
DLLIMPORT int Mobot_LinkPodAnalogReference(mobot_t* comms, int ref);
DLLIMPORT int Mobot_LinkPodDigitalWrite(mobot_t* comms, int pin, int value);
DLLIMPORT int Mobot_LinkPodPinMode(mobot_t* comms, int pin, int mode);
DLLIMPORT int Mobot_setBuzzerFrequency(mobot_t* comms, unsigned int frequency, double time);
DLLIMPORT int Mobot_setBuzzerFrequencyOn(mobot_t* comms, unsigned int frequency);
DLLIMPORT int Mobot_setBuzzerFrequencyOff(mobot_t* comms);
DLLIMPORT int Mobot_setDongleMobot(mobot_t* comms);
DLLIMPORT int Mobot_setJointDirection(mobot_t* comms, robotJointId_t id, robotJointState_t dir);
DLLIMPORT int Mobot_setJointMovementStateNB(mobot_t* comms, robotJointId_t id, robotJointState_t dir);
DLLIMPORT int Mobot_setJointMovementStateTime(mobot_t* comms, robotJointId_t id, robotJointState_t dir, double seconds);
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
DLLIMPORT int Mobot_setMovementStateNB(mobot_t* comms,
                                  robotJointState_t dir1,
                                  robotJointState_t dir2,
                                  robotJointState_t dir3,
                                  robotJointState_t dir4);
DLLIMPORT int Mobot_setMovementStateTime(mobot_t* comms,
                                  robotJointState_t dir1,
                                  robotJointState_t dir2,
                                  robotJointState_t dir3,
                                  robotJointState_t dir4,
                                  double seconds);
DLLIMPORT int Mobot_setMovementStateTimeNB(mobot_t* comms,
                                  robotJointState_t dir1,
                                  robotJointState_t dir2,
                                  robotJointState_t dir3,
                                  robotJointState_t dir4,
                                  double seconds);
DLLIMPORT int Mobot_setColorRGB(mobot_t* comms, int r, int g, int b);
DLLIMPORT int Mobot_setColor(mobot_t* comms, char * color);		/* Added by Dawn 1/7/2014 */
DLLIMPORT int Mobot_setTwoWheelRobotSpeed(mobot_t* comms, double speed, double radius);
DLLIMPORT int Mobot_smoothMoveToNB(
    mobot_t* comms, 
    robotJointId_t id, 
    double accel0, 
    double accelf, 
    double vmax, 
    double angle);
DLLIMPORT int Mobot_stop(mobot_t* comms);
DLLIMPORT int Mobot_stopOneJoint(mobot_t* comms, robotJointId_t id);
DLLIMPORT int Mobot_stopTwoJoints(mobot_t* comms, robotJointId_t id1, robotJointId_t id2);
DLLIMPORT int Mobot_stopThreeJoints(mobot_t* comms, robotJointId_t id1, robotJointId_t id2, robotJointId_t id3);
DLLIMPORT int Mobot_stopAllJoints(mobot_t* comms);
DLLIMPORT int Mobot_turnLeft(mobot_t* comms, double angle, double radius, double tracklength);
DLLIMPORT int Mobot_turnLeftNB(mobot_t* comms, double angle, double radius, double tracklength);
DLLIMPORT int Mobot_turnRight(mobot_t* comms, double angle, double radius, double tracklength);
DLLIMPORT int Mobot_turnRightNB(mobot_t* comms, double angle, double radius, double tracklength);
DLLIMPORT int Mobot_twiRecv(mobot_t* comms, uint8_t addr, void* recvbuf, int size);
DLLIMPORT int Mobot_twiSend(mobot_t* comms, uint8_t addr, uint8_t* buf, int size);
DLLIMPORT int Mobot_twiSendRecv(mobot_t* comms, uint8_t addr, 
    uint8_t* sendbuf, int sendsize,
    void* recvbuf, int recvsize);

/* compound motion functions */
DLLIMPORT int Mobot_motionArch(mobot_t* comms, double angle);
DLLIMPORT int Mobot_motionDistance(mobot_t* comms, double distance, double radius);
DLLIMPORT int Mobot_motionDistanceNB(mobot_t* comms, double distance, double radius);
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

/* Mobot internal helper functions */
DLLIMPORT int SendToIMobot(mobot_t* comms, uint8_t cmd, const void* data, int datasize);
//DLLIMPORT int SendToMobotDirect(mobot_t* comms, const void* data, int datasize);
DLLIMPORT int MobotMsgTransaction(mobot_t* comms, uint8_t cmd, /*IN&OUT*/ void* buf, int sendsize);
DLLIMPORT int RecvFromIMobot(mobot_t* comms, uint8_t* buf, int size);

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
int shiftDataIsEnabled(mobot_t* comms);
DLLIMPORT double systemTime();
#ifdef __cplusplus
}
#endif

#ifdef _CH_
extern void delay(double seconds);
#endif

#ifdef _CH_
void * CMobot::g_chmobot_dlhandle = NULL;
int CMobot::g_chmobot_dlcount = 0;
#pragma importf "chmobot.chf"
#endif

#ifndef NONRELEASE
#undef DLLIMPORT
#undef THREAD_T
#undef MUTEX_T
#undef COND_T
#undef sockaddr_t
#endif

#endif /* Header Guard */
