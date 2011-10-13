#ifndef _IMOBOTCOMMS_H_
#define _IMOBOTCOMMS_H_

#ifdef _CH_
#pragma package <chbarobo>
#ifdef _WIN32_
#define _WIN32
#define UINT8 uint8_t
#endif

#endif

#include <stdint.h>
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

#ifndef IMOBOT_JOINTS_E
#define IMOBOT_JOINTS_E
enum iMobot_joints_e {
  IMOBOT_JOINT1 = 0,
  IMOBOT_JOINT2,
  IMOBOT_JOINT3,
  IMOBOT_JOINT4,
  IMOBOT_NUM_JOINTS 
};
#endif

#ifndef IMOBOT_JOINT_DIRECTION_E
#define IMOBOT_JOINT_DIRECTION_E
enum iMobot_motor_direction_e
{
  IMOBOT_JOINT_DIR_AUTO,
  IMOBOT_JOINT_DIR_FORWARD,
  IMOBOT_JOINT_DIR_BACKWARD
};
#endif

#ifndef IMOBOT_JOINT_STATE_E
#define IMOBOT_JOINT_STATE_E
enum iMobot_joint_state_e
{
    IMOBOT_JOINT_IDLE = 0,
    IMOBOT_JOINT_MOVING,
    IMOBOT_JOINT_GOALSEEK,
};
#endif

typedef struct br_comms_s
{
  int socket;
  int connected;
#ifdef _WIN32
  HANDLE hSerial;
#endif
  sockaddr_t addr;
} br_comms_t;

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

DLLIMPORT int iMobotComms_init(br_comms_t* comms);
DLLIMPORT int iMobotComms_connect(br_comms_t* comms);
DLLIMPORT int iMobotComms_connectAddress(
    br_comms_t* comms, const char* address, int channel);
DLLIMPORT int iMobotComms_disconnect(br_comms_t* comms);
DLLIMPORT int iMobotComms_isConnected(br_comms_t* comms);
DLLIMPORT int iMobotComms_setJointDirection(br_comms_t* comms, int id, int dir);
DLLIMPORT int iMobotComms_getJointDirection(br_comms_t* comms, int id, int *dir);
DLLIMPORT int iMobotComms_setJointSpeed(br_comms_t* comms, int id, double speed);
DLLIMPORT int iMobotComms_getJointSpeed(br_comms_t* comms, int id, double *speed);
DLLIMPORT int iMobotComms_moveJointTo(br_comms_t* comms, int id, double angle);
DLLIMPORT int iMobotComms_getJointAngle(br_comms_t* comms, int id, double *angle);
DLLIMPORT int iMobotComms_getJointState(br_comms_t* comms, int id, int *state);
DLLIMPORT int iMobotComms_move(br_comms_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4);
DLLIMPORT int iMobotComms_moveTo(br_comms_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4);
DLLIMPORT int iMobotComms_moveZero(br_comms_t* comms);
DLLIMPORT int iMobotComms_moveJointWait(br_comms_t* comms, int id);
DLLIMPORT int iMobotComms_moveWait(br_comms_t* comms);
DLLIMPORT int iMobotComms_stop(br_comms_t* comms);

/* compound motion functions */
DLLIMPORT int iMobotComms_motionInchwormLeft(br_comms_t* comms);
DLLIMPORT int iMobotComms_motionInchwormRight(br_comms_t* comms);
DLLIMPORT int iMobotComms_motionRollBackward(br_comms_t* comms);
DLLIMPORT int iMobotComms_motionRollForward(br_comms_t* comms);
DLLIMPORT int iMobotComms_motionStand(br_comms_t* comms);
DLLIMPORT int iMobotComms_motionTurnLeft(br_comms_t* comms);
DLLIMPORT int iMobotComms_motionTurnRight(br_comms_t* comms);

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
class CiMobotComms {
  public:
    CiMobotComms();
    CiMobotComms(const char address[], int channel);
    ~CiMobotComms();
    int connect();
    int connectAddress(const char address[], int channel);
    int disconnect();
    int isConnected();
    int getJointAngle(int id, double &angle);
    int getJointDirection(int id, int &dir);
    int getJointSpeed(int id, double &speed);
    int getJointState(int id, int &state);
    int move(double angle1, double angle2, double angle3, double angle4);
    int moveJointTo(int id, double angle);
    int moveJointWait(int id);
    int moveTo(double angle1, double angle2, double angle3, double angle4);
    int moveWait();
    int moveZero();
    int setJointDirection(int id, int dir);
    int setJointSpeed(int id, double speed);
    int stop();

    int motionInchwormLeft();
    int motionInchwormRight();
    int motionRollBackward();
    int motionRollForward();
    int motionStand();
    int motionTurnLeft();
    int motionTurnRight();
  private:
    br_comms_t _comms;
};
#endif /* If C++ or CH */
#endif /* C_ONLY */

#ifdef _CH_
#pragma importf "imobotcomms.cpp"
#endif

#endif /* Header Guard */
