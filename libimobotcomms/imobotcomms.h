#ifndef _IMOBOTCOMMS_H_
#define _IMOBOTCOMMS_H_

#ifndef _WIN32
#include <stdint.h>
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
typedef unsigned char uint8_t;
#endif

#define DEGREE2ENC(angles, enc) \
enc[0] = angles[0] * 10; \
enc[1] = angles[1] * 10; \
enc[2] = angles[2] * 10; \
enc[3] = angles[3] * 10

#ifndef IMOBOT_MOTORS_E
#define IMOBOT_MOTORS_E
enum iMobot_motors_e {
  IMOBOT_MOTOR1 = 0,
  IMOBOT_MOTOR2,
  IMOBOT_MOTOR3,
  IMOBOT_MOTOR4,
  IMOBOT_NUM_MOTORS 
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
#define DLLIMPORT __declspec(dllexport)
#else
#define DLLIMPORT
#endif

DLLIMPORT int iMobotComms_init(br_comms_t* comms);
DLLIMPORT int iMobotComms_connect(br_comms_t* comms);
DLLIMPORT int iMobotComms_connectAddress(
    br_comms_t* comms, const char* address, int channel);
DLLIMPORT int iMobotComms_disconnect(br_comms_t* comms);
DLLIMPORT int iMobotComms_isConnected(br_comms_t* comms);
DLLIMPORT int iMobotComms_setMotorDirection(br_comms_t* comms, int id, int dir);
DLLIMPORT int iMobotComms_getMotorDirection(br_comms_t* comms, int id, int *dir);
DLLIMPORT int iMobotComms_setMotorSpeed(br_comms_t* comms, int id, int speed);
DLLIMPORT int iMobotComms_getMotorSpeed(br_comms_t* comms, int id, int *speed);
DLLIMPORT int iMobotComms_setMotorPosition(br_comms_t* comms, int id, double position);
DLLIMPORT int iMobotComms_getMotorPosition(br_comms_t* comms, int id, double *position);
DLLIMPORT int iMobotComms_getMotorState(br_comms_t* comms, int id, int *state);
DLLIMPORT int iMobotComms_poseZero(br_comms_t* comms);
DLLIMPORT int iMobotComms_waitMotor(br_comms_t* comms, int id);
DLLIMPORT int iMobotComms_moveWait(br_comms_t* comms);
DLLIMPORT int iMobotComms_stop(br_comms_t* comms);
#ifdef _WIN32
typedef struct bdaddr_s {
  UINT8 b[6];
} bdaddr_t;
int str2ba(const char *str, bdaddr_t *ba);
#define write(sock, buf, len) send(sock, buf, len, 0)
#define read(sock, buf, len) \
  recvfrom(sock, buf, len, 0, (struct sockaddr*)0, 0)
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
    ~CiMobotComms();
    int connect();
    int connectAddress(const char* address, int channel);
    int disconnect();
    int isConnected();
    int getMotorDirection(int id, int &dir);
    int getMotorPosition(int id, double &position);
    int getMotorSpeed(int id, int &speed);
    int getMotorState(int id, int &state);
    int moveWait();
    int poseZero();
    int setMotorDirection(int id, int dir);
    int setMotorPosition(int id, double position);
    int setMotorSpeed(int id, int speed);
    int stop();
    int waitMotor(int id);
  private:
    br_comms_t _comms;
};
#endif
#endif

#endif
