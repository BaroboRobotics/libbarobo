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

typedef struct br_comms_s
{
  sockaddr_t addr;
  int socket;
} br_comms_t;

#ifdef __cplusplus
extern "C" {
#endif

int BRComms_init(br_comms_t* comms);
int BRComms_connect(br_comms_t* comms, const char* address, int channel);
int BRComms_disconnect(br_comms_t* comms);
int BRComms_setMotorDirection(br_comms_t* comms, int id, int dir);
int BRComms_getMotorDirection(br_comms_t* comms, int id, int *dir);
int BRComms_setMotorSpeed(br_comms_t* comms, int id, int speed);
int BRComms_getMotorSpeed(br_comms_t* comms, int id, int *speed);
int BRComms_setMotorPosition(br_comms_t* comms, int id, int pos);
int BRComms_getMotorPosition(br_comms_t* comms, int id, int *pos);
int BRComms_stop(br_comms_t* comms);
#ifdef _WIN32
void baswap(bdaddr_t *dst, const bdaddr_t *src);
int str2ba(const char *str, bdaddr_t *ba);
#endif

#ifdef __cplusplus
}
#endif
#endif
