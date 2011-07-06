#ifndef _BAROBO_H_
#define _BAROBO_H_

/* Motor home position, 2 bytes, R/W */
#define I2C_REG_MOTORHOME0 0x30
#define I2C_REG_MOTORHOME1 0x40
#define I2C_REG_MOTORHOME2 0x50
#define I2C_REG_MOTORHOME3 0x60

/* Motor position register, 2 bytes, R/W */
#define I2C_REG_MOTORPOS0 0x32
#define I2C_REG_MOTORPOS1 0x42
#define I2C_REG_MOTORPOS2 0x52
#define I2C_REG_MOTORPOS3 0x62
#define I2C_REG_MOTORPOS(m) \
  0x32 + 0x10*(m)

/* Motor direction register, 1 byte*/
#define I2C_REG_MOTORDIR0 0x34
#define I2C_REG_MOTORDIR1 0x44
#define I2C_REG_MOTORDIR2 0x54
#define I2C_REG_MOTORDIR3 0x64
#define I2C_REG_MOTORDIR(m) \
  0x34 + 0x10*(m)

/* Motor speed register, 1 byte*/
#define I2C_REG_MOTORSPEED0 0x35
#define I2C_REG_MOTORSPEED1 0x45
#define I2C_REG_MOTORSPEED2 0x55
#define I2C_REG_MOTORSPEED3 0x65
#define I2C_REG_MOTORSPEED(m) \
  0x35 + 0x10*(m)

#define I2C_REG_MOTORSTATE0 0x37
#define I2C_REG_MOTORSTATE1 0x47
#define I2C_REG_MOTORSTATE2 0x57
#define I2C_REG_MOTORSTATE3 0x67
#define I2C_REG_MOTORSTATE(m) \
  0x37 + 0x10*(m)

#define I2C_HC_ADDR 0x55

#ifdef _CH_
#pragma package <chi2cio>
#pragma package <chbluetooth>
#include <i2c-io-api.h>
#include <i2c.h>
#include <i2c-api.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#include "libi2c/i2c-api.h"

typedef struct iMobot_s {
  int i2cDev;
  short enc[4];
  int socket;
} iMobot_t;

int BR_init(iMobot_t* iMobot);
int BR_initListenerBluetooth(iMobot_t* iMobot, int channel);
int BR_pose(iMobot_t* iMobot, double angles[4], const char motorMask);
int BR_poseJoint(iMobot_t* iMobot, unsigned short id, double angle);
int BR_move(iMobot_t* iMobot, double angles[4], const char motorMask);
int BR_stop(iMobot_t* iMobot);
int BR_setMotorDirection(iMobot_t* iMobot, int id, unsigned short direction);
int BR_setMotorDirections(iMobot_t* iMobot, unsigned short dir[4], const char motorMask);
int BR_moveWait(iMobot_t* iMobot);
int BR_isBusy(iMobot_t* iMobot);
int BR_getJointAngle(iMobot_t* iMobot, int id, double* angle);
int BR_getJointAngles(iMobot_t* iMobot, double angle[4]);
int BR_setMotorPosition(iMobot_t* iMobot, int id, double angle);
int BR_getMotorPosition(iMobot_t* iMobot, int id, double* angle);
int BR_setMotorSpeed(iMobot_t* iMobot, int id, unsigned short speed);
int BR_getMotorSpeed(iMobot_t* iMobot, int id, unsigned short* speed);
int BR_getMotorState(iMobot_t* iMobot, int id, unsigned short* state);
int BR_waitMotor(iMobot_t* iMobot, int id);
int BR_listenerMainLoop(iMobot_t* iMobot);
int BR_slaveProcessCommand(iMobot_t* iMobot, int socket, int bytesRead, const char* buf);
int BR_terminate(iMobot_t* iMobot);



#ifdef __cplusplus
}
#endif

/* The C++ interface for the imobot class */
#ifdef __cplusplus
class CiMobot {
  public:
    CiMobot();
    ~CiMobot();
    int initListenerBluetooth(int channel);
    int pose(double angles[4], const char motorMask);
    int poseJoint(unsigned short id, double angle);
    int move(double angles[4], const char motorMask);
    int stop();
    int setMotorDirection(int id, unsigned short direction);
    int setMotorDirections(unsigned short dir[4], const char motorMask);
    int moveWait();
    int isBusy();
    int getJointAngle(int id, double* angle);
    int getJointAngles(double angle[4]);
    int setMotorPosition(int id, double angle);
    int getMotorPosition(int id, double* angle);
    int setMotorSpeed(int id, unsigned short speed);
    int getMotorSpeed(int id, unsigned short* speed);
    int getMotorState(int id, unsigned short* state);
    int waitMotor(int id);
    int listenerMainLoop();
    int terminate();
  private:
    iMobot_t _iMobot;
};
#endif

#ifdef _CH_
#pragma importf "imobot.cpp"
#endif

#endif
