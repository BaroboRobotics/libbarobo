#ifndef _BAROBO_H_
#define _BAROBO_H_

#ifdef _CH_
#pragma package <chimobot>
#endif

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

typedef struct iMobot_s {
  int i2cDev;
  short enc[4];
  int socket;
} iMobot_t;

int iMobot_getJointDirection(iMobot_t* iMobot, int id, int *dir);
int iMobot_getJointAngle(iMobot_t* iMobot, int id, double *angle);
int iMobot_getJointSpeed(iMobot_t* iMobot, int id, int *speed);
int iMobot_getJointState(iMobot_t* iMobot, int id, int *state);
int iMobot_init(iMobot_t* iMobot);
int iMobot_initListenerBluetooth(iMobot_t* iMobot, int channel);
int iMobot_isBusy(iMobot_t* iMobot);
int iMobot_listenerMainLoop(iMobot_t* iMobot);
int iMobot_moveWait(iMobot_t* iMobot);
int iMobot_moveToZero(iMobot_t* iMobot);
int iMobot_setJointDirection(iMobot_t* iMobot, int id, int direction);
int iMobot_moveJointTo(iMobot_t* iMobot, int id, double angle);
int iMobot_setJointSpeed(iMobot_t* iMobot, int id, int speed);
int iMobot_stop(iMobot_t* iMobot);
int iMobot_terminate(iMobot_t* iMobot);
int iMobot_waitMotor(iMobot_t* iMobot, int id);

int iMobot_slaveProcessCommand(iMobot_t* iMobot, int socket, int bytesRead, const char* buf);

#ifdef __cplusplus
}
#endif

/* The C++ interface for the imobot class */
#if defined (__cplusplus) || defined (_CH_)
class CiMobot {
  public:
    CiMobot();
    ~CiMobot();
    int getJointDirection(int id, int &direction);
    int getJointAngle(int id, double &angle);
    int getJointSpeed(int id, int &speed);
    int getJointState(int id, int &state);
    int initListenerBluetooth(int channel);
    int isBusy();
    int listenerMainLoop();
    int moveWait();
    int moveToZero();
    int setJointDirection(int id, int direction);
    int moveJointTo(int id, double angle);
    int setJointSpeed(int id, int speed);
    int stop();
    int terminate();
    int waitMotor(int id);
  private:
    iMobot_t _iMobot;
};
#endif

#ifdef _CH_
#pragma importf <imobot.cpp>
#endif

#endif
