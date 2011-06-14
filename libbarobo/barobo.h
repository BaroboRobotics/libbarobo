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

#define I2C_HC_ADDR 0x55

#ifdef __cplusplus
extern "C" {
#endif

typedef struct iMobot_s {
  int i2cDev;
  unsigned short enc[4];
  int socket;
} iMobot_t;

int BR_init(iMobot_t* iMobot);
int BR_initListenerBluetooth(iMobot_t* iMobot, int channel);
int BR_pose(iMobot_t* iMobot, unsigned short enc[4], const char motorMask);
int BR_poseJoint(iMobot_t* iMobot, unsigned short id, unsigned short enc);
int BR_move(iMobot_t* iMobot, short enc[4], const char motorMask);
int BR_moveRelative(iMobot_t* iMobot, double angle[4], const char motorMask);
int BR_moveJoint(iMobot_t* iMobot, double angle, int motorNumber);
int BR_moveJointRelative(iMobot_t* iMobot, double angle, int motorNumber);
int BR_stop(iMobot_t* iMobot);
int BR_moveWait(iMobot_t* iMobot);
int BR_isBusy(iMobot_t* iMobot);
int BR_getJointAngle(iMobot_t* iMobot, int id, double* angle);
int BR_getJointAngles(iMobot_t* iMobot, double angle[4]);
int BR_setSpeed(iMobot_t* iMobot, double speed[4], const char motorMask);
int BR_getSpeed(iMobot_t* iMobot, double speed[4]);
int BR_listenerMainLoop(iMobot_t* iMobot);
int BR_slaveProcessCommand(iMobot_t* iMobot, int socket, int bytesRead, const char* buf);
int BR_terminate(iMobot_t* iMobot);



#ifdef __cplusplus
}
#endif

#endif
