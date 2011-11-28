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
  0x32 + 0x10*(m-1)

/* Motor direction register, 1 byte*/
#define I2C_REG_MOTORDIR0 0x34
#define I2C_REG_MOTORDIR1 0x44
#define I2C_REG_MOTORDIR2 0x54
#define I2C_REG_MOTORDIR3 0x64
#define I2C_REG_MOTORDIR(m) \
  0x34 + 0x10*(m-1)

/* Motor speed register, 1 byte*/
#define I2C_REG_MOTORSPEED0 0x35
#define I2C_REG_MOTORSPEED1 0x45
#define I2C_REG_MOTORSPEED2 0x55
#define I2C_REG_MOTORSPEED3 0x65
#define I2C_REG_MOTORSPEED(m) \
  0x35 + 0x10*(m-1)

#define I2C_REG_MOTORSTATE0 0x38
#define I2C_REG_MOTORSTATE1 0x48
#define I2C_REG_MOTORSTATE2 0x58
#define I2C_REG_MOTORSTATE3 0x68
#define I2C_REG_MOTORSTATE(m) \
  0x38 + 0x10*(m-1)

#define I2C_HC_ADDR 0x55

#ifdef __cplusplus
extern "C" {
#endif

#ifndef IMOBOT_MOTORS_E
#define IMOBOT_MOTORS_E
typedef enum iMobot_joints_e {
  IMOBOT_JOINT1 = 1,
  IMOBOT_JOINT2,
  IMOBOT_JOINT3,
  IMOBOT_JOINT4,
  IMOBOT_NUM_JOINTS = 4
}iMobotJointId_t;
#endif

typedef enum iMobot_joint_direction_e {
  IMOBOT_NEUTRAL = 0,
  IMOBOT_FORWARD,
  IMOBOT_BACKWARD
} iMobotJointDirection_t;

typedef enum iMobot_joint_state_e {
  IMOBOT_JOINT_IDLE = 0,
  IMOBOT_JOINT_MOVING,
  IMOBOT_JOINT_GOALSEEK
} iMobotJointState_t;

typedef struct iMobot_s {
  int i2cDev;
  short enc[4];
  int socket;
  double jointSpeed[4];
} iMobot_t;

int iMobot_getJointAngle(iMobot_t* iMobot, iMobotJointId_t id, double *angle);
int iMobot_getJointDirection(iMobot_t* iMobot, iMobotJointId_t id, iMobotJointDirection_t *dir);
int iMobot_getJointSpeed(iMobot_t* iMobot, iMobotJointId_t id, double *speed);
int iMobot_getJointState(iMobot_t* iMobot, iMobotJointId_t id, iMobotJointState_t *state);
int iMobot_init(iMobot_t* iMobot);
int iMobot_initListenerBluetooth(iMobot_t* iMobot, int channel);
int iMobot_isBusy(iMobot_t* iMobot);
int iMobot_listenerMainLoop(iMobot_t* iMobot);
int iMobot_move(iMobot_t* iMobot,
                double angle1,
                double angle2,
                double angle3,
                double angle4 );
int iMobot_moveNB(iMobot_t* iMobot,
                double angle1,
                double angle2,
                double angle3,
                double angle4 );
int iMobot_moveContinuous(iMobot_t* iMobot,
                                  iMobotJointDirection_t dir1,
                                  iMobotJointDirection_t dir2,
                                  iMobotJointDirection_t dir3,
                                  iMobotJointDirection_t dir4);
int iMobot_moveContinuousTime(iMobot_t* comms,
                                  iMobotJointDirection_t dir1,
                                  iMobotJointDirection_t dir2,
                                  iMobotJointDirection_t dir3,
                                  iMobotJointDirection_t dir4,
                                  int msecs);
int iMobot_moveTo(iMobot_t* iMobot,
                double angle1,
                double angle2,
                double angle3,
                double angle4 );
int iMobot_moveToNB(iMobot_t* iMobot,
                double angle1,
                double angle2,
                double angle3,
                double angle4 );
int iMobot_moveJoint(iMobot_t* iMobot, iMobotJointId_t id, double angle);
int iMobot_moveJointNB(iMobot_t* iMobot, iMobotJointId_t id, double angle);
int iMobot_moveJointTo(iMobot_t* iMobot, iMobotJointId_t id, double angle);
int iMobot_moveJointToNB(iMobot_t* iMobot, iMobotJointId_t id, double angle);
int iMobot_moveToZero(iMobot_t* iMobot);
int iMobot_moveToZeroNB(iMobot_t* iMobot);
int iMobot_moveWait(iMobot_t* iMobot);
int iMobot_setJointDirection(iMobot_t* iMobot, iMobotJointId_t id, iMobotJointDirection_t direction);
int iMobot_setJointSpeed(iMobot_t* iMobot, iMobotJointId_t id, double speed);
int iMobot_stop(iMobot_t* iMobot);
int iMobot_terminate(iMobot_t* iMobot);
int iMobot_moveJointWait(iMobot_t* iMobot, iMobotJointId_t id);

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
    int getJointAngle(iMobotJointId_t id, double &angle);
    int getJointSpeed(iMobotJointId_t id, double &speed);
    int getJointState(iMobotJointId_t id, iMobotJointState_t &state);
    int initListenerBluetooth(int channel);
    int isBusy();
    int listenerMainLoop();
    int move( double angle1,
              double angle2,
              double angle3,
              double angle4 );
    int moveNB( double angle1,
              double angle2,
              double angle3,
              double angle4 );
    int moveTo( double angle1,
              double angle2,
              double angle3,
              double angle4 );
    int moveToNB( double angle1,
              double angle2,
              double angle3,
              double angle4 );
    int moveJoint(iMobotJointId_t id, double angle);
    int moveJointNB(iMobotJointId_t id, double angle);
    int moveJointTo(iMobotJointId_t id, double angle);
    int moveJointToNB(iMobotJointId_t id, double angle);
    int moveWait();
    int moveToZero();
    int setJointSpeed(iMobotJointId_t id, double speed);
    int stop();
    int terminate();
    int moveJointWait(iMobotJointId_t id);
#ifndef _CH_
  private:
    iMobot_t _iMobot;
#else
  private:
    int getJointDirection(iMobotJointId_t id, iMobotJointDirection_t &direction);
    int setJointDirection(iMobotJointId_t id, iMobotJointDirection_t direction);
    static void *g_chimobot_dlhandle;
    static int g_chimobot_dlcount;
#endif
};
#endif

#ifdef _CH_
void * CiMobot::g_chimobot_dlhandle = NULL;
int CiMobot::g_chimobot_dlcount = 0;
#pragma importf "chimobot.chf"
#endif

#endif
