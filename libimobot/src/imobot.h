#ifndef _BAROBO_H_
#define _BAROBO_H_
#include <math.h>

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

#define MAXSPEED (M_PI/4.0)

#ifndef ROBOT_JOINTS_E
#define ROBOT_JOINTS_E
typedef enum robotJoints_e {
  ROBOT_JOINT1 = 1,
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
    ROBOT_JOINT_IDLE = 0,
    ROBOT_JOINT_MOVING,
    ROBOT_JOINT_GOALSEEK,
} robotJointState_t;
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

#ifdef __cplusplus
extern "C" {
#endif

typedef struct iMobot_s {
  int i2cDev;
  short enc[4];
  int socket;
  double jointSpeed[4];
  double jointMaxSpeed[4];
} iMobot_t;

int iMobot_getJointAngle(iMobot_t* iMobot, robotJointId_t id, double *angle);
int iMobot_getJointDirection(iMobot_t* iMobot, robotJointId_t id, robotJointState_t *dir);
int iMobot_getJointMaxSpeed(iMobot_t* iMobot, robotJointId_t id, double *maxSpeed);
int iMobot_getJointSpeed(iMobot_t* iMobot, robotJointId_t id, double *speed);
int iMobot_getJointState(iMobot_t* iMobot, robotJointId_t id, robotJointState_t *state);
int iMobot_init(iMobot_t* iMobot);
int iMobot_initListenerBluetooth(iMobot_t* iMobot, int channel);
int iMobot_isBusy(iMobot_t* iMobot);
int iMobot_isMoving(iMobot_t* iMobot);
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
                                  robotJointState_t dir1,
                                  robotJointState_t dir2,
                                  robotJointState_t dir3,
                                  robotJointState_t dir4);
int iMobot_moveContinuousTime(iMobot_t* comms,
                                  robotJointState_t dir1,
                                  robotJointState_t dir2,
                                  robotJointState_t dir3,
                                  robotJointState_t dir4,
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
int iMobot_moveJoint(iMobot_t* iMobot, robotJointId_t id, double angle);
int iMobot_moveJointNB(iMobot_t* iMobot, robotJointId_t id, double angle);
int iMobot_moveJointTo(iMobot_t* iMobot, robotJointId_t id, double angle);
int iMobot_moveJointToNB(iMobot_t* iMobot, robotJointId_t id, double angle);
int iMobot_moveToZero(iMobot_t* iMobot);
int iMobot_moveToZeroNB(iMobot_t* iMobot);
int iMobot_moveWait(iMobot_t* iMobot);
int iMobot_setJointDirection(iMobot_t* iMobot, robotJointId_t id, robotJointState_t direction);
int iMobot_setJointSpeed(iMobot_t* iMobot, robotJointId_t id, double speed);
int iMobot_setJointSpeedRatio(iMobot_t* iMobot, robotJointId_t id, double ratio);
int iMobot_setTwoWheelRobotSpeed(iMobot_t* iMobot, double speed, double radius, char unit[]);
int iMobot_stop(iMobot_t* iMobot);
int iMobot_terminate(iMobot_t* iMobot);
int iMobot_moveJointWait(iMobot_t* iMobot, robotJointId_t id);

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
    int getJointAngle(robotJointId_t id, double &angle);
    int getJointMaxSpeed(robotJointId_t id, double &maxSpeed);
    int getJointSpeed(robotJointId_t id, double &speed);
    int getJointState(robotJointId_t id, robotJointState_t &state);
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
    int moveJoint(robotJointId_t id, double angle);
    int moveJointNB(robotJointId_t id, double angle);
    int moveJointTo(robotJointId_t id, double angle);
    int moveJointToNB(robotJointId_t id, double angle);
    int moveWait();
    int moveToZero();
    int setJointSpeed(robotJointId_t id, double speed);
    int setJointSpeedRatio(robotJointId_t id, double ratio);
    int setTwoWheelRobotSpeed(double speed, double radius, char unit[]);
    int stop();
    int terminate();
    int moveJointWait(robotJointId_t id);
#ifndef _CH_
  private:
    int getJointDirection(robotJointId_t id, robotJointState_t &direction);
    int setJointDirection(robotJointId_t id, robotJointState_t direction);
    iMobot_t _iMobot;
#else
  private:
    int getJointDirection(robotJointId_t id, robotJointState_t &direction);
    int setJointDirection(robotJointId_t id, robotJointState_t direction);
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
