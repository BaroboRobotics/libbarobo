#ifndef _BAROBO_H_
#define _BAROBO_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct iMobot_s {
} iMobot_t;

int BR_move(iMobot_t* iMobot, double angle[4], const char motorMask);
int BR_moveRelative(iMobot_t* iMobot, double angle[4], const char motorMask);
int BR_moveJoint(iMobot_t* iMobot, double angle, int motorNumber);
int BR_moveJointRelative(iMobot_t* iMobot, double angle, int motorNumber);
int BR_stop(iMobot_t* iMobot);
int BR_moveWait(iMobot_t* iMobot);
int BR_isBusy(iMobot_t* iMobot);
int BR_getJointAngles(iMobot_t* iMobot, double angle[4]);
int BR_setSpeed(iMobot_t* iMobot, double speed[4], const char motorMask);
int BR_getSpeed(iMobot_t* iMobot, double speed[4]);



#ifdef __cplusplus
}
#endif

#endif
