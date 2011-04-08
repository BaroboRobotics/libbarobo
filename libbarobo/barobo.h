#ifndef _BAROBO_H_
#define _BAROBO_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct iMobot_s {
} iMobot_t;

int BR_pose(iMobot_t* iMobot, float angle[4], const char motorMask);
int BR_move(iMobot_t* iMobot, float angle[4], const char motorMask);
int BR_stop(iMobot_t* iMobot);
int BR_moveWait(iMobot_t* iMobot);
int BR_isBusy(iMobot_t* iMobot);
int BR_getJointAngles(iMobot_t* iMobot, float angle[4]);

#ifdef __cplusplus
}
#endif

#endif
