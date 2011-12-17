#include "../mobot.h"
#include <ch.h>

EXPORTCH void CMobot_CMobot_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class CMobot *c=new CMobot();
  Ch_VaStart(interp, ap, varg);
  Ch_CppChangeThisPointer(interp, c, sizeof(CMobot));
  Ch_VaEnd(interp, ap);
}

EXPORTCH void CMobot_dCMobot_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class CMobot *c;
  Ch_VaStart(interp, ap, varg);
  c = Ch_VaArg(interp, ap, class CMobot *);
  if(Ch_CppIsArrayElement(interp))
    c->~CMobot();
  else
    delete c;
  Ch_VaEnd(interp, ap);
  return;
}

EXPORTCH int connect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    retval = mobot->connect();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int connectWithAddress_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    char *address;
    int channel;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    address = Ch_VaArg(interp, ap, char *);
    channel = Ch_VaArg(interp, ap, int);
    retval = mobot->connectWithAddress(address, channel);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int disconnect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    retval = mobot->disconnect();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int isConnected_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    retval = mobot->isConnected();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int isMoving_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    retval = mobot->isMoving();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int getJointAngle_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int id;
    double* angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, int);
    angle = Ch_VaArg(interp, ap, double *);
    retval = mobot->getJointAngle((mobotJointId_t)id, *angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int getJointMaxSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int id;
    double *speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, int);
    speed = Ch_VaArg(interp, ap, double *);
    retval = mobot->getJointMaxSpeed((mobotJointId_t)id, *speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int getJointSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int id;
    double *speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, int);
    speed = Ch_VaArg(interp, ap, double *);
    retval = mobot->getJointSpeed((mobotJointId_t)id, *speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int getJointState_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int id;
    int * state;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, int);
    state = Ch_VaArg(interp, ap, int *);
    retval = mobot->getJointState((mobotJointId_t)id, (mobotJointState_t&)(*state));
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int move_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double angle1;
    double angle2;
    double angle3;
    double angle4;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    angle4 = Ch_VaArg(interp, ap, double);
    retval = mobot->move(angle1, angle2, angle3, angle4);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int moveNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double angle1;
    double angle2;
    double angle3;
    double angle4;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    angle4 = Ch_VaArg(interp, ap, double);
    retval = mobot->moveNB(angle1, angle2, angle3, angle4);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int moveContinuousNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    mobotJointDirection_t dir1;
    mobotJointDirection_t dir2;
    mobotJointDirection_t dir3;
    mobotJointDirection_t dir4;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    dir1 = (mobotJointDirection_t)Ch_VaArg(interp, ap, int);
    dir2 = (mobotJointDirection_t)Ch_VaArg(interp, ap, int);
    dir3 = (mobotJointDirection_t)Ch_VaArg(interp, ap, int);
    dir4 = (mobotJointDirection_t)Ch_VaArg(interp, ap, int);
    retval = mobot->moveContinuousNB(dir1, dir2, dir3, dir4);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int moveContinuousTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    mobotJointDirection_t dir1;
    mobotJointDirection_t dir2;
    mobotJointDirection_t dir3;
    mobotJointDirection_t dir4;
    int msecs;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    dir1 = Ch_VaArg(interp, ap, mobotJointDirection_t);
    dir2 = Ch_VaArg(interp, ap, mobotJointDirection_t);
    dir3 = Ch_VaArg(interp, ap, mobotJointDirection_t);
    dir4 = Ch_VaArg(interp, ap, mobotJointDirection_t);
    msecs = Ch_VaArg(interp, ap, int);
    retval = mobot->moveContinuousTime(dir1, dir2, dir3, dir4, msecs);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int moveJointContinuousNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    mobotJointId_t id;
    mobotJointDirection_t dir;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, mobotJointId_t );
    dir = Ch_VaArg(interp, ap, mobotJointDirection_t);
    retval = mobot->moveJointContinuousNB(id, dir);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int moveJointContinuousTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    mobotJointId_t id;
    mobotJointDirection_t dir;
    int msecs;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, mobotJointId_t);
    dir = Ch_VaArg(interp, ap, mobotJointDirection_t);
    msecs = Ch_VaArg(interp, ap, int);
    retval = mobot->moveJointContinuousTime(id, dir, msecs);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int moveJointTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    mobotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, mobotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointTo(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int moveJointToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    mobotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, mobotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointToNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int moveJointWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    mobotJointId_t id;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, mobotJointId_t);
    retval = mobot->moveJointWait(id);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int moveTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double angle1;
    double angle2;
    double angle3;
    double angle4;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    angle4 = Ch_VaArg(interp, ap, double);
    retval = mobot->moveTo(angle1, angle2, angle3, angle4);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int moveToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double angle1;
    double angle2;
    double angle3;
    double angle4;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    angle4 = Ch_VaArg(interp, ap, double);
    retval = mobot->moveToNB(angle1, angle2, angle3, angle4);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int moveWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    retval = mobot->moveWait();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int moveToZero_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    retval = mobot->moveToZero();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int moveToZeroNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    retval = mobot->moveToZeroNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int setJointSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    mobotJointId_t id;
    double speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, mobotJointId_t);
    speed = Ch_VaArg(interp, ap, double);
    retval = mobot->setJointSpeed(id, speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int setJointSpeedRatio_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    mobotJointId_t id;
    double speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, mobotJointId_t);
    speed = Ch_VaArg(interp, ap, double);
    retval = mobot->setJointSpeedRatio(id, speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int stop_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    retval = mobot->stop();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int motionInchwormLeft_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    retval = mobot->motionInchwormLeft();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int motionInchwormRight_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    retval = mobot->motionInchwormRight();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int motionRollBackward_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    retval = mobot->motionRollBackward();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int motionRollForward_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    retval = mobot->motionRollForward();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int motionStand_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    retval = mobot->motionStand();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int motionTurnLeft_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    retval = mobot->motionTurnLeft();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int motionTurnRight_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    retval = mobot->motionTurnRight();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int motionInchwormLeftNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    retval = mobot->motionInchwormLeftNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int motionInchwormRightNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    retval = mobot->motionInchwormRightNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int motionRollBackwardNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    retval = mobot->motionRollBackwardNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int motionRollForwardNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;
    
    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    retval = mobot->motionRollForwardNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int motionStandNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    retval = mobot->motionStandNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int motionTurnLeftNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    retval = mobot->motionTurnLeftNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int motionTurnRightNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    retval = mobot->motionTurnRightNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

