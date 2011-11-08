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
    retval = mobot->getJointAngle(id, *angle);
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
    retval = mobot->getJointSpeed(id, *speed);
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
    retval = mobot->getJointState(id, *state);
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

EXPORTCH int moveContinuous_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int dir1;
    int dir2;
    int dir3;
    int dir4;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    dir1 = Ch_VaArg(interp, ap, int);
    dir2 = Ch_VaArg(interp, ap, int);
    dir3 = Ch_VaArg(interp, ap, int);
    dir4 = Ch_VaArg(interp, ap, int);
    retval = mobot->moveContinuous(dir1, dir2, dir3, dir4);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int moveContinuousTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int dir1;
    int dir2;
    int dir3;
    int dir4;
    int msecs;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    dir1 = Ch_VaArg(interp, ap, int);
    dir2 = Ch_VaArg(interp, ap, int);
    dir3 = Ch_VaArg(interp, ap, int);
    dir4 = Ch_VaArg(interp, ap, int);
    msecs = Ch_VaArg(interp, ap, int);
    retval = mobot->moveContinuousTime(dir1, dir2, dir3, dir4, msecs);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int moveJointTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, int);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointTo(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int moveJointWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int id;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, int);
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

EXPORTCH int moveWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;

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

    mobot = Ch_VaArg(interp, ap, class CMobot *);
    retval = mobot->moveToZero();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int setJointSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int id;
    double speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, int);
    speed = Ch_VaArg(interp, ap, double);
    retval = mobot->setJointSpeed(id, speed);
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

