#include "../imobot.h"
#include <ch.h>

EXPORTCH void CiMobot_CiMobot_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class CiMobot *c=new CiMobot();
  Ch_VaStart(interp, ap, varg);
  Ch_CppChangeThisPointer(interp, c, sizeof(CiMobot));
  Ch_VaEnd(interp, ap);
}

EXPORTCH void CiMobot_dCiMobot_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class CiMobot *c;
  Ch_VaStart(interp, ap, varg);
  c = Ch_VaArg(interp, ap, class CiMobot *);
  if(Ch_CppIsArrayElement(interp))
    c->~CiMobot();
  else
    delete c;
  Ch_VaEnd(interp, ap);
  return;
}

EXPORTCH int getJointAngle_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    CiMobot *imobot;
    iMobotJointId_t id;
    double *angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    imobot = Ch_VaArg(interp, ap, CiMobot*);
    id = Ch_VaArg(interp, ap, iMobotJointId_t);
    angle = Ch_VaArg(interp, ap, double *);
    retval = imobot->getJointAngle(id, *angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int getJointSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    CiMobot *imobot;
    iMobotJointId_t id;
    double *speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    imobot = Ch_VaArg(interp, ap, CiMobot*);
    id = Ch_VaArg(interp, ap, iMobotJointId_t);
    speed = Ch_VaArg(interp, ap, double *);
    retval = imobot->getJointSpeed(id, *speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int getJointState_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    CiMobot *imobot;
    iMobotJointId_t id;
    iMobotJointState_t *state;
    int retval;

    Ch_VaStart(interp, ap, varg);
    imobot = Ch_VaArg(interp, ap, CiMobot*);
    id = Ch_VaArg(interp, ap, iMobotJointId_t);
    state = Ch_VaArg(interp, ap, iMobotJointState_t *);
    retval = imobot->getJointState(id, *state);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int initListenerBluetooth_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    CiMobot *imobot;
    int channel;
    int retval;

    Ch_VaStart(interp, ap, varg);
    imobot = Ch_VaArg(interp, ap, CiMobot*);
    channel = Ch_VaArg(interp, ap, int);
    retval = imobot->initListenerBluetooth(channel);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int isBusy_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    CiMobot *imobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    imobot = Ch_VaArg(interp, ap, CiMobot*);
    retval = imobot->isBusy();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int listenerMainLoop_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    CiMobot *imobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    imobot = Ch_VaArg(interp, ap, CiMobot*);
    retval = imobot->listenerMainLoop();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int move_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    CiMobot *imobot;
    double angle1;
    double angle2;
    double angle3;
    double angle4;
    int retval;

    Ch_VaStart(interp, ap, varg);
    imobot = Ch_VaArg(interp, ap, CiMobot*);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    angle4 = Ch_VaArg(interp, ap, double);
    retval = imobot->move(angle1, angle2, angle3, angle4);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int moveNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    CiMobot *imobot;
    double angle1;
    double angle2;
    double angle3;
    double angle4;
    int retval;

    Ch_VaStart(interp, ap, varg);
    imobot = Ch_VaArg(interp, ap, CiMobot*);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    angle4 = Ch_VaArg(interp, ap, double);
    retval = imobot->moveNB(angle1, angle2, angle3, angle4);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int moveTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    CiMobot *imobot;
    double angle1;
    double angle2;
    double angle3;
    double angle4;
    int retval;

    Ch_VaStart(interp, ap, varg);
    imobot = Ch_VaArg(interp, ap, CiMobot*);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    angle4 = Ch_VaArg(interp, ap, double);
    retval = imobot->moveTo(angle1, angle2, angle3, angle4);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int moveToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    CiMobot *imobot;
    double angle1;
    double angle2;
    double angle3;
    double angle4;
    int retval;

    Ch_VaStart(interp, ap, varg);
    imobot = Ch_VaArg(interp, ap, CiMobot*);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    angle4 = Ch_VaArg(interp, ap, double);
    retval = imobot->moveToNB(angle1, angle2, angle3, angle4);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int moveJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    CiMobot *imobot;
    iMobotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    imobot = Ch_VaArg(interp, ap, CiMobot*);
    id = Ch_VaArg(interp, ap, iMobotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = imobot->moveJoint(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int moveJointNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    CiMobot *imobot;
    iMobotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    imobot = Ch_VaArg(interp, ap, CiMobot*);
    id = Ch_VaArg(interp, ap, iMobotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = imobot->moveJointNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int moveJointTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    CiMobot *imobot;
    iMobotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    imobot = Ch_VaArg(interp, ap, CiMobot*);
    id = Ch_VaArg(interp, ap, iMobotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = imobot->moveJointTo(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int moveJointToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    CiMobot *imobot;
    iMobotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    imobot = Ch_VaArg(interp, ap, CiMobot*);
    id = Ch_VaArg(interp, ap, iMobotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = imobot->moveJointToNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int moveWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    CiMobot *imobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    imobot = Ch_VaArg(interp, ap, CiMobot*);
    retval = imobot->moveWait();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int moveToZero_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    CiMobot *imobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    imobot = Ch_VaArg(interp, ap, CiMobot*);
    retval = imobot->moveToZero();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int setJointSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    CiMobot *imobot;
    iMobotJointId_t id;
    double speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    imobot = Ch_VaArg(interp, ap, CiMobot*);
    id = Ch_VaArg(interp, ap, iMobotJointId_t);
    speed = Ch_VaArg(interp, ap, double);
    retval = imobot->setJointSpeed(id, speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int stop_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    CiMobot *imobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    imobot = Ch_VaArg(interp, ap, CiMobot*);
    retval = imobot->stop();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int terminate_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    CiMobot *imobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    imobot = Ch_VaArg(interp, ap, CiMobot*);
    retval = imobot->terminate();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int moveJointWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    CiMobot *imobot;
    iMobotJointId_t id;
    int retval;

    Ch_VaStart(interp, ap, varg);
    imobot = Ch_VaArg(interp, ap, CiMobot*);
    id = Ch_VaArg(interp, ap, iMobotJointId_t);
    retval = imobot->moveJointWait(id);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int getJointDirection_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    CiMobot *imobot;
    iMobotJointId_t id;
    iMobotJointDirection_t *direction;
    int retval;

    Ch_VaStart(interp, ap, varg);
    imobot = Ch_VaArg(interp, ap, CiMobot*);
    id = Ch_VaArg(interp, ap, iMobotJointId_t);
    direction = Ch_VaArg(interp, ap, iMobotJointDirection_t *);
    retval = imobot->getJointDirection(id, *direction);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int setJointDirection_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    CiMobot *imobot;
    iMobotJointId_t id;
    iMobotJointDirection_t direction;
    int retval;

    Ch_VaStart(interp, ap, varg);
    imobot = Ch_VaArg(interp, ap, CiMobot*);
    id = Ch_VaArg(interp, ap, iMobotJointId_t);
    direction = Ch_VaArg(interp, ap, iMobotJointDirection_t);
    retval = imobot->setJointDirection(id, direction);
    Ch_VaEnd(interp, ap);
    return retval;
}
