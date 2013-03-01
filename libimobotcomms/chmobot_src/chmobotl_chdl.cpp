#include "../mobot.h"
#include <ch.h>

EXPORTCH void CMobotL_CMobotL_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class CMobotL *c=new CMobotL();
  Ch_VaStart(interp, ap, varg);
  Ch_CppChangeThisPointer(interp, c, sizeof(CMobotL));
  Ch_VaEnd(interp, ap);
}

EXPORTCH void CMobotL_dCMobotL_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class CMobotL *c;
  Ch_VaStart(interp, ap, varg);
  c = Ch_VaArg(interp, ap, class CMobotL *);
  if(Ch_CppIsArrayElement(interp))
    c->~CMobotL();
  else
    delete c;
  Ch_VaEnd(interp, ap);
  return;
}

EXPORTCH int MobotL_blinkLED_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    double delay;
    int numBlinks;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    delay = Ch_VaArg(interp, ap, double);
    numBlinks = Ch_VaArg(interp, ap, int);
    retval = mobot->blinkLED(delay, numBlinks);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_connect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    retval = mobot->connect();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_connectWithAddress_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    char *address;
    int channel;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    address = Ch_VaArg(interp, ap, char *);
    if(Ch_VaCount(interp, ap) == 1) {
      channel = Ch_VaArg(interp, ap, int);
      retval = mobot->connectWithAddress(address, channel);
    } else {
      retval = mobot->connectWithAddress(address);
    }
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_connectWithIPAddress_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    char *address;
    char *port;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    address = Ch_VaArg(interp, ap, char *);
    if(Ch_VaCount(interp, ap) == 1) {
      port = Ch_VaArg(interp, ap, char *);
      retval = mobot->connectWithIPAddress(address, port);
    } else {
      retval = mobot->connectWithIPAddress(address);
    }
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_connectWithSerialID_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    char *address;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    address = Ch_VaArg(interp, ap, char *);
    retval = mobot->connectWithSerialID(address);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_disconnect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    retval = mobot->disconnect();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_disableButtonCallback_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    retval = mobot->disableButtonCallback();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_enableButtonCallback_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    void (*cb)(CMobotL*,int,int);
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    cb = (void(*)(CMobotL*,int,int))Ch_VaArg(interp, ap, void*);
    retval = mobot->enableButtonCallback(cb);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_isConnected_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    retval = mobot->isConnected();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_isMoving_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    retval = mobot->isMoving();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_getAccelerometerData_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    double *x, *y, *z;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    x = Ch_VaArg(interp, ap, double *);
    y = Ch_VaArg(interp, ap, double *);
    z = Ch_VaArg(interp, ap, double *);
    retval = mobot->getAccelerometerData(*x, *y, *z);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_getBatteryVoltage_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    double* voltage;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    voltage = Ch_VaArg(interp, ap, double *);
    retval = mobot->getBatteryVoltage(*voltage);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_getColorRGB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    int *r, *g, *b;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    r = Ch_VaArg(interp, ap, int *);
    g = Ch_VaArg(interp, ap, int *);
    b = Ch_VaArg(interp, ap, int *);
    retval = mobot->getColorRGB(*r, *g, *b);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_getFormFactor_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    int* formFactor;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    formFactor = Ch_VaArg(interp, ap, int *);
    retval = mobot->getFormFactor(*formFactor);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_getJointAngle_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    int id;
    double* angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    id = Ch_VaArg(interp, ap, int);
    angle = Ch_VaArg(interp, ap, double *);
    retval = mobot->getJointAngle((mobotJointId_t)id, *angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_getJointAngleAverage_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    int id;
    double* angle;
    int numReadings;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    id = Ch_VaArg(interp, ap, int);
    angle = Ch_VaArg(interp, ap, double *);
    if(Ch_VaCount(interp, ap) == 1) {
      numReadings = Ch_VaArg(interp, ap, int);
      retval = mobot->getJointAngleAverage((mobotJointId_t)id, *angle, numReadings);
    } else {
      retval = mobot->getJointAngleAverage((mobotJointId_t)id, *angle);
    }
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_getJointAngles_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    double* angle1;
    double* angle2;
    double* angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    angle1 = Ch_VaArg(interp, ap, double *);
    angle2 = Ch_VaArg(interp, ap, double *);
    angle3 = Ch_VaArg(interp, ap, double *);
    retval = mobot->getJointAngles(*angle1, *angle2, *angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_getJointAnglesAverage_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    double* angle1;
    double* angle2;
    double* angle3;
    int numReadings;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    angle1 = Ch_VaArg(interp, ap, double *);
    angle2 = Ch_VaArg(interp, ap, double *);
    angle3 = Ch_VaArg(interp, ap, double *);
    if(Ch_VaCount(interp ,ap) == 1) {
      numReadings = Ch_VaArg(interp, ap, int);
      retval = mobot->getJointAnglesAverage(*angle1, *angle2, *angle3, numReadings);
    } else {
      retval = mobot->getJointAnglesAverage(*angle1, *angle2, *angle3);
    }
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_getJointMaxSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    int id;
    double *speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    id = Ch_VaArg(interp, ap, int);
    speed = Ch_VaArg(interp, ap, double *);
    retval = mobot->getJointMaxSpeed((mobotJointId_t)id, *speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_getJointSafetyAngle_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    double* angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    angle = Ch_VaArg(interp, ap, double *);
    retval = mobot->getJointSafetyAngle(*angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_getJointSafetyAngleTimeout_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    double* seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    seconds = Ch_VaArg(interp, ap, double *);
    retval = mobot->getJointSafetyAngleTimeout(*seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_getJointSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    int id;
    double *speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    id = Ch_VaArg(interp, ap, int);
    speed = Ch_VaArg(interp, ap, double *);
    retval = mobot->getJointSpeed((mobotJointId_t)id, *speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_getJointSpeeds_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    double *speed1;
    double *speed2;
    double *speed3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    speed1 = Ch_VaArg(interp, ap, double *);
    speed2 = Ch_VaArg(interp, ap, double *);
    speed3 = Ch_VaArg(interp, ap, double *);
    retval = mobot->getJointSpeeds(*speed1, *speed2, *speed3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_getJointSpeedRatio_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    int id;
    double *speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    id = Ch_VaArg(interp, ap, int);
    speed = Ch_VaArg(interp, ap, double *);
    retval = mobot->getJointSpeedRatio((mobotJointId_t)id, *speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_getJointSpeedRatios_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    double *ratio1;
    double *ratio2;
    double *ratio3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    ratio1 = Ch_VaArg(interp, ap, double *);
    ratio2 = Ch_VaArg(interp, ap, double *);
    ratio3 = Ch_VaArg(interp, ap, double *);
    retval = mobot->getJointSpeedRatios(*ratio1, *ratio2, *ratio3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_getJointState_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    int id;
    int * state;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    id = Ch_VaArg(interp, ap, int);
    state = Ch_VaArg(interp, ap, int *);
    retval = mobot->getJointState((mobotJointId_t)id, (mobotJointState_t&)(*state));
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_move_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = mobot->move(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_moveNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = mobot->moveNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_moveContinuousNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    mobotJointState_t dir1;
    mobotJointState_t dir2;
    mobotJointState_t dir3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    dir1 = (mobotJointState_t)Ch_VaArg(interp, ap, int);
    dir2 = (mobotJointState_t)Ch_VaArg(interp, ap, int);
    dir3 = (mobotJointState_t)Ch_VaArg(interp, ap, int);
    retval = mobot->moveContinuousNB(dir1, dir2, dir3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_moveContinuousTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    mobotJointState_t dir1;
    mobotJointState_t dir2;
    mobotJointState_t dir3;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    dir1 = Ch_VaArg(interp, ap, mobotJointState_t);
    dir2 = Ch_VaArg(interp, ap, mobotJointState_t);
    dir3 = Ch_VaArg(interp, ap, mobotJointState_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = mobot->moveContinuousTime(dir1, dir2, dir3, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_moveJointContinuousNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    mobotJointId_t id;
    mobotJointState_t dir;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    id = Ch_VaArg(interp, ap, mobotJointId_t );
    dir = Ch_VaArg(interp, ap, mobotJointState_t);
    retval = mobot->moveJointContinuousNB(id, dir);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_moveJointContinuousTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    mobotJointId_t id;
    mobotJointState_t dir;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    id = Ch_VaArg(interp, ap, mobotJointId_t);
    dir = Ch_VaArg(interp, ap, mobotJointState_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointContinuousTime(id, dir, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_moveJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    mobotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    id = Ch_VaArg(interp, ap, mobotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJoint(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_moveJointNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    mobotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    id = Ch_VaArg(interp, ap, mobotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_moveJointTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    mobotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    id = Ch_VaArg(interp, ap, mobotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointTo(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_moveJointToDirect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    mobotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    id = Ch_VaArg(interp, ap, mobotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointToDirect(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_moveJointToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    mobotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    id = Ch_VaArg(interp, ap, mobotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointToNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_moveJointToDirectNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    mobotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    id = Ch_VaArg(interp, ap, mobotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointToDirectNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_moveJointWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    mobotJointId_t id;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    id = Ch_VaArg(interp, ap, mobotJointId_t);
    retval = mobot->moveJointWait(id);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_moveTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = mobot->moveTo(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_moveToDirect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = mobot->moveToDirect(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_driveToDirect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = mobot->driveToDirect(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int MobotL_driveTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = mobot->driveTo(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_moveToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = mobot->moveToNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_moveToDirectNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = mobot->moveToDirectNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_driveToDirectNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = mobot->driveToDirectNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int MobotL_driveToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = mobot->driveToNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_moveWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    retval = mobot->moveWait();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_moveToZero_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    retval = mobot->moveToZero();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_moveToZeroNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    retval = mobot->moveToZeroNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_recordAngle_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    mobotJointId_t id;
    double* time;
    double* angle;
    int num;
    double seconds;
    int shiftData;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    id = Ch_VaArg(interp, ap, mobotJointId_t);
    time = Ch_VaArg(interp, ap, double*);
    angle = Ch_VaArg(interp, ap, double*);
    num = Ch_VaArg(interp, ap, int);
    seconds = Ch_VaArg(interp, ap, double);
    if(Ch_VaCount(interp, ap) == 1) {
      shiftData = Ch_VaArg(interp, ap, int);
      retval = mobot->recordAngle(id, time, angle, num, seconds, shiftData);
    } else {
      retval = mobot->recordAngle(id, time, angle, num, seconds);
    }
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_recordAngleBegin_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    mobotJointId_t id;
    double** time;
    double** angle;
    double seconds;
    int shiftData;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    id = Ch_VaArg(interp, ap, mobotJointId_t);
    time = Ch_VaArg(interp, ap, double**);
    angle = Ch_VaArg(interp, ap, double**);
    seconds = Ch_VaArg(interp, ap, double);
    if(Ch_VaCount(interp, ap) == 1) {
      shiftData = Ch_VaArg(interp, ap, int);
      retval = mobot->recordAngleBegin(id, *time, *angle, seconds, shiftData);
    } else {
      retval = mobot->recordAngleBegin(id, *time, *angle, seconds);
    }
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_recordAngleEnd_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    mobotJointId_t id;
    int *num;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    id = Ch_VaArg(interp, ap, mobotJointId_t);
    num = Ch_VaArg(interp, ap, int* );
    retval = mobot->recordAngleEnd(id, *num);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_recordAngles_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    double* time;
    double* angle1;
    double* angle2;
    double* angle3;
    int num;
    double seconds;
    int shiftData;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    time = Ch_VaArg(interp, ap, double*);
    angle1 = Ch_VaArg(interp, ap, double*);
    angle2 = Ch_VaArg(interp, ap, double*);
    angle3 = Ch_VaArg(interp, ap, double*);
    num = Ch_VaArg(interp, ap, int);
    seconds = Ch_VaArg(interp, ap, double);
    if(Ch_VaCount(interp, ap) == 1) {
      shiftData = Ch_VaArg(interp, ap, int);
      retval = mobot->recordAngles(time, angle1, angle2, angle3, num, seconds, shiftData);
    } else {
      retval = mobot->recordAngles(time, angle1, angle2, angle3, num, seconds);
    }
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_recordAnglesBegin_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    double** time;
    double** angle1;
    double** angle2;
    double** angle3;
    double seconds;
    int shiftData;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    time = Ch_VaArg(interp, ap, double**);
    angle1 = Ch_VaArg(interp, ap, double**);
    angle2 = Ch_VaArg(interp, ap, double**);
    angle3 = Ch_VaArg(interp, ap, double**);
    seconds = Ch_VaArg(interp, ap, double);
    if(Ch_VaCount(interp, ap) == 1) {
      shiftData = Ch_VaArg(interp, ap, int);
      retval = mobot->recordAnglesBegin(
          *time, 
          *angle1, 
          *angle2, 
          *angle3, 
          seconds,
          shiftData);
    } else {
      retval = mobot->recordAnglesBegin(
          *time, 
          *angle1, 
          *angle2, 
          *angle3, 
          seconds);
    }
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_recordAnglesEnd_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    int retval;
    int *num;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    num = Ch_VaArg(interp, ap, int*);
    retval = mobot->recordAnglesEnd(*num);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_recordDistanceBegin_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    mobotJointId_t id;
    double** time;
    double** angle;
    double radius;
    double seconds;
    int shiftData;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    id = Ch_VaArg(interp, ap, mobotJointId_t);
    time = Ch_VaArg(interp, ap, double**);
    angle = Ch_VaArg(interp, ap, double**);
    radius = Ch_VaArg(interp, ap, double);
    seconds = Ch_VaArg(interp, ap, double);
    if(Ch_VaCount(interp, ap) == 1) {
      shiftData = Ch_VaArg(interp, ap, int);
      retval = mobot->recordDistanceBegin(id, *time, *angle, radius, seconds, shiftData);
    } else {
      retval = mobot->recordDistanceBegin(id, *time, *angle, radius, seconds);
    }
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_recordDistanceEnd_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    mobotJointId_t id;
    int *num;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    id = Ch_VaArg(interp, ap, mobotJointId_t);
    num = Ch_VaArg(interp, ap, int* );
    retval = mobot->recordDistanceEnd(id, *num);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_recordDistancesBegin_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    double** time;
    double** angle1;
    double** angle2;
    double** angle3;
    double radius;
    double seconds;
    int shiftData;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    time = Ch_VaArg(interp, ap, double**);
    angle1 = Ch_VaArg(interp, ap, double**);
    angle2 = Ch_VaArg(interp, ap, double**);
    angle3 = Ch_VaArg(interp, ap, double**);
    radius = Ch_VaArg(interp, ap, double);
    seconds = Ch_VaArg(interp, ap, double);
    if(Ch_VaCount(interp, ap) == 1) {
      shiftData = Ch_VaArg(interp, ap, int);
      retval = mobot->recordDistancesBegin(
          *time, 
          *angle1, 
          *angle2, 
          *angle3, 
          radius,
          seconds,
          shiftData);
    } else {
      retval = mobot->recordDistancesBegin(
          *time, 
          *angle1, 
          *angle2, 
          *angle3, 
          radius,
          seconds);
    }
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_recordDistancesEnd_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    int retval;
    int *num;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    num = Ch_VaArg(interp, ap, int*);
    retval = mobot->recordDistancesEnd(*num);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_recordWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    retval = mobot->recordWait();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_reset_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    retval = mobot->reset();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_resetToZero_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    retval = mobot->resetToZero();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_resetToZeroNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    retval = mobot->resetToZeroNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_setBuzzerFrequency_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    int frequency;
    double time;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    frequency = Ch_VaArg(interp, ap, int);
    time = Ch_VaArg(interp, ap, double);
    retval = mobot->setBuzzerFrequency(frequency, time);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_setBuzzerFrequencyOn_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    int frequency;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    frequency = Ch_VaArg(interp, ap, int);
    retval = mobot->setBuzzerFrequencyOn(frequency);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_setBuzzerFrequencyOff_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    retval = mobot->setBuzzerFrequencyOff();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_setColorRGB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    int r, g, b;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    r = Ch_VaArg(interp, ap, int);
    g = Ch_VaArg(interp, ap, int);
    b = Ch_VaArg(interp, ap, int);
    retval = mobot->setColorRGB(r, g, b);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_setExitState_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    mobotJointState_t dir;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    dir = Ch_VaArg(interp, ap, mobotJointState_t);
    retval = mobot->setExitState(dir);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_setJointMovementStateNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    mobotJointId_t id;
    mobotJointState_t dir;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    id = Ch_VaArg(interp, ap, mobotJointId_t );
    dir = Ch_VaArg(interp, ap, mobotJointState_t);
    retval = mobot->setJointMovementStateNB(id, dir);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_setJointMovementStateTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    mobotJointId_t id;
    mobotJointState_t dir;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    id = Ch_VaArg(interp, ap, mobotJointId_t);
    dir = Ch_VaArg(interp, ap, mobotJointState_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = mobot->setJointMovementStateTime(id, dir, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_setJointSafetyAngle_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->setJointSafetyAngle(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_setJointSafetyAngleTimeout_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    seconds = Ch_VaArg(interp, ap, double);
    retval = mobot->setJointSafetyAngleTimeout(seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_setJointSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    mobotJointId_t id;
    double speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    id = Ch_VaArg(interp, ap, mobotJointId_t);
    speed = Ch_VaArg(interp, ap, double);
    retval = mobot->setJointSpeed(id, speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_setJointSpeeds_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    mobotJointId_t id;
    double speed1;
    double speed2;
    double speed3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    speed1 = Ch_VaArg(interp, ap, double);
    speed2 = Ch_VaArg(interp, ap, double);
    speed3 = Ch_VaArg(interp, ap, double);
    retval = mobot->setJointSpeeds(speed1, speed2, speed3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_setJointSpeedRatio_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    mobotJointId_t id;
    double speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    id = Ch_VaArg(interp, ap, mobotJointId_t);
    speed = Ch_VaArg(interp, ap, double);
    retval = mobot->setJointSpeedRatio(id, speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_setJointSpeedRatios_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    double ratio1;
    double ratio2;
    double ratio3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    ratio1 = Ch_VaArg(interp, ap, double );
    ratio2 = Ch_VaArg(interp, ap, double );
    ratio3 = Ch_VaArg(interp, ap, double );
    retval = mobot->setJointSpeedRatios(ratio1, ratio2, ratio3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_setMotorPower_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    mobotJointId_t id;
    int power;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    id = Ch_VaArg(interp, ap, mobotJointId_t);
    power = Ch_VaArg(interp, ap, int);
    retval = mobot->setMotorPower(id, power);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_setMovementStateNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    mobotJointState_t dir1;
    mobotJointState_t dir2;
    mobotJointState_t dir3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    dir1 = (mobotJointState_t)Ch_VaArg(interp, ap, int);
    dir2 = (mobotJointState_t)Ch_VaArg(interp, ap, int);
    dir3 = (mobotJointState_t)Ch_VaArg(interp, ap, int);
    retval = mobot->setMovementStateNB(dir1, dir2, dir3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_setMovementStateTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    mobotJointState_t dir1;
    mobotJointState_t dir2;
    mobotJointState_t dir3;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    dir1 = Ch_VaArg(interp, ap, mobotJointState_t);
    dir2 = Ch_VaArg(interp, ap, mobotJointState_t);
    dir3 = Ch_VaArg(interp, ap, mobotJointState_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = mobot->setMovementStateTime(dir1, dir2, dir3, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_setMovementStateTimeNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    mobotJointState_t dir1;
    mobotJointState_t dir2;
    mobotJointState_t dir3;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    dir1 = Ch_VaArg(interp, ap, mobotJointState_t);
    dir2 = Ch_VaArg(interp, ap, mobotJointState_t);
    dir3 = Ch_VaArg(interp, ap, mobotJointState_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = mobot->setMovementStateTimeNB(dir1, dir2, dir3, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_setTwoWheelRobotSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    double speed;
    double radius;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    speed = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    retval = mobot->setTwoWheelRobotSpeed(speed, radius);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_stopOneJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    mobotJointId_t id;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    id = Ch_VaArg(interp, ap, mobotJointId_t);
    retval = mobot->stopOneJoint(id);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_stopAllJoints_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    retval = mobot->stopAllJoints();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int MobotL_stop_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotL *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotL *);
    retval = mobot->stop();
    Ch_VaEnd(interp, ap);
    return retval;
}

