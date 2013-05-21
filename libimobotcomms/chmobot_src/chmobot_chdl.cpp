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

EXPORTCH int blinkLED_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double delay;
    int numBlinks;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    delay = Ch_VaArg(interp, ap, double);
    numBlinks = Ch_VaArg(interp, ap, int);
    retval = mobot->blinkLED(delay, numBlinks);
    Ch_VaEnd(interp, ap);
    return retval;
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
    if(Ch_VaCount(interp, ap) == 1) {
      channel = Ch_VaArg(interp, ap, int);
      retval = mobot->connectWithAddress(address, channel);
    } else {
      retval = mobot->connectWithAddress(address);
    }
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int connectWithBluetoothAddress_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    char *address;
    int channel;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    address = Ch_VaArg(interp, ap, char *);
    if(Ch_VaCount(interp, ap) == 1) {
      channel = Ch_VaArg(interp, ap, int);
      retval = mobot->connectWithBluetoothAddress(address, channel);
    } else {
      retval = mobot->connectWithBluetoothAddress(address);
    }
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int connectWithIPAddress_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    char *address;
    char *port;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
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

EXPORTCH int disableButtonCallback_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    retval = mobot->disableButtonCallback();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int enableButtonCallback_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    void (*cb)(CMobot*,int,int);
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    cb = (void(*)(CMobot*,int,int))Ch_VaArg(interp, ap, void*);
    retval = mobot->enableButtonCallback(cb);
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
    retval = mobot->getJointAngle((robotJointId_t)id, *angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int getJointAngleAverage_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int id;
    double* angle;
    int numReadings;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, int);
    angle = Ch_VaArg(interp, ap, double *);
    if(Ch_VaCount(interp, ap) == 1) {
      numReadings = Ch_VaArg(interp, ap, int);
      retval = mobot->getJointAngleAverage((robotJointId_t)id, *angle, numReadings);
    } else {
      retval = mobot->getJointAngleAverage((robotJointId_t)id, *angle);
    }
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int getJointAngles_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double* angle1;
    double* angle2;
    double* angle3;
    double* angle4;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    angle1 = Ch_VaArg(interp, ap, double *);
    angle2 = Ch_VaArg(interp, ap, double *);
    angle3 = Ch_VaArg(interp, ap, double *);
    angle4 = Ch_VaArg(interp, ap, double *);
    retval = mobot->getJointAngles(*angle1, *angle2, *angle3, *angle4);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int getJointAnglesAverage_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double* angle1;
    double* angle2;
    double* angle3;
    double* angle4;
    int numReadings;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    angle1 = Ch_VaArg(interp, ap, double *);
    angle2 = Ch_VaArg(interp, ap, double *);
    angle3 = Ch_VaArg(interp, ap, double *);
    angle4 = Ch_VaArg(interp, ap, double *);
    if(Ch_VaCount(interp ,ap) == 1) {
      numReadings = Ch_VaArg(interp, ap, int);
      retval = mobot->getJointAnglesAverage(*angle1, *angle2, *angle3, *angle4, numReadings);
    } else {
      retval = mobot->getJointAnglesAverage(*angle1, *angle2, *angle3, *angle4);
    }
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
    retval = mobot->getJointMaxSpeed((robotJointId_t)id, *speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int getJointSafetyAngle_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double* angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    angle = Ch_VaArg(interp, ap, double *);
    retval = mobot->getJointSafetyAngle(*angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int getJointSafetyAngleTimeout_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double* seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    seconds = Ch_VaArg(interp, ap, double *);
    retval = mobot->getJointSafetyAngleTimeout(*seconds);
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
    retval = mobot->getJointSpeed((robotJointId_t)id, *speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int getJointSpeeds_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double *speed1;
    double *speed2;
    double *speed3;
    double *speed4;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    speed1 = Ch_VaArg(interp, ap, double *);
    speed2 = Ch_VaArg(interp, ap, double *);
    speed3 = Ch_VaArg(interp, ap, double *);
    speed4 = Ch_VaArg(interp, ap, double *);
    retval = mobot->getJointSpeeds(*speed1, *speed2, *speed3, *speed4);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int getJointSpeedRatio_chdl(void *varg) {
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
    retval = mobot->getJointSpeedRatio((robotJointId_t)id, *speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int getJointSpeedRatios_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double *ratio1;
    double *ratio2;
    double *ratio3;
    double *ratio4;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    ratio1 = Ch_VaArg(interp, ap, double *);
    ratio2 = Ch_VaArg(interp, ap, double *);
    ratio3 = Ch_VaArg(interp, ap, double *);
    ratio4 = Ch_VaArg(interp, ap, double *);
    retval = mobot->getJointSpeedRatios(*ratio1, *ratio2, *ratio3, *ratio4);
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
    retval = mobot->getJointState((robotJointId_t)id, (mobotJointState_t&)(*state));
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

EXPORTCH int moveBackward_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveBackward(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int moveBackwardNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveBackwardNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int moveContinuousNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    mobotJointState_t dir1;
    mobotJointState_t dir2;
    mobotJointState_t dir3;
    mobotJointState_t dir4;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    dir1 = (mobotJointState_t)Ch_VaArg(interp, ap, int);
    dir2 = (mobotJointState_t)Ch_VaArg(interp, ap, int);
    dir3 = (mobotJointState_t)Ch_VaArg(interp, ap, int);
    dir4 = (mobotJointState_t)Ch_VaArg(interp, ap, int);
    retval = mobot->moveContinuousNB(dir1, dir2, dir3, dir4);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int moveContinuousTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    mobotJointState_t dir1;
    mobotJointState_t dir2;
    mobotJointState_t dir3;
    mobotJointState_t dir4;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    dir1 = Ch_VaArg(interp, ap, mobotJointState_t);
    dir2 = Ch_VaArg(interp, ap, mobotJointState_t);
    dir3 = Ch_VaArg(interp, ap, mobotJointState_t);
    dir4 = Ch_VaArg(interp, ap, mobotJointState_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = mobot->moveContinuousTime(dir1, dir2, dir3, dir4, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int moveDistance_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double distance;
    double radius;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    distance = Ch_VaArg(interp, ap, double);
    radius= Ch_VaArg(interp, ap, double);
    retval = mobot->moveDistance(distance, radius);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int moveDistanceNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double distance;
    double radius;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    distance = Ch_VaArg(interp, ap, double);
    radius= Ch_VaArg(interp, ap, double);
    retval = mobot->moveDistanceNB(distance, radius);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int moveForward_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveForward(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int moveForwardNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveForwardNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int moveJointContinuousNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    robotJointId_t id;
    mobotJointState_t dir;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, robotJointId_t );
    dir = Ch_VaArg(interp, ap, mobotJointState_t);
    retval = mobot->moveJointContinuousNB(id, dir);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int moveJointContinuousTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    robotJointId_t id;
    mobotJointState_t dir;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    dir = Ch_VaArg(interp, ap, mobotJointState_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointContinuousTime(id, dir, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int moveJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJoint(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int moveJointNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int moveJointTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointTo(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int moveJointToDirect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointToDirect(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int moveJointToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointToNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int moveJointToDirectNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointToDirectNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int moveJointWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    robotJointId_t id;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
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

EXPORTCH int moveToDirect_chdl(void *varg) {
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
    retval = mobot->moveToDirect(angle1, angle2, angle3, angle4);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int driveToDirect_chdl(void *varg) {
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
    retval = mobot->driveToDirect(angle1, angle2, angle3, angle4);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int driveTo_chdl(void *varg) {
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
    retval = mobot->driveTo(angle1, angle2, angle3, angle4);
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

EXPORTCH int moveToDirectNB_chdl(void *varg) {
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
    retval = mobot->moveToDirectNB(angle1, angle2, angle3, angle4);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int driveToDirectNB_chdl(void *varg) {
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
    retval = mobot->driveToDirectNB(angle1, angle2, angle3, angle4);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int driveToNB_chdl(void *varg) {
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
    retval = mobot->driveToNB(angle1, angle2, angle3, angle4);
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

EXPORTCH int recordAngle_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    robotJointId_t id;
    double* time;
    double* angle;
    int num;
    double seconds;
    int shiftData;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
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

EXPORTCH int recordAngleBegin_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    robotJointId_t id;
    double** time;
    double** angle;
    double seconds;
    int shiftData;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
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

EXPORTCH int recordAngleEnd_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    robotJointId_t id;
    int *num;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    num = Ch_VaArg(interp, ap, int* );
    retval = mobot->recordAngleEnd(id, *num);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int recordAngles_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double* time;
    double* angle1;
    double* angle2;
    double* angle3;
    double* angle4;
    int num;
    double seconds;
    int shiftData;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    time = Ch_VaArg(interp, ap, double*);
    angle1 = Ch_VaArg(interp, ap, double*);
    angle2 = Ch_VaArg(interp, ap, double*);
    angle3 = Ch_VaArg(interp, ap, double*);
    angle4 = Ch_VaArg(interp, ap, double*);
    num = Ch_VaArg(interp, ap, int);
    seconds = Ch_VaArg(interp, ap, double);
    if(Ch_VaCount(interp, ap) == 1) {
      shiftData = Ch_VaArg(interp, ap, int);
      retval = mobot->recordAngles(time, angle1, angle2, angle3, angle4, num, seconds, shiftData);
    } else {
      retval = mobot->recordAngles(time, angle1, angle2, angle3, angle4, num, seconds);
    }
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int recordAnglesBegin_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double** time;
    double** angle1;
    double** angle2;
    double** angle3;
    double** angle4;
    double seconds;
    int shiftData;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    time = Ch_VaArg(interp, ap, double**);
    angle1 = Ch_VaArg(interp, ap, double**);
    angle2 = Ch_VaArg(interp, ap, double**);
    angle3 = Ch_VaArg(interp, ap, double**);
    angle4 = Ch_VaArg(interp, ap, double**);
    seconds = Ch_VaArg(interp, ap, double);
    if(Ch_VaCount(interp, ap) == 1) {
      shiftData = Ch_VaArg(interp, ap, int);
      retval = mobot->recordAnglesBegin(
          *time, 
          *angle1, 
          *angle2, 
          *angle3, 
          *angle4, 
          seconds,
          shiftData);
    } else {
      retval = mobot->recordAnglesBegin(
          *time, 
          *angle1, 
          *angle2, 
          *angle3, 
          *angle4, 
          seconds);
    }
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int recordAnglesEnd_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;
    int *num;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    num = Ch_VaArg(interp, ap, int*);
    retval = mobot->recordAnglesEnd(*num);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int recordDistanceBegin_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    robotJointId_t id;
    double** time;
    double** angle;
    double radius;
    double seconds;
    int shiftData;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
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

EXPORTCH int recordDistanceEnd_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    robotJointId_t id;
    int *num;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    num = Ch_VaArg(interp, ap, int* );
    retval = mobot->recordDistanceEnd(id, *num);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int recordDistancesBegin_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double** time;
    double** angle1;
    double** angle2;
    double** angle3;
    double** angle4;
    double radius;
    double seconds;
    int shiftData;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    time = Ch_VaArg(interp, ap, double**);
    angle1 = Ch_VaArg(interp, ap, double**);
    angle2 = Ch_VaArg(interp, ap, double**);
    angle3 = Ch_VaArg(interp, ap, double**);
    angle4 = Ch_VaArg(interp, ap, double**);
    radius = Ch_VaArg(interp, ap, double);
    seconds = Ch_VaArg(interp, ap, double);
    if(Ch_VaCount(interp, ap) == 1) {
      shiftData = Ch_VaArg(interp, ap, int);
      retval = mobot->recordDistancesBegin(
          *time, 
          *angle1, 
          *angle2, 
          *angle3, 
          *angle4, 
          radius,
          seconds,
          shiftData);
    } else {
      retval = mobot->recordDistancesBegin(
          *time, 
          *angle1, 
          *angle2, 
          *angle3, 
          *angle4, 
          radius,
          seconds);
    }
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int recordDistancesEnd_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;
    int *num;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    num = Ch_VaArg(interp, ap, int*);
    retval = mobot->recordDistancesEnd(*num);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int recordWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    retval = mobot->recordWait();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int reset_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    retval = mobot->reset();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int resetToZero_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    retval = mobot->resetToZero();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int resetToZeroNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    retval = mobot->resetToZeroNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int setExitState_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    mobotJointState_t dir;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    dir = Ch_VaArg(interp, ap, mobotJointState_t);
    retval = mobot->setExitState(dir);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int setJointMovementStateNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    robotJointId_t id;
    mobotJointState_t dir;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, robotJointId_t );
    dir = Ch_VaArg(interp, ap, mobotJointState_t);
    retval = mobot->setJointMovementStateNB(id, dir);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int setJointMovementStateTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    robotJointId_t id;
    mobotJointState_t dir;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    dir = Ch_VaArg(interp, ap, mobotJointState_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = mobot->setJointMovementStateTime(id, dir, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int setJointSafetyAngle_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->setJointSafetyAngle(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int setJointSafetyAngleTimeout_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    seconds = Ch_VaArg(interp, ap, double);
    retval = mobot->setJointSafetyAngleTimeout(seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int setJointSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    robotJointId_t id;
    double speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    speed = Ch_VaArg(interp, ap, double);
    retval = mobot->setJointSpeed(id, speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int setJointSpeeds_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    robotJointId_t id;
    double speed1;
    double speed2;
    double speed3;
    double speed4;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    speed1 = Ch_VaArg(interp, ap, double);
    speed2 = Ch_VaArg(interp, ap, double);
    speed3 = Ch_VaArg(interp, ap, double);
    speed4 = Ch_VaArg(interp, ap, double);
    retval = mobot->setJointSpeeds(speed1, speed2, speed3, speed4);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int setJointSpeedRatio_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    robotJointId_t id;
    double speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    speed = Ch_VaArg(interp, ap, double);
    retval = mobot->setJointSpeedRatio(id, speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int setJointSpeedRatios_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double ratio1;
    double ratio2;
    double ratio3;
    double ratio4;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    ratio1 = Ch_VaArg(interp, ap, double );
    ratio2 = Ch_VaArg(interp, ap, double );
    ratio3 = Ch_VaArg(interp, ap, double );
    ratio4 = Ch_VaArg(interp, ap, double );
    retval = mobot->setJointSpeedRatios(ratio1, ratio2, ratio3, ratio4);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int setMotorPower_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    robotJointId_t id;
    int power;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    power = Ch_VaArg(interp, ap, int);
    retval = mobot->setMotorPower(id, power);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int setMovementStateNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    mobotJointState_t dir1;
    mobotJointState_t dir2;
    mobotJointState_t dir3;
    mobotJointState_t dir4;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    dir1 = (mobotJointState_t)Ch_VaArg(interp, ap, int);
    dir2 = (mobotJointState_t)Ch_VaArg(interp, ap, int);
    dir3 = (mobotJointState_t)Ch_VaArg(interp, ap, int);
    dir4 = (mobotJointState_t)Ch_VaArg(interp, ap, int);
    retval = mobot->setMovementStateNB(dir1, dir2, dir3, dir4);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int setMovementStateTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    mobotJointState_t dir1;
    mobotJointState_t dir2;
    mobotJointState_t dir3;
    mobotJointState_t dir4;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    dir1 = Ch_VaArg(interp, ap, mobotJointState_t);
    dir2 = Ch_VaArg(interp, ap, mobotJointState_t);
    dir3 = Ch_VaArg(interp, ap, mobotJointState_t);
    dir4 = Ch_VaArg(interp, ap, mobotJointState_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = mobot->setMovementStateTime(dir1, dir2, dir3, dir4, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int setMovementStateTimeNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    mobotJointState_t dir1;
    mobotJointState_t dir2;
    mobotJointState_t dir3;
    mobotJointState_t dir4;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    dir1 = Ch_VaArg(interp, ap, mobotJointState_t);
    dir2 = Ch_VaArg(interp, ap, mobotJointState_t);
    dir3 = Ch_VaArg(interp, ap, mobotJointState_t);
    dir4 = Ch_VaArg(interp, ap, mobotJointState_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = mobot->setMovementStateTimeNB(dir1, dir2, dir3, dir4, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int setTwoWheelRobotSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double speed;
    double radius;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    speed = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    retval = mobot->setTwoWheelRobotSpeed(speed, radius);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int stopOneJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    robotJointId_t id;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    retval = mobot->stopOneJoint(id);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int stopTwoJoints_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    robotJointId_t id1;
    robotJointId_t id2;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id1 = Ch_VaArg(interp, ap, robotJointId_t);
    id2 = Ch_VaArg(interp, ap, robotJointId_t);
    retval = mobot->stopTwoJoints(id1, id2);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int stopThreeJoints_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    robotJointId_t id1;
    robotJointId_t id2;
    robotJointId_t id3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id1 = Ch_VaArg(interp, ap, robotJointId_t);
    id2 = Ch_VaArg(interp, ap, robotJointId_t);
    id3 = Ch_VaArg(interp, ap, robotJointId_t);
    retval = mobot->stopThreeJoints(id1, id2, id3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int stopAllJoints_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    retval = mobot->stopAllJoints();
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

EXPORTCH int turnLeft_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->turnLeft(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int turnLeftNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->turnLeftNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int turnRight_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->turnRight(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int turnRightNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->turnRightNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int motionArch_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->motionArch(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int motionArchNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->motionArchNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int motionDistance_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double radius;
    double distance;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    distance = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    retval = mobot->motionDistance(distance, radius);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int motionDistanceNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double radius;
    double distance;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    distance = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    retval = mobot->motionDistanceNB(distance, radius);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int motionInchwormLeft_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int num;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    num = Ch_VaArg(interp, ap, int);
    retval = mobot->motionInchwormLeft(num);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int motionInchwormLeftNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int num;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    num = Ch_VaArg(interp, ap, int);
    retval = mobot->motionInchwormLeftNB(num);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int motionInchwormRight_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int num;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    num = Ch_VaArg(interp, ap, int);
    retval = mobot->motionInchwormRight(num);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int motionInchwormRightNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int num;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    num = Ch_VaArg(interp, ap, int);
    retval = mobot->motionInchwormRightNB(num);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int motionRollBackward_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->motionRollBackward(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int motionRollBackwardNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->motionRollBackwardNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int motionRollForward_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->motionRollForward(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int motionRollForwardNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double angle;
    int retval;
    
    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->motionRollForwardNB(angle);
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

EXPORTCH int motionTurnLeft_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->motionTurnLeft(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int motionTurnLeftNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->motionTurnLeftNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int motionTurnRight_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->motionTurnRight(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int motionTurnRightNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->motionTurnRightNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int motionTumbleLeft_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int num;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    num = Ch_VaArg(interp, ap, int);
    retval = mobot->motionTumbleLeft(num);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int motionTumbleLeftNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int num;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    num = Ch_VaArg(interp, ap, int);
    retval = mobot->motionTumbleLeftNB(num);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int motionTumbleRight_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int num;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    num = Ch_VaArg(interp, ap, int);
    retval = mobot->motionTumbleRight(num);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int motionTumbleRightNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int num;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    num = Ch_VaArg(interp, ap, int);
    retval = mobot->motionTumbleRightNB(num);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int motionUnstand_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    retval = mobot->motionUnstand();
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int motionUnstandNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    retval = mobot->motionUnstandNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int motionWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    retval = mobot->motionWait();
    Ch_VaEnd(interp, ap);
    return retval;
}

/* CMobotGroup functions */

EXPORTCH void CMobotGroup_CMobotGroup_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class CMobotGroup *c=new CMobotGroup();
  Ch_VaStart(interp, ap, varg);
  Ch_CppChangeThisPointer(interp, c, sizeof(CMobotGroup));
  Ch_VaEnd(interp, ap);
}

EXPORTCH void CMobotGroup_dCMobotGroup_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class CMobotGroup *c;
  Ch_VaStart(interp, ap, varg);
  c = Ch_VaArg(interp, ap, class CMobotGroup *);
  if(Ch_CppIsArrayElement(interp))
    c->~CMobotGroup();
  else
    delete c;
  Ch_VaEnd(interp, ap);
  return;
}

EXPORTCH int CMG_addRobot_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    class CMobot *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    robot = Ch_VaArg(interp, ap, class CMobot*);
    retval = mobot->addRobot(*robot);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_addRobots_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    class CMobot *robot;
    int numRobots;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    robot = Ch_VaArg(interp, ap, class CMobot*);
    numRobots = Ch_VaArg(interp, ap, int);
    retval = mobot->addRobots(robot, numRobots);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_connect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    retval = mobot->connect();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_driveJointToDirect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->driveJointToDirect(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_driveToDirect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    double angle1;
    double angle2;
    double angle3;
    double angle4;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    angle4 = Ch_VaArg(interp, ap, double);
    retval = mobot->driveToDirect(angle1, angle2, angle3, angle4);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CMG_driveTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    double angle1;
    double angle2;
    double angle3;
    double angle4;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    angle4 = Ch_VaArg(interp, ap, double);
    retval = mobot->driveTo(angle1, angle2, angle3, angle4);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_driveToDirectNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    double angle1;
    double angle2;
    double angle3;
    double angle4;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    angle4 = Ch_VaArg(interp, ap, double);
    retval = mobot->driveToDirectNB(angle1, angle2, angle3, angle4);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CMG_driveToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    double angle1;
    double angle2;
    double angle3;
    double angle4;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    angle4 = Ch_VaArg(interp, ap, double);
    retval = mobot->driveToNB(angle1, angle2, angle3, angle4);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_driveJointToDirectNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->driveJointToDirectNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_isMoving_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    retval = mobot->isMoving();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_move_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    double angle1;
    double angle2;
    double angle3;
    double angle4;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    angle4 = Ch_VaArg(interp, ap, double);
    retval = mobot->move(angle1, angle2, angle3, angle4);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_moveNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    double angle1;
    double angle2;
    double angle3;
    double angle4;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    angle4 = Ch_VaArg(interp, ap, double);
    retval = mobot->moveNB(angle1, angle2, angle3, angle4);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_moveBackward_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveBackward(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_moveBackwardNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveBackwardNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_moveContinuousNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    mobotJointState_t dir1;
    mobotJointState_t dir2;
    mobotJointState_t dir3;
    mobotJointState_t dir4;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    dir1 = (mobotJointState_t)Ch_VaArg(interp, ap, int);
    dir2 = (mobotJointState_t)Ch_VaArg(interp, ap, int);
    dir3 = (mobotJointState_t)Ch_VaArg(interp, ap, int);
    dir4 = (mobotJointState_t)Ch_VaArg(interp, ap, int);
    retval = mobot->moveContinuousNB(dir1, dir2, dir3, dir4);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_moveContinuousTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    mobotJointState_t dir1;
    mobotJointState_t dir2;
    mobotJointState_t dir3;
    mobotJointState_t dir4;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    dir1 = Ch_VaArg(interp, ap, mobotJointState_t);
    dir2 = Ch_VaArg(interp, ap, mobotJointState_t);
    dir3 = Ch_VaArg(interp, ap, mobotJointState_t);
    dir4 = Ch_VaArg(interp, ap, mobotJointState_t);
    seconds = Ch_VaArg(interp, ap, double );
    retval = mobot->moveContinuousTime(dir1, dir2, dir3, dir4, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_moveForward_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveForward(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_moveForwardNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveForwardNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_moveJointContinuousNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    robotJointId_t id;
    mobotJointState_t dir;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t );
    dir = Ch_VaArg(interp, ap, mobotJointState_t);
    retval = mobot->moveJointContinuousNB(id, dir);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_moveJointContinuousTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    robotJointId_t id;
    mobotJointState_t dir;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    dir = Ch_VaArg(interp, ap, mobotJointState_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointContinuousTime(id, dir, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_moveJointTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointTo(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_moveJointToDirect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointToDirect(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_moveJointToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointToNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_moveJointToDirectNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointToDirectNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_moveJointWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    robotJointId_t id;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    retval = mobot->moveJointWait(id);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_moveTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    double angle1;
    double angle2;
    double angle3;
    double angle4;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    angle4 = Ch_VaArg(interp, ap, double);
    retval = mobot->moveTo(angle1, angle2, angle3, angle4);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_moveToDirect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    double angle1;
    double angle2;
    double angle3;
    double angle4;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    angle4 = Ch_VaArg(interp, ap, double);
    retval = mobot->moveToDirect(angle1, angle2, angle3, angle4);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_moveToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    double angle1;
    double angle2;
    double angle3;
    double angle4;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    angle4 = Ch_VaArg(interp, ap, double);
    retval = mobot->moveToNB(angle1, angle2, angle3, angle4);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_moveToDirectNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    double angle1;
    double angle2;
    double angle3;
    double angle4;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    angle4 = Ch_VaArg(interp, ap, double);
    retval = mobot->moveToDirectNB(angle1, angle2, angle3, angle4);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_moveWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    retval = mobot->moveWait();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_moveToZero_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    retval = mobot->moveToZero();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_moveToZeroNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    retval = mobot->moveToZeroNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_reset_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    retval = mobot->reset();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_resetToZero_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    retval = mobot->resetToZero();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_resetToZeroNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    retval = mobot->resetToZeroNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_setExitState_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    mobotJointState_t exitState;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    exitState = Ch_VaArg(interp, ap, mobotJointState_t);
    retval = mobot->setExitState(exitState);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_setJointMovementStateNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    robotJointId_t id;
    mobotJointState_t dir;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t );
    dir = Ch_VaArg(interp, ap, mobotJointState_t);
    retval = mobot->setJointMovementStateNB(id, dir);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_setJointMovementStateTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    robotJointId_t id;
    mobotJointState_t dir;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    dir = Ch_VaArg(interp, ap, mobotJointState_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = mobot->setJointMovementStateTime(id, dir, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_setJointMovementStateTimeNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    robotJointId_t id;
    mobotJointState_t dir;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    dir = Ch_VaArg(interp, ap, mobotJointState_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = mobot->setJointMovementStateTimeNB(id, dir, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_setJointSafetyAngle_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->setJointSafetyAngle(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_setJointSafetyAngleTimeout_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->setJointSafetyAngleTimeout(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_setJointSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    robotJointId_t id;
    double speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    speed = Ch_VaArg(interp, ap, double);
    retval = mobot->setJointSpeed(id, speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_setJointSpeeds_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    double speed1, speed2, speed3, speed4;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    speed1 = Ch_VaArg(interp, ap, double);
    speed2 = Ch_VaArg(interp, ap, double);
    speed3 = Ch_VaArg(interp, ap, double);
    speed4 = Ch_VaArg(interp, ap, double);
    retval = mobot->setJointSpeeds(speed1, speed2, speed3, speed4);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_setJointSpeedRatio_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    robotJointId_t id;
    double speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    speed = Ch_VaArg(interp, ap, double);
    retval = mobot->setJointSpeedRatio(id, speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_setJointSpeedRatios_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    robotJointId_t id;
    double ratio1;
    double ratio2;
    double ratio3;
    double ratio4;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    ratio1 = Ch_VaArg(interp, ap, double);
    ratio2 = Ch_VaArg(interp, ap, double);
    ratio3 = Ch_VaArg(interp, ap, double);
    ratio4 = Ch_VaArg(interp, ap, double);
    retval = mobot->setJointSpeedRatios(ratio1, ratio2, ratio3, ratio4);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_setMovementStateNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    mobotJointState_t dir1;
    mobotJointState_t dir2;
    mobotJointState_t dir3;
    mobotJointState_t dir4;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    dir1 = (mobotJointState_t)Ch_VaArg(interp, ap, int);
    dir2 = (mobotJointState_t)Ch_VaArg(interp, ap, int);
    dir3 = (mobotJointState_t)Ch_VaArg(interp, ap, int);
    dir4 = (mobotJointState_t)Ch_VaArg(interp, ap, int);
    retval = mobot->setMovementStateNB(dir1, dir2, dir3, dir4);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_setMovementStateTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    mobotJointState_t dir1;
    mobotJointState_t dir2;
    mobotJointState_t dir3;
    mobotJointState_t dir4;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    dir1 = Ch_VaArg(interp, ap, mobotJointState_t);
    dir2 = Ch_VaArg(interp, ap, mobotJointState_t);
    dir3 = Ch_VaArg(interp, ap, mobotJointState_t);
    dir4 = Ch_VaArg(interp, ap, mobotJointState_t);
    seconds = Ch_VaArg(interp, ap, double );
    retval = mobot->setMovementStateTime(dir1, dir2, dir3, dir4, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_setMovementStateTimeNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    mobotJointState_t dir1;
    mobotJointState_t dir2;
    mobotJointState_t dir3;
    mobotJointState_t dir4;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    dir1 = Ch_VaArg(interp, ap, mobotJointState_t);
    dir2 = Ch_VaArg(interp, ap, mobotJointState_t);
    dir3 = Ch_VaArg(interp, ap, mobotJointState_t);
    dir4 = Ch_VaArg(interp, ap, mobotJointState_t);
    seconds = Ch_VaArg(interp, ap, double );
    retval = mobot->setMovementStateTimeNB(dir1, dir2, dir3, dir4, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_setTwoWheelRobotSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    double speed;
    double radius;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    speed = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    retval = mobot->setTwoWheelRobotSpeed(speed, radius);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_stopAllJoints_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    retval = mobot->stopAllJoints();
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CMG_stopOneJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    robotJointId_t id;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    retval = mobot->stopOneJoint(id);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CMG_stopTwoJoints_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    robotJointId_t id1, id2;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    id1 = Ch_VaArg(interp, ap, robotJointId_t);
    id2 = Ch_VaArg(interp, ap, robotJointId_t);
    retval = mobot->stopTwoJoints(id1, id2);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CMG_stopThreeJoints_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    robotJointId_t id1, id2, id3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    id1 = Ch_VaArg(interp, ap, robotJointId_t);
    id2 = Ch_VaArg(interp, ap, robotJointId_t);
    id3 = Ch_VaArg(interp, ap, robotJointId_t);
    retval = mobot->stopThreeJoints(id1, id2, id3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_turnLeft_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->turnLeft(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_turnLeftNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->turnLeftNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_turnRight_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->turnRight(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_turnRightNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->turnRightNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_motionArch_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->motionArch(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CMG_motionArchNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->motionArchNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_motionDistance_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    double distance, radius;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    distance = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    retval = mobot->motionDistance(distance, radius);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CMG_motionDistanceNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    double distance, radius;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    distance = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    retval = mobot->motionDistanceNB(distance, radius);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_motionInchwormLeft_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    int num;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    num = Ch_VaArg(interp, ap, int);
    retval = mobot->motionInchwormLeft(num);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CMG_motionInchwormLeftNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    int num;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    num = Ch_VaArg(interp, ap, int);
    retval = mobot->motionInchwormLeftNB(num);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_motionInchwormRight_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    int num;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    num = Ch_VaArg(interp, ap, int);
    retval = mobot->motionInchwormRight(num);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CMG_motionInchwormRightNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    int num;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    num = Ch_VaArg(interp, ap, int);
    retval = mobot->motionInchwormRightNB(num);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_motionRollBackward_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->motionRollBackward(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CMG_motionRollBackwardNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->motionRollBackwardNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_motionRollForward_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->motionRollForward(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CMG_motionRollForwardNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    double angle;
    int retval;
    
    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->motionRollForwardNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_motionSkinny_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->motionSkinny(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CMG_motionSkinnyNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    double angle;
    int retval;
    
    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->motionSkinnyNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_motionStand_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    retval = mobot->motionStand();
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CMG_motionStandNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    retval = mobot->motionStandNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_motionTurnLeft_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->motionTurnLeft(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CMG_motionTurnLeftNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->motionTurnLeftNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_motionTurnRight_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->motionTurnRight(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CMG_motionTurnRightNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->motionTurnRightNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_motionTumbleLeft_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    int num;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    num = Ch_VaArg(interp, ap, int);
    retval = mobot->motionTumbleLeft(num);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CMG_motionTumbleLeftNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    int num;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    num = Ch_VaArg(interp, ap, int);
    retval = mobot->motionTumbleLeftNB(num);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_motionTumbleRight_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    int num;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    num = Ch_VaArg(interp, ap, int);
    retval = mobot->motionTumbleRight(num);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CMG_motionTumbleRightNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    int num;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    num = Ch_VaArg(interp, ap, int);
    retval = mobot->motionTumbleRightNB(num);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_motionUnstand_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    retval = mobot->motionUnstand();
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CMG_motionUnstandNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    retval = mobot->motionUnstandNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMG_motionWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobotGroup *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobotGroup *);
    retval = mobot->motionWait();
    Ch_VaEnd(interp, ap);
    return retval;
}

