#include <mobot.h>
#include <linkbot.h>
#include <ch.h>

EXPORTCH void CLinkbotL_CLinkbotL_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class CLinkbotL *c=new CLinkbotL();
  Ch_VaStart(interp, ap, varg);
  Ch_CppChangeThisPointer(interp, c, sizeof(CLinkbotL));
  Ch_VaEnd(interp, ap);
}

EXPORTCH void CLinkbotL_dCLinkbotL_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class CLinkbotL *c;
  Ch_VaStart(interp, ap, varg);
  c = Ch_VaArg(interp, ap, class CLinkbotL *);
  if(Ch_CppIsArrayElement(interp))
    c->~CLinkbotL();
  else
    delete c;
  Ch_VaEnd(interp, ap);
  return;
}

EXPORTCH int LinkbotL_accelTimeNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    double radius;
    double acceleration;
    double timeout;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    radius = Ch_VaArg(interp, ap, double);
    acceleration = Ch_VaArg(interp, ap, double);
    timeout = Ch_VaArg(interp, ap, double);
    retval = mobot->accelTimeNB(radius, acceleration, timeout);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_accelToVelocityNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    double radius;
    double acceleration;
    double v;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    radius = Ch_VaArg(interp, ap, double);
    acceleration = Ch_VaArg(interp, ap, double);
    v = Ch_VaArg(interp, ap, double);
    retval = mobot->accelToVelocityNB(radius, acceleration, v);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_accelToMaxSpeedNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    double radius;
    double acceleration;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    radius = Ch_VaArg(interp, ap, double);
    acceleration = Ch_VaArg(interp, ap, double);
    retval = mobot->accelToMaxSpeedNB(radius, acceleration);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_accelAngularTimeNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int id;
    double acceleration;
    double time;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, int);
    acceleration = Ch_VaArg(interp, ap, double);
    time = Ch_VaArg(interp, ap, double);
    retval = mobot->accelAngularTimeNB((robotJointId_t)id, acceleration, time);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_accelAngularToVelocityNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int id;
    double acceleration;
    double v;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, int);
    acceleration = Ch_VaArg(interp, ap, double);
    v = Ch_VaArg(interp, ap, double);
    retval = mobot->accelAngularToVelocityNB((robotJointId_t)id, acceleration, v);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_accelAngularAngleNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int id;
    double acceleration;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, int);
    acceleration = Ch_VaArg(interp, ap, double);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->accelAngularAngleNB((robotJointId_t)id, acceleration, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}


EXPORTCH int LinkbotL_accelAngularCycloidNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
	int id;
    double angle;
    double timeout;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
	id = Ch_VaArg(interp, ap, int); 
    angle = Ch_VaArg(interp, ap, double);
    timeout = Ch_VaArg(interp, ap, double);
    retval = mobot->accelAngularCycloidNB((robotJointId_t)id, angle, timeout);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_accelAngularHarmonicNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
	int id;
    double angle;
    double timeout;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
	id = Ch_VaArg(interp, ap, int); 
    angle = Ch_VaArg(interp, ap, double);
    timeout = Ch_VaArg(interp, ap, double);
    retval = mobot->accelAngularHarmonicNB((robotJointId_t)id, angle, timeout);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_smoothMoveToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int id;
    double accel0;
    double accelf;
    double vmax;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, int);
    accel0 = Ch_VaArg(interp, ap, double);
    accelf = Ch_VaArg(interp, ap, double);
    vmax = Ch_VaArg(interp, ap, double);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->smoothMoveToNB((robotJointId_t)id, accel0, accelf, vmax, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_blinkLED_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    double delay;
    int numBlinks;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    delay = Ch_VaArg(interp, ap, double);
    numBlinks = Ch_VaArg(interp, ap, int);
    retval = mobot->blinkLED(delay, numBlinks);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_connect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = mobot->connect();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_connectWithAddress_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    char *address;
    int channel;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
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

EXPORTCH int LinkbotL_connectWithIPAddress_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    char *address;
    char *port;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
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

EXPORTCH int LinkbotL_connectWithSerialID_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    char *address;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    address = Ch_VaArg(interp, ap, char *);
    retval = mobot->connectWithSerialID(address);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_disconnect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = mobot->disconnect();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_disableButtonCallback_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = mobot->disableButtonCallback();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_enableButtonCallback_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    void *data;
    void (*cb)(void*,int,int);
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    data = Ch_VaArg(interp, ap, void*);
    cb = (void(*)(void*,int,int))Ch_VaArg(interp, ap, void*);
    retval = mobot->enableButtonCallback(data, cb);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_enableRecordDataShift_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = mobot->enableRecordDataShift();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_disableRecordDataShift_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = mobot->disableRecordDataShift();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_isConnected_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = mobot->isConnected();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_isMoving_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = mobot->isMoving();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_getAccelerometerData_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    double *x, *y, *z;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    x = Ch_VaArg(interp, ap, double *);
    y = Ch_VaArg(interp, ap, double *);
    z = Ch_VaArg(interp, ap, double *);
    retval = mobot->getAccelerometerData(*x, *y, *z);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_getBatteryVoltage_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    double* voltage;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    voltage = Ch_VaArg(interp, ap, double *);
    retval = mobot->getBatteryVoltage(*voltage);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_LinkPodAnalogRead_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int pin;
    //int * value;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    pin = Ch_VaArg(interp, ap, int );
    //value = Ch_VaArg(interp, ap, int *);
    //retval = mobot->LinkPodAnalogRead(pin, *value);
	retval = mobot->LinkPodAnalogRead(pin);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH double LinkbotL_LinkPodAnalogReadVolts_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int pin;
    //double * value;
    double retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    pin = Ch_VaArg(interp, ap, int );
    //value = Ch_VaArg(interp, ap, double *);
    //retval = mobot->LinkPodAnalogReadVolts(pin, *value);
	retval = mobot->LinkPodAnalogReadVolts(pin);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_LinkPodDigitalRead_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int pin;
    //int * value;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    pin = Ch_VaArg(interp, ap, int );
    //value = Ch_VaArg(interp, ap, int *);
    //retval = mobot->LinkPodDigitalRead(pin, *value);
	retval = mobot->LinkPodDigitalRead(pin);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_getColorRGB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int *r, *g, *b;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    r = Ch_VaArg(interp, ap, int *);
    g = Ch_VaArg(interp, ap, int *);
    b = Ch_VaArg(interp, ap, int *);
    retval = mobot->getColorRGB(*r, *g, *b);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_getColorName_chdl(void *varg) { 
	ChInterp_t interp; 
	ChVaList_t ap; 
	class CLinkbot *mobot; 
	char * color; 
	int retval; 

	Ch_VaStart(interp, ap, varg); 
	mobot = Ch_VaArg(interp, ap, class CLinkbot *); 
	color = Ch_VaArg(interp, ap, char *); 
	retval = mobot->getColorName(color); 
	Ch_VaEnd(interp, ap); 
	return retval; 
} 

//new

EXPORTCH int LinkbotL_getLEDColorRGB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int *r, *g, *b;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    r = Ch_VaArg(interp, ap, int *);
    g = Ch_VaArg(interp, ap, int *);
    b = Ch_VaArg(interp, ap, int *);
    retval = mobot->getColorRGB(*r, *g, *b);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_getLEDColorName_chdl(void *varg) { 
	ChInterp_t interp; 
	ChVaList_t ap; 
	class CLinkbot *mobot; 
	char * color; 
	int retval; 

	Ch_VaStart(interp, ap, varg); 
	mobot = Ch_VaArg(interp, ap, class CLinkbot *); 
	color = Ch_VaArg(interp, ap, char *); 
	retval = mobot->getColorName(color); 
	Ch_VaEnd(interp, ap); 
	return retval; 
} 

//end new
EXPORTCH int LinkbotL_getFormFactor_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int* formFactor;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    formFactor = Ch_VaArg(interp, ap, int *);
    retval = mobot->getFormFactor(*formFactor);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_getJointAngle_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int id;
    double* angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, int);
    angle = Ch_VaArg(interp, ap, double *);
    retval = mobot->getJointAngle((robotJointId_t)id, *angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_getJointAngleAverage_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int id;
    double* angle;
    int numReadings;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
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

EXPORTCH int LinkbotL_getJointAngles_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    double* angle1;
    double* angle2;
    double* angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle1 = Ch_VaArg(interp, ap, double *);
    angle2 = Ch_VaArg(interp, ap, double *);
    angle3 = Ch_VaArg(interp, ap, double *);
    retval = mobot->getJointAngles(*angle1, *angle2, *angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_getJointAnglesAverage_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    double* angle1;
    double* angle2;
    double* angle3;
    int numReadings;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
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

EXPORTCH int LinkbotL_getJointMaxSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int id;
    double *speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, int);
    speed = Ch_VaArg(interp, ap, double *);
    retval = mobot->getJointMaxSpeed((robotJointId_t)id, *speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_getJointSafetyAngle_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    double* angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle = Ch_VaArg(interp, ap, double *);
    retval = mobot->getJointSafetyAngle(*angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_getJointSafetyAngleTimeout_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    double* seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    seconds = Ch_VaArg(interp, ap, double *);
    retval = mobot->getJointSafetyAngleTimeout(*seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_getJointSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int id;
    double *speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, int);
    speed = Ch_VaArg(interp, ap, double *);
    retval = mobot->getJointSpeed((robotJointId_t)id, *speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_getJointSpeeds_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    double *speed1;
    double *speed2;
    double *speed3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    speed1 = Ch_VaArg(interp, ap, double *);
    speed2 = Ch_VaArg(interp, ap, double *);
    speed3 = Ch_VaArg(interp, ap, double *);
    retval = mobot->getJointSpeeds(*speed1, *speed2, *speed3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_getJointSpeedRatio_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int id;
    double *speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, int);
    speed = Ch_VaArg(interp, ap, double *);
    retval = mobot->getJointSpeedRatio((robotJointId_t)id, *speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_getJointSpeedRatios_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    double *ratio1;
    double *ratio2;
    double *ratio3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    ratio1 = Ch_VaArg(interp, ap, double *);
    ratio2 = Ch_VaArg(interp, ap, double *);
    ratio3 = Ch_VaArg(interp, ap, double *);
    retval = mobot->getJointSpeedRatios(*ratio1, *ratio2, *ratio3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_getJointState_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int id;
    int * state;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, int);
    state = Ch_VaArg(interp, ap, int *);
    retval = mobot->getJointState((robotJointId_t)id, (robotJointState_t&)(*state));
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_move_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = mobot->move(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_moveNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = mobot->moveNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_moveContinuousNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    robotJointState_t dir1;
    robotJointState_t dir2;
    robotJointState_t dir3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    dir1 = (robotJointState_t)Ch_VaArg(interp, ap, int);
    dir2 = (robotJointState_t)Ch_VaArg(interp, ap, int);
    dir3 = (robotJointState_t)Ch_VaArg(interp, ap, int);
    retval = mobot->moveContinuousNB(dir1, dir2, dir3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_moveContinuousTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    robotJointState_t dir1;
    robotJointState_t dir2;
    robotJointState_t dir3;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    dir1 = Ch_VaArg(interp, ap, robotJointState_t);
    dir2 = Ch_VaArg(interp, ap, robotJointState_t);
    dir3 = Ch_VaArg(interp, ap, robotJointState_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = mobot->moveContinuousTime(dir1, dir2, dir3, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_moveJointContinuousNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    robotJointId_t id;
    robotJointState_t dir;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, robotJointId_t );
    dir = Ch_VaArg(interp, ap, robotJointState_t);
    retval = mobot->moveJointContinuousNB(id, dir);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_moveJointContinuousTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    robotJointId_t id;
    robotJointState_t dir;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    dir = Ch_VaArg(interp, ap, robotJointState_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointContinuousTime(id, dir, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_moveJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJoint(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_moveJointNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_moveJointTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointTo(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_moveJointToDirect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointToDirect(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_moveJointToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointToNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_moveJointToDirectNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointToDirectNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_moveJointWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    robotJointId_t id;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    retval = mobot->moveJointWait(id);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_moveTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = mobot->moveTo(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_moveToDirect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = mobot->moveToDirect(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_driveToDirect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = mobot->driveToDirect(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int LinkbotL_driveTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = mobot->driveTo(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_moveToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = mobot->moveToNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_moveToDirectNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = mobot->moveToDirectNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_driveToDirectNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = mobot->driveToDirectNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int LinkbotL_driveToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = mobot->driveToNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_moveWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = mobot->moveWait();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_moveToZero_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = mobot->moveToZero();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_moveToZeroNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = mobot->moveToZeroNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_recordAngle_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    robotJointId_t id;
    double* time;
    double* angle;
    int num;
    double seconds;
    int shiftData;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
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

EXPORTCH int LinkbotL_recordAngleBegin_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    robotJointId_t id;
    double** time;
    double** angle;
    double seconds;
    int shiftData;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
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

EXPORTCH int LinkbotL_recordAngleEnd_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    robotJointId_t id;
    int *num;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    num = Ch_VaArg(interp, ap, int* );
    retval = mobot->recordAngleEnd(id, *num);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_recordAngles_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    double* time;
    double* angle1;
    double* angle2;
    double* angle3;
    int num;
    double seconds;
    int shiftData;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
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

EXPORTCH int LinkbotL_recordAnglesBegin_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    double** time;
    double** angle1;
    double** angle2;
    double** angle3;
    double seconds;
    int shiftData;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
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

EXPORTCH int LinkbotL_recordAnglesEnd_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int retval;
    int *num;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    num = Ch_VaArg(interp, ap, int*);
    retval = mobot->recordAnglesEnd(*num);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_recordDistanceBegin_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    robotJointId_t id;
    double** time;
    double** angle;
    double radius;
    double seconds;
    int shiftData;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
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

EXPORTCH int LinkbotL_recordDistanceEnd_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    robotJointId_t id;
    int *num;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    num = Ch_VaArg(interp, ap, int* );
    retval = mobot->recordDistanceEnd(id, *num);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_recordDistancesBegin_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    double** time;
    double** angle1;
    double** angle2;
    double** angle3;
    double radius;
    double seconds;
    int shiftData;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
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

EXPORTCH int LinkbotL_recordDistancesEnd_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int retval;
    int *num;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    num = Ch_VaArg(interp, ap, int*);
    retval = mobot->recordDistancesEnd(*num);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_recordWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = mobot->recordWait();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_reset_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = mobot->reset();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_resetToZero_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = mobot->resetToZero();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_resetToZeroNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = mobot->resetToZeroNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_LinkPodAnalogWrite_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int pin;
    int value;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    pin = Ch_VaArg(interp, ap, int);
    value = Ch_VaArg(interp, ap, int);
    retval = mobot->LinkPodAnalogWrite(pin, value);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_LinkPodAnalogReference_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int value;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    value = Ch_VaArg(interp, ap, int);
    retval = mobot->LinkPodAnalogReference(value);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_LinkPodDigitalWrite_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int pin;
    int value;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    pin = Ch_VaArg(interp, ap, int);
    value = Ch_VaArg(interp, ap, int);
    retval = mobot->LinkPodDigitalWrite(pin, value);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_LinkPodPinMode_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int pin;
    int value;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    pin = Ch_VaArg(interp, ap, int);
    value = Ch_VaArg(interp, ap, int);
    retval = mobot->LinkPodPinMode(pin, value);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_setBuzzerFrequency_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int frequency;
    double time;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    frequency = Ch_VaArg(interp, ap, int);
    time = Ch_VaArg(interp, ap, double);
    retval = mobot->setBuzzerFrequency(frequency, time);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_setBuzzerFrequencyOn_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int frequency;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    frequency = Ch_VaArg(interp, ap, int);
    retval = mobot->setBuzzerFrequencyOn(frequency);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_setBuzzerFrequencyOff_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = mobot->setBuzzerFrequencyOff();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_setColorRGB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int r, g, b;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    r = Ch_VaArg(interp, ap, int);
    g = Ch_VaArg(interp, ap, int);
    b = Ch_VaArg(interp, ap, int);
    retval = mobot->setColorRGB(r, g, b);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_setColor_chdl(void *varg) { 
	ChInterp_t interp; 
	ChVaList_t ap; 
	class CLinkbotL *mobot; 
	char *color; 
	int retval; 

	Ch_VaStart(interp, ap, varg); 
	mobot = Ch_VaArg(interp, ap, class CLinkbotL *); 
	color = Ch_VaArg(interp, ap, char *);  
	retval = mobot->setColor(color); 
	Ch_VaEnd(interp, ap); 
	return retval; 
}

//new
EXPORTCH int LinkbotL_setLEDColorRGB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int r, g, b;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    r = Ch_VaArg(interp, ap, int);
    g = Ch_VaArg(interp, ap, int);
    b = Ch_VaArg(interp, ap, int);
    retval = mobot->setColorRGB(r, g, b);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_setLEDColor_chdl(void *varg) { 
	ChInterp_t interp; 
	ChVaList_t ap; 
	class CLinkbotL *mobot; 
	char *color; 
	int retval; 

	Ch_VaStart(interp, ap, varg); 
	mobot = Ch_VaArg(interp, ap, class CLinkbotL *); 
	color = Ch_VaArg(interp, ap, char *);  
	retval = mobot->setColor(color); 
	Ch_VaEnd(interp, ap); 
	return retval; 
}
//end new
EXPORTCH int LinkbotL_setExitState_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    robotJointState_t dir;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    dir = Ch_VaArg(interp, ap, robotJointState_t);
    retval = mobot->setExitState(dir);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_setJointMovementStateNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    robotJointId_t id;
    robotJointState_t dir;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, robotJointId_t );
    dir = Ch_VaArg(interp, ap, robotJointState_t);
    retval = mobot->setJointMovementStateNB(id, dir);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_setJointMovementStateTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    robotJointId_t id;
    robotJointState_t dir;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    dir = Ch_VaArg(interp, ap, robotJointState_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = mobot->setJointMovementStateTime(id, dir, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_setJointSafetyAngle_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->setJointSafetyAngle(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_setJointSafetyAngleTimeout_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    seconds = Ch_VaArg(interp, ap, double);
    retval = mobot->setJointSafetyAngleTimeout(seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_setJointSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    robotJointId_t id;
    double speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    speed = Ch_VaArg(interp, ap, double);
    retval = mobot->setJointSpeed(id, speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_setJointSpeeds_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    robotJointId_t id;
    double speed1;
    double speed2;
    double speed3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    speed1 = Ch_VaArg(interp, ap, double);
    speed2 = Ch_VaArg(interp, ap, double);
    speed3 = Ch_VaArg(interp, ap, double);
    retval = mobot->setJointSpeeds(speed1, speed2, speed3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_setJointSpeedRatio_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    robotJointId_t id;
    double speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    speed = Ch_VaArg(interp, ap, double);
    retval = mobot->setJointSpeedRatio(id, speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_setJointSpeedRatios_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    double ratio1;
    double ratio2;
    double ratio3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    ratio1 = Ch_VaArg(interp, ap, double );
    ratio2 = Ch_VaArg(interp, ap, double );
    ratio3 = Ch_VaArg(interp, ap, double );
    retval = mobot->setJointSpeedRatios(ratio1, ratio2, ratio3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_setMotorPower_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    robotJointId_t id;
    int power;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    power = Ch_VaArg(interp, ap, int);
    retval = mobot->setMotorPower(id, power);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_setMovementStateNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    robotJointState_t dir1;
    robotJointState_t dir2;
    robotJointState_t dir3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    dir1 = (robotJointState_t)Ch_VaArg(interp, ap, int);
    dir2 = (robotJointState_t)Ch_VaArg(interp, ap, int);
    dir3 = (robotJointState_t)Ch_VaArg(interp, ap, int);
    retval = mobot->setMovementStateNB(dir1, dir2, dir3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_setMovementStateTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    robotJointState_t dir1;
    robotJointState_t dir2;
    robotJointState_t dir3;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    dir1 = Ch_VaArg(interp, ap, robotJointState_t);
    dir2 = Ch_VaArg(interp, ap, robotJointState_t);
    dir3 = Ch_VaArg(interp, ap, robotJointState_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = mobot->setMovementStateTime(dir1, dir2, dir3, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_setMovementStateTimeNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    robotJointState_t dir1;
    robotJointState_t dir2;
    robotJointState_t dir3;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    dir1 = Ch_VaArg(interp, ap, robotJointState_t);
    dir2 = Ch_VaArg(interp, ap, robotJointState_t);
    dir3 = Ch_VaArg(interp, ap, robotJointState_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = mobot->setMovementStateTimeNB(dir1, dir2, dir3, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_setTwoWheelRobotSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    double speed;
    double radius;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    speed = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    retval = mobot->setTwoWheelRobotSpeed(speed, radius);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_setSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    double speed;
    double radius;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    speed = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    retval = mobot->setSpeed(speed, radius);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_stopOneJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    robotJointId_t id;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    retval = mobot->stopOneJoint(id);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_stopAllJoints_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = mobot->stopAllJoints();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_stop_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = mobot->stop();
    Ch_VaEnd(interp, ap);
    return retval;
}

/* CLinkbotLGroup functions */

EXPORTCH void CLinkbotLGroup_CLinkbotLGroup_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class CLinkbotLGroup *c=new CLinkbotLGroup();
  Ch_VaStart(interp, ap, varg);
  Ch_CppChangeThisPointer(interp, c, sizeof(CLinkbotLGroup));
  Ch_VaEnd(interp, ap);
}

EXPORTCH void CLinkbotLGroup_dCLinkbotLGroup_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class CLinkbotLGroup *c;
  Ch_VaStart(interp, ap, varg);
  c = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
  if(Ch_CppIsArrayElement(interp))
    c->~CLinkbotLGroup();
  else
    delete c;
  Ch_VaEnd(interp, ap);
  return;
}

EXPORTCH int CMGL_addRobot_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    class CLinkbotL *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    robot = Ch_VaArg(interp, ap, class CLinkbotL*);
    retval = mobot->addRobot(*robot);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_addRobots_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    class CLinkbotL *robots;
    int numRobots;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    robots = Ch_VaArg(interp, ap, class CLinkbotL*);
    numRobots = Ch_VaArg(interp, ap, int);
    retval = mobot->addRobots(robots, numRobots);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_connect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    retval = mobot->connect();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_driveJointToDirect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->driveJointToDirect(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_driveToDirect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = mobot->driveToDirect(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CMGL_driveTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = mobot->driveTo(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_driveToDirectNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = mobot->driveToDirectNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CMGL_driveToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = mobot->driveToNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_driveJointToDirectNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->driveJointToDirectNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_isMoving_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    retval = mobot->isMoving();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_move_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = mobot->move(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_moveNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = mobot->moveNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_moveContinuousNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    robotJointState_t dir1;
    robotJointState_t dir2;
    robotJointState_t dir3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    dir1 = (robotJointState_t)Ch_VaArg(interp, ap, int);
    dir2 = (robotJointState_t)Ch_VaArg(interp, ap, int);
    dir3 = (robotJointState_t)Ch_VaArg(interp, ap, int);
    retval = mobot->moveContinuousNB(dir1, dir2, dir3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_moveForeverNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    retval = mobot->moveForeverNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_moveJointForeverNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
	robotJointId_t id;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
    retval = mobot->moveJointForeverNB(id);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_moveContinuousTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    robotJointState_t dir1;
    robotJointState_t dir2;
    robotJointState_t dir3;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    dir1 = Ch_VaArg(interp, ap, robotJointState_t);
    dir2 = Ch_VaArg(interp, ap, robotJointState_t);
    dir3 = Ch_VaArg(interp, ap, robotJointState_t);
    seconds = Ch_VaArg(interp, ap, double );
    retval = mobot->moveContinuousTime(dir1, dir2, dir3, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_moveJointContinuousNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    robotJointId_t id;
    robotJointState_t dir;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t );
    dir = Ch_VaArg(interp, ap, robotJointState_t);
    retval = mobot->moveJointContinuousNB(id, dir);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_moveJointContinuousTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    robotJointId_t id;
    robotJointState_t dir;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    dir = Ch_VaArg(interp, ap, robotJointState_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointContinuousTime(id, dir, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_moveJointTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointTo(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_moveJointToDirect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointToDirect(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_moveJointToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointToNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_moveJointToDirectNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointToDirectNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_moveJointWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    robotJointId_t id;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    retval = mobot->moveJointWait(id);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_moveTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = mobot->moveTo(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_moveToDirect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = mobot->moveToDirect(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_moveToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = mobot->moveToNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_moveToDirectNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = mobot->moveToDirectNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_moveWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    retval = mobot->moveWait();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_moveToZero_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    retval = mobot->moveToZero();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_moveToZeroNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    retval = mobot->moveToZeroNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_moveJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
	robotJointId_t id;
	double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJoint(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_moveJointNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
	robotJointId_t id;
	double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_moveTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
	double time;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
	time = Ch_VaArg(interp, ap, double);
    retval = mobot->moveTime(time);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_moveTimeNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
	double time;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
	time = Ch_VaArg(interp, ap, double);
    retval = mobot->moveTimeNB(time);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_moveJointTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
	robotJointId_t id;
	double time;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	time = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointTime(id, time);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_moveJointTimeNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
	robotJointId_t id;
	double time;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	time = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointTimeNB(id, time);
    Ch_VaEnd(interp, ap);
    return retval;
}


EXPORTCH int CMGL_reset_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    retval = mobot->reset();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_resetToZero_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    retval = mobot->resetToZero();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_resetToZeroNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    retval = mobot->resetToZeroNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_setExitState_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    robotJointState_t exitState;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    exitState = Ch_VaArg(interp, ap, robotJointState_t);
    retval = mobot->setExitState(exitState);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_setJointMovementStateNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    robotJointId_t id;
    robotJointState_t dir;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t );
    dir = Ch_VaArg(interp, ap, robotJointState_t);
    retval = mobot->setJointMovementStateNB(id, dir);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_setJointMovementStateTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    robotJointId_t id;
    robotJointState_t dir;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    dir = Ch_VaArg(interp, ap, robotJointState_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = mobot->setJointMovementStateTime(id, dir, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_setJointMovementStateTimeNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    robotJointId_t id;
    robotJointState_t dir;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    dir = Ch_VaArg(interp, ap, robotJointState_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = mobot->setJointMovementStateTimeNB(id, dir, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_setJointSafetyAngle_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->setJointSafetyAngle(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_setJointSafetyAngleTimeout_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->setJointSafetyAngleTimeout(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_setJointSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    robotJointId_t id;
    double speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    speed = Ch_VaArg(interp, ap, double);
    retval = mobot->setJointSpeed(id, speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_setJointSpeeds_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    double speed1, speed2, speed3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    speed1 = Ch_VaArg(interp, ap, double);
    speed2 = Ch_VaArg(interp, ap, double);
    speed3 = Ch_VaArg(interp, ap, double);
    retval = mobot->setJointSpeeds(speed1, speed2, speed3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_setJointSpeedRatio_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    robotJointId_t id;
    double speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    speed = Ch_VaArg(interp, ap, double);
    retval = mobot->setJointSpeedRatio(id, speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_setJointSpeedRatios_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    robotJointId_t id;
    double ratio1;
    double ratio2;
    double ratio3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    ratio1 = Ch_VaArg(interp, ap, double);
    ratio2 = Ch_VaArg(interp, ap, double);
    ratio3 = Ch_VaArg(interp, ap, double);
    retval = mobot->setJointSpeedRatios(ratio1, ratio2, ratio3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_setSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    double speed;
    double radius;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    speed = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
    retval = mobot->setSpeed(speed, radius);
    Ch_VaEnd(interp, ap);
    return retval;
}



EXPORTCH int CMGL_setMovementStateNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    robotJointState_t dir1;
    robotJointState_t dir2;
    robotJointState_t dir3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    dir1 = (robotJointState_t)Ch_VaArg(interp, ap, int);
    dir2 = (robotJointState_t)Ch_VaArg(interp, ap, int);
    dir3 = (robotJointState_t)Ch_VaArg(interp, ap, int);
    retval = mobot->setMovementStateNB(dir1, dir2, dir3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_setMovementStateTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    robotJointState_t dir1;
    robotJointState_t dir2;
    robotJointState_t dir3;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    dir1 = Ch_VaArg(interp, ap, robotJointState_t);
    dir2 = Ch_VaArg(interp, ap, robotJointState_t);
    dir3 = Ch_VaArg(interp, ap, robotJointState_t);
    seconds = Ch_VaArg(interp, ap, double );
    retval = mobot->setMovementStateTime(dir1, dir2, dir3, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_setMovementStateTimeNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    robotJointState_t dir1;
    robotJointState_t dir2;
    robotJointState_t dir3;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    dir1 = Ch_VaArg(interp, ap, robotJointState_t);
    dir2 = Ch_VaArg(interp, ap, robotJointState_t);
    dir3 = Ch_VaArg(interp, ap, robotJointState_t);
    seconds = Ch_VaArg(interp, ap, double );
    retval = mobot->setMovementStateTimeNB(dir1, dir2, dir3, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_setTwoWheelRobotSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    double speed;
    double radius;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    speed = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    retval = mobot->setTwoWheelRobotSpeed(speed, radius);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_stopAllJoints_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    retval = mobot->stopAllJoints();
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CMGL_stopOneJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    robotJointId_t id;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    retval = mobot->stopOneJoint(id);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_holdJoints_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    retval = mobot->holdJoints();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_holdJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
	robotJointId_t id;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
    retval = mobot->holdJoint(id);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_relaxJoints_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    retval = mobot->relaxJoints();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_relaxJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
	robotJointId_t id;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
    retval = mobot->relaxJoint(id);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMGL_holdJointsAtExit_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    retval = mobot->holdJointsAtExit();
    Ch_VaEnd(interp, ap);
    return retval;
}

/*Functions for compatibility with RoboSim*/
EXPORTCH int LinkbotL_getxy_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    double *x, *y;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    x = Ch_VaArg(interp, ap, double *);
	y = Ch_VaArg(interp, ap, double *);
    retval = mobot->getxy(*x, *y);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_line_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    double x1, y1, z1;
	double x2, y2, z2;
	int linewidth;
	char *color;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    x1 = Ch_VaArg(interp, ap, double);
	y1 = Ch_VaArg(interp, ap, double);
	z1 = Ch_VaArg(interp, ap, double);
	x2 = Ch_VaArg(interp, ap, double);
	y2 = Ch_VaArg(interp, ap, double);
	z2 = Ch_VaArg(interp, ap, double);
	linewidth = Ch_VaArg(interp, ap, int);
	color = Ch_VaArg(interp, ap, char *);
    retval = mobot->line(x1,y1,z1,x2,y2,z2,linewidth,color);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_movexy_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    double x, y;
	double radius, trackwidth;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    x = Ch_VaArg(interp, ap, double);
	y = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
    retval = mobot->movexy(x, y, radius, trackwidth);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_movexyTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    double x, y;
	double radius, trackwidth;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    x = Ch_VaArg(interp, ap, double);
	y = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
    retval = mobot->movexyTo(x, y, radius, trackwidth);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_movexyToExpr_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    double x0, xf;
	double radius, trackwidth;
	int n;
	char *expr;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    x0 = Ch_VaArg(interp, ap, double);
	xf = Ch_VaArg(interp, ap, double);
	n = Ch_VaArg(interp, ap, int);
	expr = Ch_VaArg(interp, ap, char *);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
    retval = mobot->movexyToExpr(x0, xf, n, expr, radius, trackwidth);
    Ch_VaEnd(interp, ap);
    return retval;
}
typedef double (*ImovexyToFuncHandle)(double);
static ChInterp_t interpI;
static double ImovexyToFunc_chdl_funarg(double x);
static void *ImovexyToFunc_chdl_funptr;
EXPORTCH int LinkbotL_movexyToFunc_chdl(void *varg) {
    ChVaList_t ap;
    class CLinkbotL *robot;
    double x0;
    double xf;
    int n;
    ImovexyToFuncHandle handle_ch, handle_c = NULL;
    double radius;
    double trackwidth;
    int retval;

    Ch_VaStart(interpI, ap, varg);
    robot = Ch_VaArg(interpI, ap, class CLinkbotL *);
    x0 = Ch_VaArg(interpI, ap, double);
    xf = Ch_VaArg(interpI, ap, double);
    n = Ch_VaArg(interpI, ap, int);
    handle_ch = Ch_VaArg(interpI, ap, ImovexyToFuncHandle);
    ImovexyToFunc_chdl_funptr = (void *)handle_ch;
    if (handle_ch != NULL) {
        handle_c = (ImovexyToFuncHandle)ImovexyToFunc_chdl_funarg;
    }    
    radius = Ch_VaArg(interpI, ap, double);
    trackwidth = Ch_VaArg(interpI, ap, double);
    retval = robot->movexyToFunc(x0, xf, n, handle_c, radius, trackwidth);
    Ch_VaEnd(interpI, ap); 
    return retval;
}
static double ImovexyToFunc_chdl_funarg(double x) { 
    double retval;
    Ch_CallFuncByAddr(interpI, ImovexyToFunc_chdl_funptr, &retval, x);
    return retval;
}

EXPORTCH int LinkbotL_movexyNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    double x, y;
	double radius, trackwidth;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    x = Ch_VaArg(interp, ap, double);
	y = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
    retval = mobot->movexyNB(x, y, radius, trackwidth);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_movexyToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    double x, y;
	double radius, trackwidth;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    x = Ch_VaArg(interp, ap, double);
	y = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
    retval = mobot->movexyToNB(x, y, radius, trackwidth);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int LinkbotL_movexyWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = mobot->movexyWait();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_point_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    double x, y, z;
	int pointsize;
	char *color;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    x = Ch_VaArg(interp, ap, double);
	y = Ch_VaArg(interp, ap, double);
	z = Ch_VaArg(interp, ap, double);
	pointsize = Ch_VaArg(interp, ap, int);
	color = Ch_VaArg(interp, ap, char *);
    retval = mobot->point(x, y, z, pointsize, color);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_text_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    double x, y, z;
	char *text;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    x = Ch_VaArg(interp, ap, double);
	y = Ch_VaArg(interp, ap, double);
	z = Ch_VaArg(interp, ap, double);
	text = Ch_VaArg(interp, ap, char *);
    retval = mobot->text(x, y, z, text);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_traceOn_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = mobot->traceOn();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_traceOff_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = mobot->traceOff();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_recordxyBegin_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    double x, y, timeInterval;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    x = Ch_VaArg(interp, ap, double);
	y = Ch_VaArg(interp, ap, double);
	timeInterval = Ch_VaArg(interp, ap, double);
    retval = mobot->recordxyBegin(x, y, timeInterval);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_recordxyEnd_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int *numpoints;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    numpoints = Ch_VaArg(interp, ap, int *);
    retval = mobot->recordxyEnd(*numpoints);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_moveTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    double time;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    time = Ch_VaArg(interp, ap, double);
    retval = mobot->moveTime(time);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int LinkbotL_moveTimeNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    double time;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    time = Ch_VaArg(interp, ap, double);
    retval = mobot->moveTimeNB(time);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_moveJointTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
	robotJointId_t id;
    double time;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
    time = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointTime(id,time);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_moveJointTimeNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
	robotJointId_t id;
    double time;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
    time = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointTimeNB(id,time);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_moveForeverNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = mobot->moveForeverNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_moveJointForeverNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
	robotJointId_t id;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
    retval = mobot->moveJointForeverNB(id);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_holdJoints_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = mobot->holdJoints();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_holdJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
	robotJointId_t id;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
    retval = mobot->holdJoint(id);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_relaxJoints_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = mobot->relaxJoints();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_relaxJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
	robotJointId_t id;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
    retval = mobot->relaxJoint(id);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int LinkbotL_holdJointsAtExit_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = mobot->holdJointsAtExit();
    Ch_VaEnd(interp, ap);
    return retval;
}