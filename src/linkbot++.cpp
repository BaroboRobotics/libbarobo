#include "linkbot.h"
#include <stdlib.h>

#define DEPRECATED(from, to) \
  fprintf(stderr, "Warning: The function \"%s()\" is deprecated. Please use \"%s()\"\n" , from, to)

int CLinkbot::enableAccelEventCallback(void *userdata, 
    void(*accelCallback)(int millis, double x, double y, double z, void* userdata))
{
  return Mobot_enableAccelEventCallback(
      _comms,
      userdata,
      accelCallback);
}

int CLinkbot::disableAccelEventCallback()
{
  return Mobot_disableAccelEventCallback(_comms);
}

const char* CLinkbot::getID()
{
  return _comms->serialID;
}

int CLinkbot::getAccelerometerData(double &accel_x, double &accel_y, double &accel_z)
{
  double _x, _y, _z; int rc;
  rc = Mobot_getAccelerometerData(_comms, &_x, &_y, &_z);
  if(rc) {
    return rc;
  }
  accel_x = _x;
  accel_y = _y;
  accel_z = _z;
  return 0;
}

int CLinkbot::getBatteryVoltage(double &voltage)
{
  return Mobot_getBatteryVoltage(_comms, &voltage);
}

/*int CLinkbot::LinkPodAnalogRead(int adc, int &value)
{
  return Mobot_LinkPodAnalogRead(_comms, adc, &value);
}*/

int CLinkbot::LinkPodAnalogRead(int pin)
{
    int value;
	Mobot_LinkPodAnalogRead(_comms, pin, &value);
	return value;
}

/*int CLinkbot::LinkPodAnalogReadVolts(int adc, double & volts)
{
  return Mobot_LinkPodAnalogReadVolts(_comms, adc, &volts);
}*/

double CLinkbot::LinkPodAnalogReadVolts(int pin)
{
    double volts;
	Mobot_LinkPodAnalogReadVolts(_comms, pin, &volts);
	return volts;
}

/*int CLinkbot::LinkPodDigitalRead(int pin, int & value)
{
  return Mobot_LinkPodDigitalRead(_comms, pin, &value);
}*/

int CLinkbot::LinkPodDigitalRead(int pin)
{
    int value;
	Mobot_LinkPodDigitalRead(_comms, pin, &value);
	return value;
}

int CLinkbot::getJointAngles(
    double &angle1,
    double &angle2,
    double &angle3)
{
  double time;
  double angle4;
  int err;
  err = Mobot_getJointAnglesTime(
      _comms, 
      &time,
      &angle1,
      &angle2,
      &angle3,
      &angle4);
  if(err) return err;
  angle1 = RAD2DEG(angle1);
  angle2 = RAD2DEG(angle2);
  angle3 = RAD2DEG(angle3);
  angle4 = RAD2DEG(angle4);
  return 0;
}

int CLinkbot::getJointAnglesAverage(
    double &angle1,
    double &angle2,
    double &angle3,
    int numReadings)
{
  int err;
  double angle4;
  err = Mobot_getJointAnglesAverage(
      _comms, 
      &angle1,
      &angle2,
      &angle3,
      &angle4,
      numReadings);
  if(err) return err;
  angle1 = RAD2DEG(angle1);
  angle2 = RAD2DEG(angle2);
  angle3 = RAD2DEG(angle3);
  angle4 = RAD2DEG(angle4);
  return 0;
}

int CLinkbot::getJointSpeeds(double &speed1, double &speed2, double &speed3)
{
  int i;
  double speed4;
  int err = Mobot_getJointSpeeds(_comms, &speed1, &speed2, &speed3, &speed4);
  speed1 = RAD2DEG(speed1);
  speed2 = RAD2DEG(speed2);
  speed3 = RAD2DEG(speed3);
  speed4 = RAD2DEG(speed4);
  return err;
}

int CLinkbot::getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3)
{
  double ratio4;
  return Mobot_getJointSpeedRatios(_comms, &ratio1, &ratio2, &ratio3, &ratio4);
}

int CLinkbot::getColorRGB(int &r, int &g, int &b)
{
  return Mobot_getColorRGB(_comms, &r, &g, &b);
}

int CLinkbot::getColorName(char color[]) //C++ compatible version of getColor()
{
  return Mobot_getColor(_comms, color);
}

//new functions start
int CLinkbot::getLEDColorRGB(int &r, int &g, int &b)
{
  return Mobot_getColorRGB(_comms, &r, &g, &b);
}


int CLinkbot::getLEDColorName(char color[]) //C++ compatible version of getColor()
{
  return Mobot_getColor(_comms, color);
}
//new functions end

int CLinkbot::driveToDirect( double angle1,
                          double angle2,
                          double angle3)
{
  return Mobot_driveToDirect(
      _comms, 
      DEG2RAD(angle1), 
      DEG2RAD(angle2), 
      DEG2RAD(angle3), 
      DEG2RAD(0));
}

int CLinkbot::driveTo( double angle1,
                          double angle2,
                          double angle3)
{
  return Mobot_driveToDirect(
      _comms, 
      DEG2RAD(angle1), 
      DEG2RAD(angle2), 
      DEG2RAD(angle3), 
      DEG2RAD(0));
}

int CLinkbot::driveToDirectNB( double angle1,
                          double angle2,
                          double angle3)
{
  return Mobot_driveToDirectNB(
      _comms, 
      DEG2RAD(angle1), 
      DEG2RAD(angle2), 
      DEG2RAD(angle3), 
      DEG2RAD(0));
}

int CLinkbot::driveToNB( double angle1,
                          double angle2,
                          double angle3)
{
  return Mobot_driveToDirectNB(
      _comms, 
      DEG2RAD(angle1), 
      DEG2RAD(angle2), 
      DEG2RAD(angle3), 
      DEG2RAD(0));
}

int CLinkbot::move( double angle1,
                        double angle2,
                        double angle3)
{
  return Mobot_move(
      _comms, 
      DEG2RAD(angle1), 
      DEG2RAD(angle2), 
      DEG2RAD(angle3), 
      DEG2RAD(0));
}

int CLinkbot::moveNB( double angle1,
                        double angle2,
                        double angle3)
{
  return Mobot_moveNB(
      _comms, 
      DEG2RAD(angle1), 
      DEG2RAD(angle2), 
      DEG2RAD(angle3), 
      DEG2RAD(0));
}

int CLinkbot::moveContinuousNB( robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3)
{
  DEPRECATED("moveContinuousNB", "setMovementStateNB");
  return Mobot_moveContinuousNB(_comms, dir1, dir2, dir3, ROBOT_NEUTRAL);
}

int CLinkbot::moveContinuousTime( robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, double seconds)
{
  DEPRECATED("moveContinuousTime", "setMovementStateTime");
  return Mobot_moveContinuousTime(_comms, dir1, dir2, dir3, ROBOT_NEUTRAL, seconds);
}

int CLinkbot::moveTo( double angle1,
                          double angle2,
                          double angle3)
{
  return Mobot_moveTo(
      _comms, 
      DEG2RAD(angle1), 
      DEG2RAD(angle2), 
      DEG2RAD(angle3), 
      DEG2RAD(0));
}

int CLinkbot::moveToDirect( double angle1,
                          double angle2,
                          double angle3)
{
  return Mobot_moveToDirect(
      _comms, 
      DEG2RAD(angle1), 
      DEG2RAD(angle2), 
      DEG2RAD(angle3), 
      DEG2RAD(0));
}

int CLinkbot::moveToNB( double angle1,
                          double angle2,
                          double angle3)
{
  return Mobot_moveToNB(
      _comms, 
      DEG2RAD(angle1), 
      DEG2RAD(angle2), 
      DEG2RAD(angle3), 
      DEG2RAD(0));
}

int CLinkbot::moveToDirectNB( double angle1,
                          double angle2,
                          double angle3)
{
  return Mobot_moveToDirectNB(
      _comms, 
      DEG2RAD(angle1), 
      DEG2RAD(angle2), 
      DEG2RAD(angle3), 
      DEG2RAD(0));
}

int CLinkbot::recordAngles(double *time, 
    double *angle1, 
    double *angle2, 
    double *angle3, 
    int num, 
    double seconds,
    int shiftData)
{
  int rc;
  double *angle4;
  angle4 = (double*)malloc(sizeof(double)*num);
  rc = Mobot_recordAngles(_comms, time, angle1, angle2, angle3, angle4, num, seconds, shiftData);
  free(angle4);
  return rc;
}

int CLinkbot::recordAnglesBegin(double* &time, 
    double* &angle1, 
    double* &angle2, 
    double* &angle3, 
    double seconds,
    int shiftData)
{
  double *angle4;
  return Mobot_recordAnglesBegin(_comms, &time, &angle1, &angle2, &angle3, &angle4, seconds, shiftData);
}

int CLinkbot::recordDistancesBegin(
    double* &time,
    double* &distance1,
    double* &distance2,
    double* &distance3,
    double radius, 
    double timeInterval,
    int shiftData)
{
  double* distance4;
  return Mobot_recordDistancesBegin(_comms,
    &time,
    &distance1,
    &distance2,
    &distance3,
    &distance4,
    radius, 
    timeInterval,
    shiftData);
}

int CLinkbot::LinkPodAnalogWrite(int pin, int value)
{
  return Mobot_LinkPodAnalogWrite(_comms, pin, value);
}

int CLinkbot::LinkPodAnalogReference(int ref)
{
  return Mobot_LinkPodAnalogReference(_comms, ref);
}

int CLinkbot::LinkPodDigitalWrite(int pin, int value)
{
  return Mobot_LinkPodDigitalWrite(_comms, pin, value);
}

int CLinkbot::LinkPodPinMode(int pin, int mode)
{
  return Mobot_LinkPodPinMode(_comms, pin, mode);
}

int CLinkbot::setBuzzerFrequency(int frequency, double time)
{
  return Mobot_setBuzzerFrequency(_comms, frequency, time);
}

int CLinkbot::setBuzzerFrequencyOn(int frequency)
{
  return Mobot_setBuzzerFrequencyOn(_comms, frequency);
}

int CLinkbot::setBuzzerFrequencyOff()
{
  return Mobot_setBuzzerFrequencyOff(_comms);
}

int CLinkbot::setColorRGB(int r, int g, int b)
{
  return Mobot_setColorRGB(_comms, r, g, b);
}

int CLinkbot::setColor(char* color)
{
  return Mobot_setColor(_comms, color);
}

//new begin
int CLinkbot::setLEDColorRGB(int r, int g, int b)
{
  return Mobot_setColorRGB(_comms, r, g, b);
}

int CLinkbot::setLEDColor(char* color)
{
  return Mobot_setColor(_comms, color);
}
//new end


int CLinkbot::setJointSpeeds(double speed1, double speed2, double speed3)
{
  return Mobot_setJointSpeeds(
      _comms, 
      DEG2RAD(speed1), 
      DEG2RAD(speed2), 
      DEG2RAD(speed3), 
      DEG2RAD(0));
}

int CLinkbot::setJointSpeedRatios(double ratio1, double ratio2, double ratio3)
{
  return Mobot_setJointSpeedRatios(_comms, ratio1, ratio2, ratio3, 0);
}

int CLinkbot::setMovementStateNB( robotJointState_t dir1,
                                robotJointState_t dir2,
                                robotJointState_t dir3)
{
  return Mobot_setMovementStateNB(_comms, dir1, dir2, dir3, ROBOT_NEUTRAL);
}

int CLinkbot::setMovementStateTime( robotJointState_t dir1,
                                  robotJointState_t dir2,
                                  robotJointState_t dir3,
                                  double seconds)
{
  return Mobot_setMovementStateTime(_comms, dir1, dir2, dir3, ROBOT_NEUTRAL, seconds);
}

int CLinkbot::setMovementStateTimeNB( robotJointState_t dir1,
                                  robotJointState_t dir2,
                                  robotJointState_t dir3,
                                  double seconds)
{
  return Mobot_setMovementStateTimeNB(_comms, dir1, dir2, dir3, ROBOT_NEUTRAL, seconds);
}
