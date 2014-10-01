/*
   Copyright 2013 Barobo, Inc.

   This file is part of libbarobo.

   BaroboLink is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   BaroboLink is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with BaroboLink.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _LINKBOT_H_
#define _LINKBOT_H_

#include "mobot.h"

#ifndef NONRELEASE
#ifdef DLLIMPORT
#undef DLLIMPORT
#endif
#define DLLIMPORT
#endif

#ifndef _CH_
class DLLIMPORT CLinkbot : public CMobot
{
  public:
    virtual int driveToDirect(double angle1, double angle2, double angle3);
    virtual int driveTo(double angle1, double angle2, double angle3);
    virtual int driveToDirectNB(double angle1, double angle2, double angle3);
    virtual int driveToNB(double angle1, double angle2, double angle3);
    virtual int enableAccelEventCallback(void *userdata, 
        void (*accelCallback)(int millis, double x, double y, double z, void *userdata));
    virtual int disableAccelEventCallback();
    virtual int getAccelerometerData(double &accel_x, double &accel_y, double &accel_z);
    virtual int getBatteryVoltage(double &voltage);
	virtual int analogRead(int pin);
	virtual double analogReadVolts(int pin);
	virtual int digitalRead(int pin);
    virtual int getColorRGB(int &r, int &g, int &b);
	virtual int getColorName(char color[]);	
	//new
    virtual int getLEDColorRGB(int &r, int &g, int &b);
	virtual int getLEDColorName(char color[]);
    virtual const char* getID();
#ifdef SWIG
    %apply double& OUTPUT {double &angle1, double &angle2, double &angle3};
    void getJointAnglesInstant(double &angle1, double &angle2, double &angle3);
#else
    virtual int getJointAnglesInstant(double &angle1, double &angle2, double &angle3);
#endif
    virtual int getJointAnglesAverage(double &angle1, double &angle2, double &angle3, int numReadings=10);
	virtual int getJointAngles(double &angle1, double &angle2, double &angle3, int numReadings=10);
    virtual int getJointSpeeds(double &speed1, double &speed2, double &speed3);
    virtual int getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3);
    virtual int move(double angle1, double angle2, double angle3);
    virtual int moveNB(double angle1, double angle2, double angle3);
    virtual int moveContinuousNB(robotJointState_t dir1, 
                       robotJointState_t dir2, 
                       robotJointState_t dir3);
    virtual int moveContinuousTime(robotJointState_t dir1, 
                           robotJointState_t dir2, 
                           robotJointState_t dir3, 
                           double seconds);
    virtual int moveTo(double angle1, double angle2, double angle3);
    virtual int moveToDirect(double angle1, double angle2, double angle3);
    virtual int moveToNB(double angle1, double angle2, double angle3);
    virtual int moveToDirectNB(double angle1, double angle2, double angle3);
    virtual int recordAngles(double time[], 
                     double angle1[], 
                     double angle2[], 
                     double angle3[], 
                     int num, 
                     double seconds,
                     int shiftData = 1);
    virtual int recordAnglesBegin(robotRecordData_t &time, 
                          robotRecordData_t &angle1, 
                          robotRecordData_t &angle2, 
                          robotRecordData_t &angle3, 
                          double seconds,
                          int shiftData = 1);
    virtual int recordDistancesBegin(robotRecordData_t &time, 
                          robotRecordData_t &distance1, 
                          robotRecordData_t &distance2, 
                          robotRecordData_t &distance3, 
                          double radius,
                          double seconds,
                          int shiftData = 1);
    virtual int analogWrite(int pin, int value);
    virtual int analogReference(int ref);
    virtual int digitalWrite(int pin, int value);
    virtual int pinMode(int pin, int mode);
    virtual int setBuzzerFrequency(int frequency, double time);
    virtual int setBuzzerFrequencyOn(int frequency);
    virtual int setBuzzerFrequencyOff();
    virtual int setColorRGB(int r, int g, int b);
	virtual int setColor(char * color);
	//new
    virtual int setLEDColorRGB(int r, int g, int b);
	virtual int setLEDColor(char * color);

    virtual int setJointSpeeds(double speed1, double speed2, double speed3);
    virtual int setJointSpeedRatios(double ratios1, double ratios2, double ratios3);
    virtual int setMovementStateNB( robotJointState_t dir1,
        robotJointState_t dir2,
        robotJointState_t dir3);
    virtual int setMovementStateTime( robotJointState_t dir1,
        robotJointState_t dir2,
        robotJointState_t dir3,
        double seconds);
    virtual int setMovementStateTimeNB( robotJointState_t dir1,
        robotJointState_t dir2,
        robotJointState_t dir3,
        double seconds);
	/*Functions for compatibility with RoboSim*/
	virtual int getxy(double &x, double &y);
	virtual int line(double x1, double y1, double z1, double x2, double y2, double z2, 
		     int linewidth, char *color);
	virtual int movexy(double x, double y, double radius, double trackwidth);
	virtual int movexyTo(double x, double y, double radius, double trackwidth);
	virtual int movexyToExpr(double x0, double xf, int n, char *expr, double radius,
		      double trackwidth);
	virtual int movexyToFunc(double x0, double xf, int n, double (*func)(double x), 
		      double radius, double trackwidth);
	virtual int movexyNB(double x, double y, double radius, double trackwidth);
	virtual int movexyToNB(double x, double y, double radius, double trackwidth);
	virtual int movexyWait(void);
	virtual int point(double x, double y, double z, int pointsize, char *color);
	virtual int text(double x, double y, double z, char *text); 
	virtual int traceOn(void);
	virtual int traceOff(void);
	virtual int recordxyBegin(double x, double y, double timeInterval);
	virtual int recordxyEnd(int &numpoints);
	virtual int drivexy(double x, double y, double radius, double trackwidth);
	virtual int drivexyTo(double x, double y, double radius, double trackwidth);
	virtual int drivexyToExpr(double x0, double xf, int n, char *expr, double radius,
		      double trackwidth);
	virtual int drivexyToFunc(double x0, double xf, int n, double (*func)(double x), 
		      double radius, double trackwidth);
	virtual int drivexyNB(double x, double y, double radius, double trackwidth);
	virtual int drivexyToNB(double x, double y, double radius, double trackwidth);
	virtual int jumpTo(double angle1, double angle2, double angle3);
    virtual int jumpToNB(double angle1, double angle2, double angle3);
    virtual int jumpJointTo(robotJointId_t id, double angle);
    virtual int jumpJointToNB(robotJointId_t id, double angle);

};
#endif

#ifdef _CH_
class CLinkbotI
{
  public:
    CLinkbotI();
    ~CLinkbotI();
    int driveAccelTimeNB(double radius, double acceleration, double time);
    int driveAccelToVelocityNB(double radius, double acceleration, double velocity);
    int driveAccelToMaxSpeedNB(double radius, double acceleration);
	int driveAccelDistanceNB(double radius, double acceleration, double distance);
	int driveAccelSmoothNB(double radius, double accel0, double accelf, double vmax, double distance);
    int accelJointTimeNB(robotJointId_t id, double acceleration, double time);
    int accelJointToVelocityNB(robotJointId_t id, double acceleration, double speed);
    int accelJointAngleNB(robotJointId_t id, double acceleration, double angle);
	int accelJointToMaxSpeedNB(robotJointId_t id, double acceleration);
    int accelJointSmoothNB(
        robotJointId_t id,
        double accel0,
        double accelf,
        double vmax,
        double angle);
	/*Cycloidal acceleration profile*/
	int accelJointCycloidalNB(robotJointId_t id, double angle, double time);
	int driveAccelCycloidalNB(double radius, double distance, double time);

	int accelJointHarmonicNB(robotJointId_t id, double angle, double time);
	int driveAccelHarmonicNB(double radius, double distance, double time);

    int blinkLED(double delay, int numBlinks);
/* connect() Return Error Codes:
   -1 : General Error
   -2 : Lockfile Exists
   -3 : Address Format Incorrect
   -4 : Not enough entries in the configuration file
   -5 : Bluetooth device not found
   -6 : Protocol version mismatch
   */
    int connect();
    int connectWithAddress(const char address[], ...);
#ifdef ENABLE_BLUETOOTH
    int connectWithBluetoothAddress(const char address[], ...);
#endif
    int connectWithIPAddress(const char address[], ...);
#ifndef _WIN32
    int connectWithTTY(const char ttyfilename[]);
#endif
    int disconnect();
    int driveJointToDirect(robotJointId_t id, double angle);
    int driveJointTo(robotJointId_t id, double angle);
    int driveJointToDirectNB(robotJointId_t id, double angle);
    int driveJointToNB(robotJointId_t id, double angle);
    int driveToDirect(double angle1, double angle2, double angle3);
    int driveTo(double angle1, double angle2, double angle3);
    int driveToDirectNB(double angle1, double angle2, double angle3);
    int driveToNB(double angle1, double angle2, double angle3);

	/*New car finctions*/
	int driveBackward(double angle);
    int driveBackwardNB(double angle);
	int driveDistance(double distance, double radius);
    int driveDistanceNB(double distance, double radius);
	int driveForeverNB();
	int driveForward(double angle);
    int driveForwardNB(double angle);
	int driveTime(double time);
	int driveTimeNB(double time);
	int drivexy(double x, double y, double radius, double trackwidth);
    int drivexyNB(double x, double y, double radius, double trackwidth);
	int drivexyTo(double x, double y, double radius, double trackwidth);
	int drivexyToNB(double x, double y, double radius, double trackwidth);
	int drivexyToExpr(double x0, double xf, int n, char *expr, double radius,
		      double trackwidth);
	int drivexyToFunc(double x0, double xf, int n, double (*func)(double x), 
		      double radius, double trackwidth);




    int delaySeconds(int seconds);
	//new
	int delay(unsigned int milliseconds);
    int enableButtonCallback(void* userdata, void (*buttonCallback)(void* userdata, int button, int buttonDown));
    int disableButtonCallback();
    int enableRecordDataShift();
    int disableRecordDataShift();
    int isConnected();
    int isMoving();
	int isNotMoving();
	int jumpTo(double angle1, double angle2, double angle3);
    int jumpToNB(double angle1, double angle2, double angle3);
    int jumpJointTo(robotJointId_t id, double angle);
    int jumpJointToNB(robotJointId_t id, double angle);
    int getAccelerometerData(double &accel_x, double &accel_y, double &accel_z);
    int getBatteryVoltage(double &voltage);
    int analogRead(int pin); 
	double analogReadVolts(int pin);
    int digitalRead(int pin);
    int getColorRGB(int &r, int &g, int &b);
	int getColorName(char color[]);
	int getColor(string_t & color);
	int getLEDColorRGB(int &r, int &g, int &b);
	int getLEDColorName(char color[]);
	int getLEDColor(string_t & color);
			
    int getDistance(double &distance, double radius);
    int getFormFactor(int &formFactor);
    static const char* getConfigFilePath();
    int getID();
    int getJointAngle(robotJointId_t id, double &angle, ...);
    int getJointAngleAverage(robotJointId_t id, double &angle, ... );
    int getJointAngles(double &angle1, double &angle2, double &angle3, ...);
    int getJointAnglesAverage(double &angle1, double &angle2, double &angle3, ...);
	int getJointAngleInstant(robotJointId_t id, double &angle);
	int getJointAnglesInstant(double &angle1, double &angle2, double &angle3);
    int getJointMaxSpeed(robotJointId_t id, double &maxSpeed);
    int getJointSafetyAngle(double &angle);
    int getJointSafetyAngleTimeout(double &seconds);
    int getJointSpeed(robotJointId_t id, double &speed);
    int getJointSpeedRatio(robotJointId_t id, double &ratio);
    int getJointSpeeds(double &speed1, double &speed2, double &speed3);
    int getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3);
    int getJointState(robotJointId_t id, robotJointState_t &state);
    mobot_t* getMobotObject();
    int getVersion();
    int move(double angle1, double angle2, double angle3);
    int moveNB(double angle1, double angle2, double angle3);
    int moveBackward(double angle);
    int moveBackwardNB(double angle);
    int moveContinuousNB(robotJointState_t dir1, 
                       robotJointState_t dir2, 
                       robotJointState_t dir3);
    int moveContinuousTime(robotJointState_t dir1, 
                           robotJointState_t dir2, 
                           robotJointState_t dir3, 
                           double seconds);
    int moveDistance(double distance, double radius);
    int moveDistanceNB(double distance, double radius);
    int moveForward(double angle);
    int moveForwardNB(double angle);
	int moveJointByPowerNB(robotJointId_t id, double power);
    int moveJointContinuousNB(robotJointId_t id, robotJointState_t dir);
    int moveJointContinuousTime(robotJointId_t id, robotJointState_t dir, double seconds);
    int moveJoint(robotJointId_t id, double angle);
    int moveJointNB(robotJointId_t id, double angle);
    int moveJointTo(robotJointId_t id, double angle);
    int moveJointToDirect(robotJointId_t id, double angle);
    int moveJointToNB(robotJointId_t id, double angle);
    int moveJointToDirectNB(robotJointId_t id, double angle);
    int moveJointWait(robotJointId_t id);
    int moveTo(double angle1, double angle2, double angle3);
    int moveToDirect(double angle1, double angle2, double angle3);
    int moveToNB(double angle1, double angle2, double angle3);
    int moveToDirectNB(double angle1, double angle2, double angle3);
	/*Functions with the same functionality as jump()*/
	int moveToByTrackPos(double angle1, double angle2, double angle3);
	int moveToByTrackPosNB(double angle1, double angle2, double angle3);
	int moveJointToByTrackPos(robotJointId_t id, double angle);
	int moveJointToByTrackPosNB(robotJointId_t id, double angle);
    /*end*/
    int moveWait();
    int moveToZero();
    int moveToZeroNB();

    int moveForeverNB();
	int moveJointForeverNB(robotJointId_t id);

    int movexy(double x, double y, double radius, double trackwidth);
    int movexyNB(double x, double y, double radius, double trackwidth);
    int recordAngle(robotJointId_t id, double time[:], double angle[:], int num, double seconds, ...);
    int recordAngles(double time[:], 
                     double angle1[:], 
                     double angle2[:], 
                     double angle3[:], 
                     int num, 
                     double seconds,
                     ...);
    int recordAngleBegin(robotJointId_t id, robotRecordData_t &time, robotRecordData_t &angle, double seconds, ...);
    int recordDistanceBegin(robotJointId_t id, robotRecordData_t &time, robotRecordData_t &distance, double radius, double seconds, ...);
    int recordAngleEnd(robotJointId_t id, int &num);
    int recordDistanceEnd(robotJointId_t id, int &num);
    int recordDistanceOffset(double distance);
    int recordAnglesBegin(robotRecordData_t &time, 
                          robotRecordData_t &angle1, 
                          robotRecordData_t &angle2, 
                          robotRecordData_t &angle3, 
                          double seconds,
                          ...);
    int recordDistancesBegin(robotRecordData_t &time, 
                          robotRecordData_t &distance1, 
                          robotRecordData_t &distance2, 
                          robotRecordData_t &distance3, 
                          double radius,
                          double seconds,
                          ...);
    int recordAnglesEnd(int &num);
    int recordDistancesEnd(int &num);
    int recordWait();
    int reset();
    int resetToZero();
    int resetToZeroNB();
    int setBuzzerFrequency(int frequency, double time);
    int setBuzzerFrequencyOn(int frequency);
    int setBuzzerFrequencyOff();
    int analogWrite(int pin, int value);
    int analogReference(int ref);
    int digitalWrite(int pin, int value);
    int pinMode(int pin, int mode);
    int setColorRGB(int r, int g, int b);
	int setColor(char * color);	
	int setLEDColorRGB(int r, int g, int b);
	int setLEDColor(char * color);

    int setExitState(robotJointState_t exitState);
    int setJointMovementStateNB(robotJointId_t id, robotJointState_t dir);
    int setJointMovementStateTime(robotJointId_t id, robotJointState_t dir, double seconds);
    int setJointSafetyAngle(double angle);
    int setJointSafetyAngleTimeout(double seconds);
    int setJointSpeed(robotJointId_t id, double speed);
    int setJointSpeeds(double speed1, double speed2, double speed3);
    int setJointSpeedRatio(robotJointId_t id, double ratio);
    int setJointSpeedRatios(double ratios1, double ratios2, double ratios3);
    int setMotorPower(robotJointId_t id, int power);
    int setJointPower(robotJointId_t id, double power);
    int setMovementStateNB( robotJointState_t dir1,
        robotJointState_t dir2,
        robotJointState_t dir3);
    int setMovementStateTime( robotJointState_t dir1,
        robotJointState_t dir2,
        robotJointState_t dir3,
        double seconds);
    int setMovementStateTimeNB( robotJointState_t dir1,
        robotJointState_t dir2,
        robotJointState_t dir3,
        double seconds);
    int setTwoWheelRobotSpeed(double speed, double radius);
	int setSpeed(double speed, double radius);
    int stop();
    int stopOneJoint(robotJointId_t id);
    int stopTwoJoints(robotJointId_t id1, robotJointId_t id2);
    int stopThreeJoints(robotJointId_t id1, robotJointId_t id2, robotJointId_t id3);
    int stopAllJoints();

	int holdJoint(robotJointId_t id);
	int holdJoints();
	int holdJointsAtExit();
	int relaxJoint(robotJointId_t id);
	int relaxJoints();

    int turnLeft(double angle, double radius, double tracklength);
    int turnLeftNB(double angle, double radius, double tracklength);
    int turnRight(double angle, double radius, double tracklength);
    int turnRightNB(double angle, double radius, double tracklength);

    int motionDistance(double distance, double radius);
    int motionRollBackward(double angle);
    int motionRollForward(double angle);
    int motionTurnLeft(double angle);
    int motionTurnRight(double angle);

	int moveTime(double time);
	int moveTimeNB(double time);
	int moveJointTime(robotJointId_t id, double time);
	int moveJointTimeNB(robotJointId_t id, double time);

    /* Non-Blocking motion functions */
    int motionDistanceNB(double distance, double radius);
    int motionRollBackwardNB(double angle);
    int motionRollForwardNB(double angle);
    int motionTurnLeftNB(double angle);
    int motionTurnRightNB(double angle);
    int motionWait();
    int systemTime(double &time);

    /* Linkbot Only Functions */
    int connectWithSerialID(const char serialID[]);

	/*Functions for compatibility with RoboSim*/
	int getxy(double &x, double &y);
	int line(double x1, double y1, double z1, double x2, double y2, double z2, 
		     int linewidth, char *color);
	int movexy(double x, double y, double radius, double trackwidth);
	int movexyTo(double x, double y, double radius, double trackwidth);
	int movexyToExpr(double x0, double xf, int n, char *expr, double radius,
		      double trackwidth);
	int movexyToFunc(double x0, double xf, int n, double (*func)(double x), 
		      double radius, double trackwidth);
	int movexyNB(double x, double y, double radius, double trackwidth);
	int movexyToNB(double x, double y, double radius, double trackwidth);
	int movexyWait(void);
	int point(double x, double y, double z, int pointsize, char *color);
	int text(double x, double y, double z, char *text); 
	int traceOn(void);
	int traceOff(void);
	int recordxyBegin(double x, double y, double timeInterval);
	int recordxyEnd(int &numpoints);

	/*Gripper functions*/
	int openGripper(double angle);
	int openGripperNB(double angle);
	int closeGripper(void);
	int closeGripperNB(void);

  private:
    void* memholder1;
    int memholder2;
};
#else
class DLLIMPORT CLinkbotI : public CLinkbot
{
  public:
    CLinkbotI();
    ~CLinkbotI();
    virtual int connect();
    virtual int connectWithSerialID(const char serialID[]);
};
#endif

#ifdef _CH_
class CLinkbotL
{
  public:
    CLinkbotL();
    ~CLinkbotL();
    int accelJointTimeNB(robotJointId_t id, double acceleration, double time);
    int accelJointToVelocityNB(robotJointId_t id, double acceleration, double speed);
    int accelJointAngleNB(robotJointId_t id, double acceleration, double angle);
	int accelJointToMaxSpeedNB(robotJointId_t id, double acceleration);
    int accelJointSmoothNB(
        robotJointId_t id,
        double accel0,
        double accelf,
        double vmax,
        double angle);

    int accelJointCycloidalNB(robotJointId_t id, double angle, double time);
	int accelJointHarmonicNB(robotJointId_t id, double angle, double time);

    int blinkLED(double delay, int numBlinks);
/* connect() Return Error Codes:
   -1 : General Error
   -2 : Lockfile Exists
   -3 : Address Format Incorrect
   -4 : Not enough entries in the configuration file
   -5 : Bluetooth device not found
   -6 : Protocol version mismatch
   */
    int connect();
#ifndef _CH_
    int connectWithAddress(const char address[], int channel = 1);
    int connectWithIPAddress(const char address[], const char port[] = "5768");
#else
    int connectWithAddress(const char address[], ...);
    int connectWithIPAddress(const char address[], ...);
#endif
#ifndef _WIN32
    int connectWithTTY(const char ttyfilename[]);
#endif
    int connectWithSerialID(const char serialID[]);
    int delaySeconds(int seconds);
	//new
	int delay(unsigned int milliseconds);
    int disconnect();
    int driveJointToDirect(robotJointId_t id, double angle);
    int driveJointTo(robotJointId_t id, double angle);
    int driveJointToDirectNB(robotJointId_t id, double angle);
    int driveJointToNB(robotJointId_t id, double angle);
    int driveToDirect(double angle1, double angle2, double angle3);
    int driveTo(double angle1, double angle2, double angle3);
    int driveToDirectNB(double angle1, double angle2, double angle3);
    int driveToNB(double angle1, double angle2, double angle3);
    int enableButtonCallback(void* userdata, void (*buttonCallback)(void* userdata, int button, int buttonDown));
    int disableButtonCallback();
    int enableRecordDataShift();
    int disableRecordDataShift();
    int isConnected();
    int isMoving();
	int isNotMoving();
	int jumpTo(double angle1, double angle2, double angle3);
    int jumpToNB(double angle1, double angle2, double angle3);
    int jumpJointTo(robotJointId_t id, double angle);
    int jumpJointToNB(robotJointId_t id, double angle);
    int getAccelerometerData(double &accel_x, double &accel_y, double &accel_z);
    int getBatteryVoltage(double &voltage);
    int analogRead(int pin);
	double analogReadVolts(int pin);
    int digitalRead(int pin); //new
    int getFormFactor(int &formFactor);
    static const char* getConfigFilePath();
    int getID();
	int getJointAngleInstant(robotJointId_t id, double &angle);
	int getJointAnglesInstant(double &angle1, double &angle2, double &angle3);
    
#ifdef _CH_
    int getJointAngleAverage(robotJointId_t id, double &angle, ... );
	int getJointAngle(robotJointId_t id, double &angle, ...);
#else
    int getJointAngleAverage(robotJointId_t id, double &angle, int numReadings=10);
	int getJointAngle(robotJointId_t id, double &angle, int numReadigns=10);
#endif
    
#ifdef _CH_
    int getJointAnglesAverage(double &angle1, double &angle2, double &angle3, ...);
	int getJointAngles(double &angle1, double &angle2, double &angle3, ...);
#else
    int getJointAnglesAverage(double &angle1, double &angle2, double &angle3, int numReadings=10);
	int getJointAngles(double &angle1, double &angle2, double &angle3, int numReadings=10);
#endif
    int getJointMaxSpeed(robotJointId_t id, double &maxSpeed);
    int getJointSafetyAngle(double &angle);
    int getJointSafetyAngleTimeout(double &seconds);
    int getJointSpeed(robotJointId_t id, double &speed);
    int getJointSpeedRatio(robotJointId_t id, double &ratio);
    int getJointSpeeds(double &speed1, double &speed2, double &speed3);
    int getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3);
    int getJointState(robotJointId_t id, robotJointState_t &state);
    mobot_t* getMobotObject();
    int getVersion();
    int getColorRGB(int &r, int &g, int &b);
	int getColorName(char color[]);
	//new
	int getLEDColorRGB(int &r, int &g, int &b);
	int getLEDColorName(char color[]);
#ifdef _CH_
    int getColor(string_t & color);
    int getLEDColor(string_t & color);
#endif
    int move(double angle1, double angle2, double angle3);
    int moveNB(double angle1, double angle2, double angle3);
    int moveContinuousNB(robotJointState_t dir1, 
                       robotJointState_t dir2, 
                       robotJointState_t dir3);
    int moveContinuousTime(robotJointState_t dir1, 
                           robotJointState_t dir2, 
                           robotJointState_t dir3, 
                           double seconds);
	int moveJointByPowerNB(robotJointId_t id, double power);
    int moveJointContinuousNB(robotJointId_t id, robotJointState_t dir);
    int moveJointContinuousTime(robotJointId_t id, robotJointState_t dir, double seconds);
    int moveJoint(robotJointId_t id, double angle);
    int moveJointNB(robotJointId_t id, double angle);
    int moveJointTo(robotJointId_t id, double angle);
    int moveJointToDirect(robotJointId_t id, double angle);
    int moveJointToNB(robotJointId_t id, double angle);
    int moveJointToDirectNB(robotJointId_t id, double angle);
    int moveJointWait(robotJointId_t id);
    int moveTo(double angle1, double angle2, double angle3);
    int moveToDirect(double angle1, double angle2, double angle3);
    int moveToNB(double angle1, double angle2, double angle3);
    int moveToDirectNB(double angle1, double angle2, double angle3);
	int moveToByTrackPos(double angle1, double angle2, double angle3);
	int moveToByTrackPosNB(double angle1, double angle2, double angle3);
	int moveJointToByTrackPos(robotJointId_t id, double angle);
	int moveJointToByTrackPosNB(robotJointId_t id, double angle);
    int moveWait();
    int moveToZero();
    int moveToZeroNB();

	int moveForeverNB();
	int moveJointForeverNB(robotJointId_t id);
#ifdef _CH_
    int recordAngle(robotJointId_t id, double time[:], double angle[:], int num, double seconds, ...);
    int recordAngles(double time[:], 
                     double angle1[:], 
                     double angle2[:], 
                     double angle3[:], 
                     int num, 
                     double seconds,
                     ...);
#else
    int recordAngle(robotJointId_t id, double time[], double angle[], int num, double seconds, int shiftData = 1);
    int recordAngles(double time[], 
                     double angle1[], 
                     double angle2[], 
                     double angle3[], 
                     int num, 
                     double seconds,
                     int shiftData = 1);
#endif
#ifndef _CH_
    int recordAngleBegin(robotJointId_t id, robotRecordData_t &time, robotRecordData_t &angle, double seconds, int shiftData = 1);
    int recordDistanceBegin(robotJointId_t id, robotRecordData_t &time, robotRecordData_t &distance, double radius, double seconds, int shiftData = 1);
#else
    int recordAngleBegin(robotJointId_t id, robotRecordData_t &time, robotRecordData_t &angle, double seconds, ...);
    int recordDistanceBegin(robotJointId_t id, robotRecordData_t &time, robotRecordData_t &distance, double radius, double seconds, ...);
#endif
    int recordAngleEnd(robotJointId_t id, int &num);
    int recordDistanceEnd(robotJointId_t id, int &num);
#ifndef _CH_
    int recordAnglesBegin(robotRecordData_t &time, 
                          robotRecordData_t &angle1, 
                          robotRecordData_t &angle2, 
                          robotRecordData_t &angle3, 
                          double seconds,
                          int shiftData = 1);
    int recordDistancesBegin(robotRecordData_t &time, 
                          robotRecordData_t &distance1, 
                          robotRecordData_t &distance2, 
                          robotRecordData_t &distance3, 
                          double radius,
                          double seconds,
                          int shiftData = 1);
#else
    int recordAnglesBegin(robotRecordData_t &time, 
                          robotRecordData_t &angle1, 
                          robotRecordData_t &angle2, 
                          robotRecordData_t &angle3, 
                          double seconds,
                          ...);
    int recordDistancesBegin(robotRecordData_t &time, 
                          robotRecordData_t &distance1, 
                          robotRecordData_t &distance2, 
                          robotRecordData_t &distance3, 
                          double radius,
                          double seconds,
                          ...);
#endif
    int recordAnglesEnd(int &num);
    int recordDistancesEnd(int &num);
    int recordDistanceOffset(double distance);
    int recordWait();
    int reset();
    int resetToZero();
    int resetToZeroNB();
    int analogWrite(int pin, int value);
    int analogReference(int ref);
    int digitalWrite(int pin, int value);
    int pinMode(int pin, int mode);
    int setBuzzerFrequency(int frequency, double time);
    int setBuzzerFrequencyOn(int frequency);
    int setBuzzerFrequencyOff();
    int setColorRGB(int r, int g, int b);
	int setColor(char * color);	
    //new
	int setLEDColorRGB(int r, int g, int b);
	int setLEDColor(char * color);

    int setExitState(robotJointState_t exitState);
    int setJointMovementStateNB(robotJointId_t id, robotJointState_t dir);
    int setJointMovementStateTime(robotJointId_t id, robotJointState_t dir, double seconds);
    int setJointSafetyAngle(double angle);
    int setJointSafetyAngleTimeout(double seconds);
    int setJointSpeed(robotJointId_t id, double speed);
    int setJointSpeeds(double speed1, double speed2, double speed3);
    int setJointSpeedRatio(robotJointId_t id, double ratio);
    int setJointSpeedRatios(double ratios1, double ratios2, double ratios3);
    int setMotorPower(robotJointId_t id, int power);
    int setJointPower(robotJointId_t id, double power);
    int setMovementStateNB( robotJointState_t dir1,
        robotJointState_t dir2,
        robotJointState_t dir3);
    int setMovementStateTime( robotJointState_t dir1,
        robotJointState_t dir2,
        robotJointState_t dir3,
        double seconds);
    int setMovementStateTimeNB( robotJointState_t dir1,
        robotJointState_t dir2,
        robotJointState_t dir3,
        double seconds);
    int setTwoWheelRobotSpeed(double speed, double radius);
	int setSpeed(double speed, double radius);
    int stop();
    int stopOneJoint(robotJointId_t id);
    int stopTwoJoints(robotJointId_t id1, robotJointId_t id2);
    int stopThreeJoints(robotJointId_t id1, robotJointId_t id2, robotJointId_t id3);
    int stopAllJoints();

    int holdJoint(robotJointId_t id);
	int holdJoints();
	int holdJointsAtExit();
	int relaxJoint(robotJointId_t id);
	int relaxJoints();

    int systemTime(double &time);

	int moveJointTime(robotJointId_t id, double time);
	int moveJointTimeNB(robotJointId_t id, double time);
	int moveTime(double time);
	int moveTimeNB(double time);

    /*Functions for compatibility with RoboSim*/
	int getxy(double &x, double &y);
	int line(double x1, double y1, double z1, double x2, double y2, double z2, 
		     int linewidth, char *color);
	int movexy(double x, double y, double radius, double trackwidth);
	int movexyTo(double x, double y, double radius, double trackwidth);
	int movexyToExpr(double x0, double xf, int n, char *expr, double radius,
		      double trackwidth);
	int movexyToFunc(double x0, double xf, int n, double (*func)(double x), 
		      double radius, double trackwidth);
	int movexyNB(double x, double y, double radius, double trackwidth);
	int movexyToNB(double x, double y, double radius, double trackwidth);
	int movexyWait(void);
	int point(double x, double y, double z, int pointsize, char *color);
	int text(double x, double y, double z, char *text); 
	int traceOn(void);
	int traceOff(void);
	int recordxyBegin(double x, double y, double timeInterval);
	int recordxyEnd(int &numpoints);

	/*Gripper functions*/
	int openGripper(double angle);
	int openGripperNB(double angle);
	int closeGripper(void);
	int closeGripperNB(void);

  private:
    void* memholder1;
    int memholder2;
};
#else
class DLLIMPORT CLinkbotL : public CLinkbot
{
  public:
    CLinkbotL();
    ~CLinkbotL();
    virtual int connect();
    virtual int connectWithSerialID(const char serialID[]);
};
#endif

#ifdef _CH_
class CLinkbotIGroup
{
  public:
    CLinkbotIGroup();
    ~CLinkbotIGroup();
    int addRobot(CLinkbotI& mobot);
    int addRobots(array CLinkbotI mobots[], ...);
    int connect();
    int driveJointToDirect(robotJointId_t id, double angle);
    int driveJointTo(robotJointId_t id, double angle);
    int driveJointToDirectNB(robotJointId_t id, double angle);
    int driveJointToNB(robotJointId_t id, double angle);
    int driveToDirect(double angle1, double angle2, double angle3);
    int driveTo(double angle1, double angle2, double angle3);
    int driveToDirectNB(double angle1, double angle2, double angle3);
    int driveToNB(double angle1, double angle2, double angle3);
	/*new car functions*/
	int driveBackward(double angle);
    int driveBackwardNB(double angle);
	int driveDistance(double distance, double radius);
    int driveDistanceNB(double distance, double radius);
	int driveForward(double angle);
    int driveForwardNB(double angle);
	int driveTime(double time);
	int driveTimeNB(double time);
	int driveForeverNB();

    int isMoving();
	int isNotMoving();
	int jumpTo(double angle1, double angle2, double angle3);
	int jumpToNB(double angle1, double angle2, double angle3);
	int jumpJointTo(robotJointId_t id, double angle);
	int jumpJointToNB(robotJointId_t id, double angle);
    int move(double angle1, double angle2, double angle3);
    int moveNB(double angle1, double angle2, double angle3);
    int moveBackward(double angle);
    int moveBackwardNB(double angle);
    int moveContinuousNB(robotJointState_t dir1, 
                       robotJointState_t dir2, 
                       robotJointState_t dir3);
    int moveContinuousTime(robotJointState_t dir1, 
                           robotJointState_t dir2, 
                           robotJointState_t dir3, 
                           double seconds);
    int moveDistance(double distance, double radius);
    int moveDistanceNB(double distance, double radius);
    int moveForward(double angle);
    int moveForwardNB(double angle);
	int moveJointByPowerNB(robotJointId_t id, double power);
    int moveJointContinuousNB(robotJointId_t id, robotJointState_t dir);
    int moveJointContinuousTime(robotJointId_t id, robotJointState_t dir, double seconds);
    int moveJointTo(robotJointId_t id, double angle);
    int moveJointToDirect(robotJointId_t id, double angle);
    int moveJointToNB(robotJointId_t id, double angle);
    int moveJointToDirectNB(robotJointId_t id, double angle);
    int moveJointWait(robotJointId_t id);
    int moveTo(double angle1, double angle2, double angle3);
    int moveToDirect(double angle1, double angle2, double angle3);
    int moveToNB(double angle1, double angle2, double angle3);
    int moveToDirectNB(double angle1, double angle2, double angle3);
	int moveToByTrackPos(double angle1, double angle2, double angle3);
	int moveToByTrackPosNB(double angle1, double angle2, double angle3);
	int moveJointToByTrackPos(robotJointId_t id, double angle);
	int moveJointToByTrackPosNB(robotJointId_t id, double angle);
    int moveWait();
    int moveToZero();
    int moveToZeroNB();

    int moveForeverNB();
	int moveJointForeverNB(robotJointId_t id);

	int moveJoint(robotJointId_t id, double angle);
	int moveJointNB(robotJointId_t id, double angle);
	int moveTime(double time);
	int moveTimeNB(double time);
	int moveJointTime(robotJointId_t id, double time);
	int moveJointTimeNB(robotJointId_t id, double time);
	int setSpeed(double speed, double radius);

    int reset();
    int resetToZero();
    int resetToZeroNB();
    int setExitState(robotJointState_t exitState);
    int setJointMovementStateNB(robotJointId_t id, robotJointState_t dir);
    int setJointMovementStateTime(robotJointId_t id, robotJointState_t dir, double seconds);
    int setJointMovementStateTimeNB(robotJointId_t id, robotJointState_t dir, double seconds);
    int setJointSafetyAngle(double angle);
    int setJointSafetyAngleTimeout(double angle);
    int setJointSpeed(robotJointId_t id, double speed);
    int setJointSpeeds(double speed1, double speed2, double speed3);
    int setJointSpeedRatio(robotJointId_t id, double ratio);
    int setJointSpeedRatios(double ratio1, double ratio2, double ratio3);
    int setMovementStateNB(robotJointState_t dir1, 
        robotJointState_t dir2, 
        robotJointState_t dir3);
    int setMovementStateTime(robotJointState_t dir1, 
        robotJointState_t dir2, 
        robotJointState_t dir3, 
        double seconds);
    int setMovementStateTimeNB(robotJointState_t dir1, 
        robotJointState_t dir2, 
        robotJointState_t dir3, 
        double seconds);
    int setTwoWheelRobotSpeed(double speed, double radius);
	int setSpeed(double speed, double radius);
    int stopAllJoints();
    int stopOneJoint(robotJointId_t id);

    int holdJoint(robotJointId_t id);
	int holdJoints();
	int holdJointsAtExit();
	int relaxJoint(robotJointId_t id);
	int relaxJoints();

    int turnLeft(double angle, double radius, double tracklength);
    int turnLeftNB(double angle, double radius, double tracklength);
    int turnRight(double angle, double radius, double tracklength);
    int turnRightNB(double angle, double radius, double tracklength);

    int motionDistance(double distance, double radius);
    int motionDistanceNB(double distance, double radius);
    static void* motionDistanceThread(void*);
    int motionRollBackward(double angle);
    int motionRollBackwardNB(double angle);
    static void* motionRollBackwardThread(void*);
    int motionRollForward(double angle);
    int motionRollForwardNB(double angle);
    static void* motionRollForwardThread(void*);
    int motionTurnLeft(double angle);
    int motionTurnLeftNB(double angle);
    static void* motionTurnLeftThread(void*);
    int motionTurnRight(double angle);
    int motionTurnRightNB(double angle);
    static void* motionTurnRightThread(void*);
    int motionWait();

	/*Acceleration functions*/
	int driveAccelTimeNB(double radius, double acceleration, double time);
    int driveAccelToVelocityNB(double radius, double acceleration, double velocity);
    int driveAccelToMaxSpeedNB(double radius, double acceleration);
	int driveAccelDistanceNB(double radius, double acceleration, double distance);
	int driveAccelSmoothNB(double radius, double accel0, double accelf, double vmax, double distance);
	int driveAccelCycloidalNB(double radius, double distance, double time);
	int driveAccelHarmonicNB(double radius, double distance, double time);
    int accelJointTimeNB(robotJointId_t id, double acceleration, double time);
    int accelJointToVelocityNB(robotJointId_t id, double acceleration, double speed);
	int accelJointToMaxSpeedNB(robotJointId_t id, double acceleration);
    int accelJointAngleNB(robotJointId_t id, double acceleration, double angle);
    int accelJointSmoothNB(
        robotJointId_t id,
        double accel0,
        double accelf,
        double vmax,
        double angle);
	int accelJointCycloidalNB(robotJointId_t id, double angle, double time);
	int accelJointHarmonicNB(robotJointId_t id, double angle, double time);

	int openGripper(double angle);
	int openGripperNB(double angle);
	int closeGripper(void);
	int closeGripperNB(void);

    CMobot **_robots2;
    int _numRobots;
    int argInt;
    double argDouble;
    int _numAllocated;
#ifndef _CH_
    THREAD_T* _thread;
#else
    void* _thread;
#endif
    int _motionInProgress;
    CMobot **_robots;
};
#else
class CLinkbotIGroup : public CMobotGroup
{
  public:
    CLinkbotIGroup();
    ~CLinkbotIGroup();
    int addRobot(CLinkbotI& mobot);
    int addRobots(CLinkbotI mobots[], int numMobots);
    int connect();
    int driveToDirect(double angle1, double angle2, double angle3);
    int driveTo(double angle1, double angle2, double angle3);
    int driveToDirectNB(double angle1, double angle2, double angle3);
    int driveToNB(double angle1, double angle2, double angle3);
	int jumpTo(double angle1, double angle2, double angle3);
	int jumpToNB(double angle1, double angle2, double angle3);
	int jumpJointTo(robotJointId_t id, double angle);
	int jumpJointToNB(robotJointId_t id, double angle);
    int move(double angle1, double angle2, double angle3);
    int moveNB(double angle1, double angle2, double angle3);
	/*rename of jump functions*/
	int moveToByTrackPos(double angle1, double angle2, double angle3);
	int moveToByTrackPosNB(double angle1, double angle2, double angle3);
	int moveJointToByTrackPos(robotJointId_t id, double angle);
	int moveJointToByTrackPosNB(robotJointId_t id, double angle);
	int moveJointByPowerNB(robotJointId_t id, double power);
    int moveContinuousNB(robotJointState_t dir1, 
                       robotJointState_t dir2, 
                       robotJointState_t dir3);
    int moveContinuousTime(robotJointState_t dir1, 
                           robotJointState_t dir2, 
                           robotJointState_t dir3, 
                           double seconds);


	int moveForeverNB();
	int driveForeverNB();
	int moveJointForeverNB(robotJointId_t id);

	int holdJoint(robotJointId_t id);
	int holdJoints();
	int holdJointsAtExit();
	int relaxJoint(robotJointId_t id);
	int relaxJoints();

	int moveJoint(robotJointId_t id, double angle);
	int moveJointNB(robotJointId_t id, double angle);
	int moveTime(double time);
	int moveTimeNB(double time);
	int driveTime(double time);
	int driveTimeNB(double time);
	int moveJointTime(robotJointId_t id, double time);
	int moveJointTimeNB(robotJointId_t id, double time);
	int setSpeed(double speed, double radius);

    int moveTo(double angle1, double angle2, double angle3);
    int moveToDirect(double angle1, double angle2, double angle3);
    int moveToNB(double angle1, double angle2, double angle3);
    int moveToDirectNB(double angle1, double angle2, double angle3);
    int setJointSpeeds(double speed1, double speed2, double speed3);
    int setJointSpeedRatios(double ratio1, double ratio2, double ratio3);
    int setMovementStateNB(robotJointState_t dir1, 
        robotJointState_t dir2, 
        robotJointState_t dir3);
    int setMovementStateTime(robotJointState_t dir1, 
        robotJointState_t dir2, 
        robotJointState_t dir3, 
        double seconds);
    int setMovementStateTimeNB(robotJointState_t dir1, 
        robotJointState_t dir2, 
        robotJointState_t dir3, 
        double seconds);
    /* All of these private members should be inherited from CMobotGroup */
  protected:
    CLinkbotI **_robots;
#if 0
  private:
    int _numRobots;
    CLinkbotI *_robots[64];
    int argInt;
    double argDouble;
#ifndef _CH_
    THREAD_T* _thread;
#else
    void* _thread;
#endif
    int _motionInProgress;
#endif
};
#endif

#ifdef _CH_
class CLinkbotLGroup
{
  public:
    CLinkbotLGroup();
    ~CLinkbotLGroup();
    int addRobot(CLinkbotL& mobot);
    int addRobots(array CLinkbotL mobots[], ...);
    int connect();
    int driveJointToDirect(robotJointId_t id, double angle);
    int driveJointTo(robotJointId_t id, double angle);
    int driveJointToDirectNB(robotJointId_t id, double angle);
    int driveJointToNB(robotJointId_t id, double angle);
    int driveToDirect(double angle1, double angle2, double angle3);
    int driveTo(double angle1, double angle2, double angle3);
    int driveToDirectNB(double angle1, double angle2, double angle3);
    int driveToNB(double angle1, double angle2, double angle3);
    int isMoving();
	int isNotMoving();
	int jumpTo(double angle1, double angle2, double angle3);
	int jumpToNB(double angle1, double angle2, double angle3);
	int jumpJointTo(robotJointId_t id, double angle);
	int jumpJointToNB(robotJointId_t id, double angle);
    int move(double angle1, double angle2, double angle3);
    int moveNB(double angle1, double angle2, double angle3);
	int moveJointByPowerNB(robotJointId_t id, double power);
    int moveContinuousNB(robotJointState_t dir1, 
                       robotJointState_t dir2, 
                       robotJointState_t dir3);
    int moveContinuousTime(robotJointState_t dir1, 
                           robotJointState_t dir2, 
                           robotJointState_t dir3, 
                           double seconds);
    int moveJointContinuousNB(robotJointId_t id, robotJointState_t dir);
    int moveJointContinuousTime(robotJointId_t id, robotJointState_t dir, double seconds);
    int moveJointTo(robotJointId_t id, double angle);
    int moveJointToDirect(robotJointId_t id, double angle);
    int moveJointToNB(robotJointId_t id, double angle);
    int moveJointToDirectNB(robotJointId_t id, double angle);
    int moveJointWait(robotJointId_t id);
    int moveTo(double angle1, double angle2, double angle3);
    int moveToDirect(double angle1, double angle2, double angle3);
    int moveToNB(double angle1, double angle2, double angle3);
    int moveToDirectNB(double angle1, double angle2, double angle3);
	int moveToByTrackPos(double angle1, double angle2, double angle3);
	int moveToByTrackPosNB(double angle1, double angle2, double angle3);
	int moveJointToByTrackPos(robotJointId_t id, double angle);
	int moveJointToByTrackPosNB(robotJointId_t id, double angle);
    int moveWait();
    int moveToZero();
    int moveToZeroNB();

	int moveForeverNB();
	int moveJointForeverNB(robotJointId_t id);
	int holdJoint(robotJointId_t id);
	int holdJoints();
	int holdJointsAtExit();
	int relaxJoint(robotJointId_t id);
	int relaxJoints();

	int moveJoint(robotJointId_t id, double angle);
	int moveJointNB(robotJointId_t id, double angle);
	int moveTime(double time);
	int moveTimeNB(double time);
	int moveJointTime(robotJointId_t id, double time);
	int moveJointTimeNB(robotJointId_t id, double time);
	int setSpeed(double speed, double radius);

    int reset();
    int resetToZero();
    int resetToZeroNB();
    int setExitState(robotJointState_t exitState);
    int setJointMovementStateNB(robotJointId_t id, robotJointState_t dir);
    int setJointMovementStateTime(robotJointId_t id, robotJointState_t dir, double seconds);
    int setJointMovementStateTimeNB(robotJointId_t id, robotJointState_t dir, double seconds);
    int setJointSafetyAngle(double angle);
    int setJointSafetyAngleTimeout(double angle);
    int setJointSpeed(robotJointId_t id, double speed);
    int setJointSpeeds(double speed1, double speed2, double speed3);
    int setJointSpeedRatio(robotJointId_t id, double ratio);
    int setJointSpeedRatios(double ratio1, double ratio2, double ratio3);
    int setMovementStateNB(robotJointState_t dir1, 
        robotJointState_t dir2, 
        robotJointState_t dir3);
    int setMovementStateTime(robotJointState_t dir1, 
        robotJointState_t dir2, 
        robotJointState_t dir3, 
        double seconds);
    int setMovementStateTimeNB(robotJointState_t dir1, 
        robotJointState_t dir2, 
        robotJointState_t dir3, 
        double seconds);
    int setTwoWheelRobotSpeed(double speed, double radius);
	int setSpeed(double speed, double radius);
    int stopAllJoints();
    int stopOneJoint(robotJointId_t id);

	/*Acceleration functions*/
    int accelJointTimeNB(robotJointId_t id, double acceleration, double time);
    int accelJointToVelocityNB(robotJointId_t id, double acceleration, double speed);
	int accelJointToMaxSpeedNB(robotJointId_t id, double acceleration);
    int accelJointAngleNB(robotJointId_t id, double acceleration, double angle);
    int accelJointSmoothNB(
        robotJointId_t id,
        double accel0,
        double accelf,
        double vmax,
        double angle);
	int accelJointCycloidalNB(robotJointId_t id, double angle, double time);
	int accelJointHarmonicNB(robotJointId_t id, double angle, double time);


    CMobot **_robots2;
    int _numRobots;
    int argInt;
    double argDouble;
    int _numAllocated;
#ifndef _CH_
    THREAD_T* _thread;
#else
    void* _thread;
#endif
    int _motionInProgress;
    CMobot **_robots;
};
#else
class CLinkbotLGroup : public CLinkbotIGroup
{
  public:
    CLinkbotLGroup();
    ~CLinkbotLGroup();
    int addRobot(CLinkbotL& mobot);
    int addRobots(CLinkbotL robots[], int numRobots);
    int connect();
	int moveForeverNB();
	int moveJointForeverNB(robotJointId_t id);

	int holdJoint(robotJointId_t id);
	int holdJoints();
	int holdJointsAtExit();
	int relaxJoint(robotJointId_t id);
	int relaxJoints();
	int jumpTo(double angle1, double angle2, double angle3);
	int jumpToNB(double angle1, double angle2, double angle3);
	int jumpJointTo(robotJointId_t id, double angle);
	int jumpJointToNB(robotJointId_t id, double angle);

	int moveJoint(robotJointId_t id, double angle);
	int moveJointNB(robotJointId_t id, double angle);
	int moveTime(double time);
	int moveTimeNB(double time);
	int moveJointTime(robotJointId_t id, double time);
	int moveJointTimeNB(robotJointId_t id, double time);
	int setSpeed(double speed, double radius);
};
#endif

class CMelody
{
  public:
    CMelody();
    ~CMelody();
    void setTempo(int bpm);
    void addNote(const char* note, int divider);
  private:
    mobotMelodyNote_t* _head;
};

#ifdef _CH_
#pragma importf "chlinkboti.chf"
#pragma importf "chlinkbotl.chf"
#endif

#endif
