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

#ifndef _CH_
class CLinkbot : public CMobot
{
  public:
    int driveToDirect(double angle1, double angle2, double angle3);
    int driveTo(double angle1, double angle2, double angle3);
    int driveToDirectNB(double angle1, double angle2, double angle3);
    int driveToNB(double angle1, double angle2, double angle3);
    int getAccelerometerData(double &accel_x, double &accel_y, double &accel_z);
    int getBatteryVoltage(double &voltage);
    int getBreakoutADC(int adc, int & value);
    int getBreakoutADCVolts(int adc, double & volts);
    int getBreakoutDigitalPin(int pin, int & value);
    int getColorRGB(int &r, int &g, int &b);
	int getColorName(char color[]);		
    int getID();
#ifdef SWIG
    %apply double& OUTPUT {double &angle1, double &angle2, double &angle3};
    void getJointAngles(double &angle1, double &angle2, double &angle3);
#else
    int getJointAngles(double &angle1, double &angle2, double &angle3);
#endif
    int getJointAnglesAverage(double &angle1, double &angle2, double &angle3, int numReadings=10);
    int getJointSpeeds(double &speed1, double &speed2, double &speed3);
    int getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3);
    int move(double angle1, double angle2, double angle3);
    int moveNB(double angle1, double angle2, double angle3);
    int moveContinuousNB(robotJointState_t dir1, 
                       robotJointState_t dir2, 
                       robotJointState_t dir3);
    int moveContinuousTime(robotJointState_t dir1, 
                           robotJointState_t dir2, 
                           robotJointState_t dir3, 
                           double seconds);
    int moveTo(double angle1, double angle2, double angle3);
    int moveToDirect(double angle1, double angle2, double angle3);
    int moveToNB(double angle1, double angle2, double angle3);
    int moveToDirectNB(double angle1, double angle2, double angle3);
    int recordAngles(double time[], 
                     double angle1[], 
                     double angle2[], 
                     double angle3[], 
                     int num, 
                     double seconds,
                     int shiftData = 1);
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
    int setBreakoutAnalogPin(int pin, int value);
    int setBreakoutAnalogRef(int ref);
    int setBreakoutDigitalPin(int pin, int value);
    int setBreakoutPinMode(int pin, int mode);
    int setBuzzerFrequency(int frequency, double time);
    int setBuzzerFrequencyOn(int frequency);
    int setBuzzerFrequencyOff();
    int setColorRGB(int r, int g, int b);
	int setColor(char * color);		
    int setJointSpeeds(double speed1, double speed2, double speed3);
    int setJointSpeedRatios(double ratios1, double ratios2, double ratios3);
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

};
#endif

#ifdef _CH_
class CLinkbotI
{
  public:
    CLinkbotI();
    ~CLinkbotI();
    int accelTimeNB(double radius, double acceleration, double time);
    int accelToVelocityNB(double radius, double acceleration, double velocity);
    int accelToMaxSpeedNB(double radius, double acceleration);
    int accelAngularTimeNB(robotJointId_t id, double acceleration, double time);
    int accelAngularToVelocityNB(robotJointId_t id, double acceleration, double speed);
    int accelAngularAngleNB(robotJointId_t id, double acceleration, double angle);
    int smoothMoveToNB(
        robotJointId_t id,
        double accel0,
        double accelf,
        double vmax,
        double angle);
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
    int connectWithBluetoothAddress(const char address[], ...);
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
    int delaySeconds(int seconds);
    int enableButtonCallback(void (*buttonCallback)(CLinkbotI* mobot, int button, int buttonDown));
    int disableButtonCallback();
    int isConnected();
    int isMoving();
    int getAccelerometerData(double &accel_x, double &accel_y, double &accel_z);
    int getBatteryVoltage(double &voltage);
    int getBreakoutADC(int adc, int & value);
    int getBreakoutADCVolts(int adc, double & volts);
    int getBreakoutDigitalPin(int pin, int & value);
    int getColorRGB(int &r, int &g, int &b);
	int getColorName(char color[]);
	int getColor(string_t & color);		
    int getDistance(double &distance, double radius);
    int getFormFactor(int &formFactor);
    static const char* getConfigFilePath();
    int getID();
    int getJointAngle(robotJointId_t id, double &angle);
    int getJointAngleAverage(robotJointId_t id, double &angle, ... );
    int getJointAngles(double &angle1, double &angle2, double &angle3);
    int getJointAnglesAverage(double &angle1, double &angle2, double &angle3, ...);
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
    int moveWait();
    int moveToZero();
    int moveToZeroNB();
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
    int setBreakoutAnalogPin(int pin, int value);
    int setBreakoutAnalogRef(int ref);
    int setBreakoutDigitalPin(int pin, int value);
    int setBreakoutPinMode(int pin, int mode);
    int setColorRGB(int r, int g, int b);
	int setColor(char * color);		
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
    int stop();
    int stopOneJoint(robotJointId_t id);
    int stopTwoJoints(robotJointId_t id1, robotJointId_t id2);
    int stopThreeJoints(robotJointId_t id1, robotJointId_t id2, robotJointId_t id3);
    int stopAllJoints();
    int turnLeft(double angle, double radius, double tracklength);
    int turnLeftNB(double angle, double radius, double tracklength);
    int turnRight(double angle, double radius, double tracklength);
    int turnRightNB(double angle, double radius, double tracklength);

    int motionDistance(double distance, double radius);
    int motionRollBackward(double angle);
    int motionRollForward(double angle);
    int motionTurnLeft(double angle);
    int motionTurnRight(double angle);

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
  private:
    void* memholder1;
    int memholder2;
};
#else
class CLinkbotI : public CLinkbot
{
  public:
    CLinkbotI();
    ~CLinkbotI();
    int connect();
    int connectWithSerialID(const char serialID[]);
};
#endif

#ifdef _CH_
class CLinkbotL
{
  public:
    CLinkbotL();
    ~CLinkbotL();
    int accelTimeNB(double radius, double acceleration, double time);
    int accelToVelocityNB(double radius, double acceleration, double velocity);
    int accelToMaxSpeedNB(double radius, double acceleration);
    int accelAngularTimeNB(robotJointId_t id, double acceleration, double time);
    int accelAngularToVelocityNB(robotJointId_t id, double acceleration, double speed);
    int accelAngularAngleNB(robotJointId_t id, double acceleration, double angle);
    int smoothMoveToNB(
        robotJointId_t id,
        double accel0,
        double accelf,
        double vmax,
        double angle);
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
    int disconnect();
    int driveJointToDirect(robotJointId_t id, double angle);
    int driveJointTo(robotJointId_t id, double angle);
    int driveJointToDirectNB(robotJointId_t id, double angle);
    int driveJointToNB(robotJointId_t id, double angle);
    int driveToDirect(double angle1, double angle2, double angle3);
    int driveTo(double angle1, double angle2, double angle3);
    int driveToDirectNB(double angle1, double angle2, double angle3);
    int driveToNB(double angle1, double angle2, double angle3);
    int enableButtonCallback(void (*buttonCallback)(CLinkbotL* mobot, int button, int buttonDown));
    int disableButtonCallback();
    int isConnected();
    int isMoving();
    int getAccelerometerData(double &accel_x, double &accel_y, double &accel_z);
    int getBatteryVoltage(double &voltage);
    int getBreakoutADC(int adc, int & value);
    int getBreakoutADCVolts(int adc, double & volts);
    int getBreakoutDigitalPin(int pin, int & value);
    int getFormFactor(int &formFactor);
    static const char* getConfigFilePath();
    int getID();
    int getJointAngle(robotJointId_t id, double &angle);
#ifdef _CH_
    int getJointAngleAverage(robotJointId_t id, double &angle, ... );
#else
    int getJointAngleAverage(robotJointId_t id, double &angle, int numReadings=10);
#endif
    int getJointAngles(double &angle1, double &angle2, double &angle3);
#ifdef _CH_
    int getJointAnglesAverage(double &angle1, double &angle2, double &angle3, ...);
#else
    int getJointAnglesAverage(double &angle1, double &angle2, double &angle3, int numReadings=10);
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
#ifdef _CH_
    int getColor(string_t & color);		
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
    int moveWait();
    int moveToZero();
    int moveToZeroNB();
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
    int recordWait();
    int reset();
    int resetToZero();
    int resetToZeroNB();
    int setBreakoutAnalogPin(int pin, int value);
    int setBreakoutAnalogRef(int ref);
    int setBreakoutDigitalPin(int pin, int value);
    int setBreakoutPinMode(int pin, int mode);
    int setBuzzerFrequency(int frequency, double time);
    int setBuzzerFrequencyOn(int frequency);
    int setBuzzerFrequencyOff();
    int setColorRGB(int r, int g, int b);
	int setColor(char * color);		
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
    int stop();
    int stopOneJoint(robotJointId_t id);
    int stopTwoJoints(robotJointId_t id1, robotJointId_t id2);
    int stopThreeJoints(robotJointId_t id1, robotJointId_t id2, robotJointId_t id3);
    int stopAllJoints();
    int systemTime(double &time);
  private:
    void* memholder1;
    int memholder2;
};
#else
class CLinkbotL : public CLinkbot
{
  public:
    CLinkbotL();
    ~CLinkbotL();
    int connect();
    int connectWithSerialID(const char serialID[]);
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
    int isMoving();
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
    int moveWait();
    int moveToZero();
    int moveToZeroNB();
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
    int stopAllJoints();
    int stopOneJoint(robotJointId_t id);
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
    int move(double angle1, double angle2, double angle3);
    int moveNB(double angle1, double angle2, double angle3);
    int moveContinuousNB(robotJointState_t dir1, 
                       robotJointState_t dir2, 
                       robotJointState_t dir3);
    int moveContinuousTime(robotJointState_t dir1, 
                           robotJointState_t dir2, 
                           robotJointState_t dir3, 
                           double seconds);
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
    int move(double angle1, double angle2, double angle3);
    int moveNB(double angle1, double angle2, double angle3);
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
    int moveWait();
    int moveToZero();
    int moveToZeroNB();
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
    int stopAllJoints();
    int stopOneJoint(robotJointId_t id);

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
