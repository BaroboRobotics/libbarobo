#ifndef _LINKBOT_H_
#define _LINKBOT_H_

#include "mobot.h"

#ifdef _CH_
class CLinkbotI
{
  public:
    CLinkbotI();
    ~CLinkbotI();
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
    int disconnect();
    int driveJointToDirect(mobotJointId_t id, double angle);
    int driveJointTo(mobotJointId_t id, double angle);
    int driveJointToDirectNB(mobotJointId_t id, double angle);
    int driveJointToNB(mobotJointId_t id, double angle);
    int driveToDirect(double angle1, double angle2, double angle3);
    int driveTo(double angle1, double angle2, double angle3);
    int driveToDirectNB(double angle1, double angle2, double angle3);
    int driveToNB(double angle1, double angle2, double angle3);
    int enableButtonCallback(void (*buttonCallback)(CLinkbotI* mobot, int button, int buttonDown));
    int disableButtonCallback();
    int isConnected();
    int isMoving();
    int getAccelerometerData(double &accel_x, double &accel_y, double &accel_z);
    int getBatteryVoltage(double &voltage);
    int getColorRGB(int &r, int &g, int &b);
    int getFormFactor(int &formFactor);
    static const char* getConfigFilePath();
    int getID();
    int getJointAngle(mobotJointId_t id, double &angle);
#ifdef _CH_
    int getJointAngleAverage(mobotJointId_t id, double &angle, ... );
#else
    int getJointAngleAverage(mobotJointId_t id, double &angle, int numReadings=10);
#endif
    int getJointAngles(double &angle1, double &angle2, double &angle3);
#ifdef _CH_
    int getJointAnglesAverage(double &angle1, double &angle2, double &angle3, ...);
#else
    int getJointAnglesAverage(double &angle1, double &angle2, double &angle3, int numReadings=10);
#endif
    int getJointMaxSpeed(mobotJointId_t id, double &maxSpeed);
    int getJointSafetyAngle(double &angle);
    int getJointSafetyAngleTimeout(double &seconds);
    int getJointSpeed(mobotJointId_t id, double &speed);
    int getJointSpeedRatio(mobotJointId_t id, double &ratio);
    int getJointSpeeds(double &speed1, double &speed2, double &speed3);
    int getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3);
    int getJointState(mobotJointId_t id, mobotJointState_t &state);
    mobot_t* getMobotObject();
    int move(double angle1, double angle2, double angle3);
    int moveNB(double angle1, double angle2, double angle3);
    int moveBackward(double angle);
    int moveBackwardNB(double angle);
    int moveContinuousNB(mobotJointState_t dir1, 
                       mobotJointState_t dir2, 
                       mobotJointState_t dir3);
    int moveContinuousTime(mobotJointState_t dir1, 
                           mobotJointState_t dir2, 
                           mobotJointState_t dir3, 
                           double seconds);
    int moveDistance(double distance, double radius);
    int moveDistanceNB(double distance, double radius);
    int moveForward(double angle);
    int moveForwardNB(double angle);
    int moveJointContinuousNB(mobotJointId_t id, mobotJointState_t dir);
    int moveJointContinuousTime(mobotJointId_t id, mobotJointState_t dir, double seconds);
    int moveJoint(mobotJointId_t id, double angle);
    int moveJointNB(mobotJointId_t id, double angle);
    int moveJointTo(mobotJointId_t id, double angle);
    int moveJointToDirect(mobotJointId_t id, double angle);
    int moveJointToNB(mobotJointId_t id, double angle);
    int moveJointToDirectNB(mobotJointId_t id, double angle);
    int moveJointWait(mobotJointId_t id);
    int moveTo(double angle1, double angle2, double angle3);
    int moveToDirect(double angle1, double angle2, double angle3);
    int moveToNB(double angle1, double angle2, double angle3);
    int moveToDirectNB(double angle1, double angle2, double angle3);
    int moveWait();
    int moveToZero();
    int moveToZeroNB();
#ifdef _CH_
    int recordAngle(mobotJointId_t id, double time[:], double angle[:], int num, double seconds, ...);
    int recordAngles(double time[:], 
                     double angle1[:], 
                     double angle2[:], 
                     double angle3[:], 
                     int num, 
                     double seconds,
                     ...);
#else
    int recordAngle(mobotJointId_t id, double time[], double angle[], int num, double seconds, int shiftData = 1);
    int recordAngles(double time[], 
                     double angle1[], 
                     double angle2[], 
                     double angle3[], 
                     int num, 
                     double seconds,
                     int shiftData = 1);
#endif
#ifndef _CH_
    int recordAngleBegin(mobotJointId_t id, robotRecordData_t &time, robotRecordData_t &angle, double seconds, int shiftData = 1);
    int recordDistanceBegin(mobotJointId_t id, robotRecordData_t &time, robotRecordData_t &distance, double radius, double seconds, int shiftData = 1);
#else
    int recordAngleBegin(mobotJointId_t id, robotRecordData_t &time, robotRecordData_t &angle, double seconds, ...);
    int recordDistanceBegin(mobotJointId_t id, robotRecordData_t &time, robotRecordData_t &distance, double radius, double seconds, ...);
#endif
    int recordAngleEnd(mobotJointId_t id, int &num);
    int recordDistanceEnd(mobotJointId_t id, int &num);
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
    int setBuzzerFrequency(int frequency, double time);
    int setBuzzerFrequencyOn(int frequency);
    int setBuzzerFrequencyOff();
    int setColorRGB(int r, int g, int b);
    int setExitState(mobotJointState_t exitState);
    int setJointMovementStateNB(mobotJointId_t id, mobotJointState_t dir);
    int setJointMovementStateTime(mobotJointId_t id, mobotJointState_t dir, double seconds);
    int setJointSafetyAngle(double angle);
    int setJointSafetyAngleTimeout(double seconds);
    int setJointSpeed(mobotJointId_t id, double speed);
    int setJointSpeeds(double speed1, double speed2, double speed3);
    int setJointSpeedRatio(mobotJointId_t id, double ratio);
    int setJointSpeedRatios(double ratios1, double ratios2, double ratios3);
    int setMotorPower(mobotJointId_t id, int power);
    int setMovementStateNB( mobotJointState_t dir1,
        mobotJointState_t dir2,
        mobotJointState_t dir3);
    int setMovementStateTime( mobotJointState_t dir1,
        mobotJointState_t dir2,
        mobotJointState_t dir3,
        double seconds);
    int setMovementStateTimeNB( mobotJointState_t dir1,
        mobotJointState_t dir2,
        mobotJointState_t dir3,
        double seconds);
    int setTwoWheelRobotSpeed(double speed, double radius);
    int stop();
    int stopOneJoint(mobotJointId_t id);
    int stopTwoJoints(mobotJointId_t id1, mobotJointId_t id2);
    int stopThreeJoints(mobotJointId_t id1, mobotJointId_t id2, mobotJointId_t id3);
    int stopAllJoints();
    int turnLeft(double angle);
    int turnLeftNB(double angle);
    int turnRight(double angle);
    int turnRightNB(double angle);

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
#ifndef _CH_
  private:
    mobot_t *_comms;
    void (*buttonCallback)(CMobot *mobot, int button, int buttonDown);
#endif /* Not _CH_*/
};
#else
class CLinkbotI : public CMobot 
{
  public:
    CLinkbotI();
    ~CLinkbotI();
    int connect();
    int connectWithSerialID(const char serialID[]);
    int driveToDirect(double angle1, double angle2, double angle3);
    int driveTo(double angle1, double angle2, double angle3);
    int driveToDirectNB(double angle1, double angle2, double angle3);
    int driveToNB(double angle1, double angle2, double angle3);
    int getAccelerometerData(double &accel_x, double &accel_y, double &accel_z);
    int getBatteryVoltage(double &voltage);
    int getColorRGB(int &r, int &g, int &b);
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
    int moveContinuousNB(mobotJointState_t dir1, 
                       mobotJointState_t dir2, 
                       mobotJointState_t dir3);
    int moveContinuousTime(mobotJointState_t dir1, 
                           mobotJointState_t dir2, 
                           mobotJointState_t dir3, 
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
    int setBuzzerFrequency(int frequency, double time);
    int setBuzzerFrequencyOn(int frequency);
    int setBuzzerFrequencyOff();
    int setColorRGB(int r, int g, int b);
    int setJointSpeeds(double speed1, double speed2, double speed3);
    int setJointSpeedRatios(double ratios1, double ratios2, double ratios3);
    int setMovementStateNB( mobotJointState_t dir1,
        mobotJointState_t dir2,
        mobotJointState_t dir3);
    int setMovementStateTime( mobotJointState_t dir1,
        mobotJointState_t dir2,
        mobotJointState_t dir3,
        double seconds);
    int setMovementStateTimeNB( mobotJointState_t dir1,
        mobotJointState_t dir2,
        mobotJointState_t dir3,
        double seconds);
};
#endif

#ifdef _CH_
class CLinkbotL
{
  public:
    CLinkbotL();
    ~CLinkbotL();
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
    int disconnect();
    int driveJointToDirect(mobotJointId_t id, double angle);
    int driveJointTo(mobotJointId_t id, double angle);
    int driveJointToDirectNB(mobotJointId_t id, double angle);
    int driveJointToNB(mobotJointId_t id, double angle);
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
    int getFormFactor(int &formFactor);
    static const char* getConfigFilePath();
    int getID();
    int getJointAngle(mobotJointId_t id, double &angle);
#ifdef _CH_
    int getJointAngleAverage(mobotJointId_t id, double &angle, ... );
#else
    int getJointAngleAverage(mobotJointId_t id, double &angle, int numReadings=10);
#endif
    int getJointAngles(double &angle1, double &angle2, double &angle3);
#ifdef _CH_
    int getJointAnglesAverage(double &angle1, double &angle2, double &angle3, ...);
#else
    int getJointAnglesAverage(double &angle1, double &angle2, double &angle3, int numReadings=10);
#endif
    int getJointMaxSpeed(mobotJointId_t id, double &maxSpeed);
    int getJointSafetyAngle(double &angle);
    int getJointSafetyAngleTimeout(double &seconds);
    int getJointSpeed(mobotJointId_t id, double &speed);
    int getJointSpeedRatio(mobotJointId_t id, double &ratio);
    int getJointSpeeds(double &speed1, double &speed2, double &speed3);
    int getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3);
    int getJointState(mobotJointId_t id, mobotJointState_t &state);
    mobot_t* getMobotObject();
    int getColorRGB(int &r, int &g, int &b);
    int move(double angle1, double angle2, double angle3);
    int moveNB(double angle1, double angle2, double angle3);
    int moveContinuousNB(mobotJointState_t dir1, 
                       mobotJointState_t dir2, 
                       mobotJointState_t dir3);
    int moveContinuousTime(mobotJointState_t dir1, 
                           mobotJointState_t dir2, 
                           mobotJointState_t dir3, 
                           double seconds);
    int moveJointContinuousNB(mobotJointId_t id, mobotJointState_t dir);
    int moveJointContinuousTime(mobotJointId_t id, mobotJointState_t dir, double seconds);
    int moveJoint(mobotJointId_t id, double angle);
    int moveJointNB(mobotJointId_t id, double angle);
    int moveJointTo(mobotJointId_t id, double angle);
    int moveJointToDirect(mobotJointId_t id, double angle);
    int moveJointToNB(mobotJointId_t id, double angle);
    int moveJointToDirectNB(mobotJointId_t id, double angle);
    int moveJointWait(mobotJointId_t id);
    int moveTo(double angle1, double angle2, double angle3);
    int moveToDirect(double angle1, double angle2, double angle3);
    int moveToNB(double angle1, double angle2, double angle3);
    int moveToDirectNB(double angle1, double angle2, double angle3);
    int moveWait();
    int moveToZero();
    int moveToZeroNB();
#ifdef _CH_
    int recordAngle(mobotJointId_t id, double time[:], double angle[:], int num, double seconds, ...);
    int recordAngles(double time[:], 
                     double angle1[:], 
                     double angle2[:], 
                     double angle3[:], 
                     int num, 
                     double seconds,
                     ...);
#else
    int recordAngle(mobotJointId_t id, double time[], double angle[], int num, double seconds, int shiftData = 1);
    int recordAngles(double time[], 
                     double angle1[], 
                     double angle2[], 
                     double angle3[], 
                     int num, 
                     double seconds,
                     int shiftData = 1);
#endif
#ifndef _CH_
    int recordAngleBegin(mobotJointId_t id, robotRecordData_t &time, robotRecordData_t &angle, double seconds, int shiftData = 1);
    int recordDistanceBegin(mobotJointId_t id, robotRecordData_t &time, robotRecordData_t &distance, double radius, double seconds, int shiftData = 1);
#else
    int recordAngleBegin(mobotJointId_t id, robotRecordData_t &time, robotRecordData_t &angle, double seconds, ...);
    int recordDistanceBegin(mobotJointId_t id, robotRecordData_t &time, robotRecordData_t &distance, double radius, double seconds, ...);
#endif
    int recordAngleEnd(mobotJointId_t id, int &num);
    int recordDistanceEnd(mobotJointId_t id, int &num);
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
    int setBuzzerFrequency(int frequency, double time);
    int setBuzzerFrequencyOn(int frequency);
    int setBuzzerFrequencyOff();
    int setColorRGB(int r, int g, int b);
    int setExitState(mobotJointState_t exitState);
    int setJointMovementStateNB(mobotJointId_t id, mobotJointState_t dir);
    int setJointMovementStateTime(mobotJointId_t id, mobotJointState_t dir, double seconds);
    int setJointSafetyAngle(double angle);
    int setJointSafetyAngleTimeout(double seconds);
    int setJointSpeed(mobotJointId_t id, double speed);
    int setJointSpeeds(double speed1, double speed2, double speed3);
    int setJointSpeedRatio(mobotJointId_t id, double ratio);
    int setJointSpeedRatios(double ratios1, double ratios2, double ratios3);
    int setMotorPower(mobotJointId_t id, int power);
    int setMovementStateNB( mobotJointState_t dir1,
        mobotJointState_t dir2,
        mobotJointState_t dir3);
    int setMovementStateTime( mobotJointState_t dir1,
        mobotJointState_t dir2,
        mobotJointState_t dir3,
        double seconds);
    int setMovementStateTimeNB( mobotJointState_t dir1,
        mobotJointState_t dir2,
        mobotJointState_t dir3,
        double seconds);
    int setTwoWheelRobotSpeed(double speed, double radius);
    int stop();
    int stopOneJoint(mobotJointId_t id);
    int stopTwoJoints(mobotJointId_t id1, mobotJointId_t id2);
    int stopThreeJoints(mobotJointId_t id1, mobotJointId_t id2, mobotJointId_t id3);
    int stopAllJoints();

#ifndef _CH_
  private:
    int getJointDirection(mobotJointId_t id, mobotJointState_t &dir);
    int setJointDirection(mobotJointId_t id, mobotJointState_t dir);
    mobot_t *_comms;
    void (*buttonCallback)(CMobot *mobot, int button, int buttonDown);
#else
#endif /* Not _CH_*/
};
#else
class CLinkbotL : public CMobot
{
  public:
    CLinkbotL();
    ~CLinkbotL();
    int connect();
    int connectWithSerialID(const char serialID[]);
    int driveToDirect(double angle1, double angle2, double angle3);
    int driveTo(double angle1, double angle2, double angle3);
    int driveToDirectNB(double angle1, double angle2, double angle3);
    int driveToNB(double angle1, double angle2, double angle3);
    int getAccelerometerData(double &accel_x, double &accel_y, double &accel_z);
    int getBatteryVoltage(double &voltage);
    int getID();
    int getJointAngles(double &angle1, double &angle2, double &angle3);
    int getJointAnglesAverage(double &angle1, double &angle2, double &angle3, int numReadings=10);
    int getJointSpeeds(double &speed1, double &speed2, double &speed3);
    int getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3);
    int getColorRGB(int &r, int &g, int &b);
    int move(double angle1, double angle2, double angle3);
    int moveNB(double angle1, double angle2, double angle3);
    int moveContinuousNB(mobotJointState_t dir1, 
                       mobotJointState_t dir2, 
                       mobotJointState_t dir3);
    int moveContinuousTime(mobotJointState_t dir1, 
                           mobotJointState_t dir2, 
                           mobotJointState_t dir3, 
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
    int setBuzzerFrequency(int frequency, double time);
    int setBuzzerFrequencyOn(int frequency);
    int setBuzzerFrequencyOff();
    int setColorRGB(int r, int g, int b);
    int setJointSpeeds(double speed1, double speed2, double speed3);
    int setJointSpeedRatios(double ratios1, double ratios2, double ratios3);
    int setMovementStateNB( mobotJointState_t dir1,
        mobotJointState_t dir2,
        mobotJointState_t dir3);
    int setMovementStateTime( mobotJointState_t dir1,
        mobotJointState_t dir2,
        mobotJointState_t dir3,
        double seconds);
    int setMovementStateTimeNB( mobotJointState_t dir1,
        mobotJointState_t dir2,
        mobotJointState_t dir3,
        double seconds);
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
    int driveJointToDirect(mobotJointId_t id, double angle);
    int driveJointTo(mobotJointId_t id, double angle);
    int driveJointToDirectNB(mobotJointId_t id, double angle);
    int driveJointToNB(mobotJointId_t id, double angle);
    int driveToDirect(double angle1, double angle2, double angle3);
    int driveTo(double angle1, double angle2, double angle3);
    int driveToDirectNB(double angle1, double angle2, double angle3);
    int driveToNB(double angle1, double angle2, double angle3);
    int isMoving();
    int move(double angle1, double angle2, double angle3);
    int moveNB(double angle1, double angle2, double angle3);
    int moveBackward(double angle);
    int moveBackwardNB(double angle);
    int moveContinuousNB(mobotJointState_t dir1, 
                       mobotJointState_t dir2, 
                       mobotJointState_t dir3);
    int moveContinuousTime(mobotJointState_t dir1, 
                           mobotJointState_t dir2, 
                           mobotJointState_t dir3, 
                           double seconds);
    int moveDistance(double distance, double radius);
    int moveDistanceNB(double distance, double radius);
    int moveForward(double angle);
    int moveForwardNB(double angle);
    int moveJointContinuousNB(mobotJointId_t id, mobotJointState_t dir);
    int moveJointContinuousTime(mobotJointId_t id, mobotJointState_t dir, double seconds);
    int moveJointTo(mobotJointId_t id, double angle);
    int moveJointToDirect(mobotJointId_t id, double angle);
    int moveJointToNB(mobotJointId_t id, double angle);
    int moveJointToDirectNB(mobotJointId_t id, double angle);
    int moveJointWait(mobotJointId_t id);
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
    int setExitState(mobotJointState_t exitState);
    int setJointMovementStateNB(mobotJointId_t id, mobotJointState_t dir);
    int setJointMovementStateTime(mobotJointId_t id, mobotJointState_t dir, double seconds);
    int setJointMovementStateTimeNB(mobotJointId_t id, mobotJointState_t dir, double seconds);
    int setJointSafetyAngle(double angle);
    int setJointSafetyAngleTimeout(double angle);
    int setJointSpeed(mobotJointId_t id, double speed);
    int setJointSpeeds(double speed1, double speed2, double speed3);
    int setJointSpeedRatio(mobotJointId_t id, double ratio);
    int setJointSpeedRatios(double ratio1, double ratio2, double ratio3);
    int setMovementStateNB(mobotJointState_t dir1, 
        mobotJointState_t dir2, 
        mobotJointState_t dir3);
    int setMovementStateTime(mobotJointState_t dir1, 
        mobotJointState_t dir2, 
        mobotJointState_t dir3, 
        double seconds);
    int setMovementStateTimeNB(mobotJointState_t dir1, 
        mobotJointState_t dir2, 
        mobotJointState_t dir3, 
        double seconds);
    int setTwoWheelRobotSpeed(double speed, double radius);
    int stopAllJoints();
    int stopOneJoint(mobotJointId_t id);
    int turnLeft(double angle);
    int turnLeftNB(double angle);
    int turnRight(double angle);
    int turnRightNB(double angle);

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
    int moveContinuousNB(mobotJointState_t dir1, 
                       mobotJointState_t dir2, 
                       mobotJointState_t dir3);
    int moveContinuousTime(mobotJointState_t dir1, 
                           mobotJointState_t dir2, 
                           mobotJointState_t dir3, 
                           double seconds);
    int moveTo(double angle1, double angle2, double angle3);
    int moveToDirect(double angle1, double angle2, double angle3);
    int moveToNB(double angle1, double angle2, double angle3);
    int moveToDirectNB(double angle1, double angle2, double angle3);
    int setJointSpeeds(double speed1, double speed2, double speed3);
    int setJointSpeedRatios(double ratio1, double ratio2, double ratio3);
    int setMovementStateNB(mobotJointState_t dir1, 
        mobotJointState_t dir2, 
        mobotJointState_t dir3);
    int setMovementStateTime(mobotJointState_t dir1, 
        mobotJointState_t dir2, 
        mobotJointState_t dir3, 
        double seconds);
    int setMovementStateTimeNB(mobotJointState_t dir1, 
        mobotJointState_t dir2, 
        mobotJointState_t dir3, 
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
    int driveJointToDirect(mobotJointId_t id, double angle);
    int driveJointTo(mobotJointId_t id, double angle);
    int driveJointToDirectNB(mobotJointId_t id, double angle);
    int driveJointToNB(mobotJointId_t id, double angle);
    int driveToDirect(double angle1, double angle2, double angle3);
    int driveTo(double angle1, double angle2, double angle3);
    int driveToDirectNB(double angle1, double angle2, double angle3);
    int driveToNB(double angle1, double angle2, double angle3);
    int isMoving();
    int move(double angle1, double angle2, double angle3);
    int moveNB(double angle1, double angle2, double angle3);
    int moveContinuousNB(mobotJointState_t dir1, 
                       mobotJointState_t dir2, 
                       mobotJointState_t dir3);
    int moveContinuousTime(mobotJointState_t dir1, 
                           mobotJointState_t dir2, 
                           mobotJointState_t dir3, 
                           double seconds);
    int moveJointContinuousNB(mobotJointId_t id, mobotJointState_t dir);
    int moveJointContinuousTime(mobotJointId_t id, mobotJointState_t dir, double seconds);
    int moveJointTo(mobotJointId_t id, double angle);
    int moveJointToDirect(mobotJointId_t id, double angle);
    int moveJointToNB(mobotJointId_t id, double angle);
    int moveJointToDirectNB(mobotJointId_t id, double angle);
    int moveJointWait(mobotJointId_t id);
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
    int setExitState(mobotJointState_t exitState);
    int setJointMovementStateNB(mobotJointId_t id, mobotJointState_t dir);
    int setJointMovementStateTime(mobotJointId_t id, mobotJointState_t dir, double seconds);
    int setJointMovementStateTimeNB(mobotJointId_t id, mobotJointState_t dir, double seconds);
    int setJointSafetyAngle(double angle);
    int setJointSafetyAngleTimeout(double angle);
    int setJointSpeed(mobotJointId_t id, double speed);
    int setJointSpeeds(double speed1, double speed2, double speed3);
    int setJointSpeedRatio(mobotJointId_t id, double ratio);
    int setJointSpeedRatios(double ratio1, double ratio2, double ratio3);
    int setMovementStateNB(mobotJointState_t dir1, 
        mobotJointState_t dir2, 
        mobotJointState_t dir3);
    int setMovementStateTime(mobotJointState_t dir1, 
        mobotJointState_t dir2, 
        mobotJointState_t dir3, 
        double seconds);
    int setMovementStateTimeNB(mobotJointState_t dir1, 
        mobotJointState_t dir2, 
        mobotJointState_t dir3, 
        double seconds);
    int setTwoWheelRobotSpeed(double speed, double radius);
    int stopAllJoints();
    int stopOneJoint(mobotJointId_t id);

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

#endif
