/* File: recorddistances.ch
   Record time and distances, plot the acquired data for two Mobots */
#include <mobot.h>
#include <chplot.h>
CLinkbotI robot1, robot2;
double speed1=1.5, speed2=3; // speeds of robots in inches per second
double radius1=1.75, radius2=1.75; // the radii of two wheels of robot1 and 2
double time=16;              // motion time in seconds for robot1
double delaytime=8;          // delay time for robot2 
double timeInterval = 0.1;   // time interval in 0.1 second 
int shiftData = 0;           // flag to disable the shifting of data
int numDataPoints1, numDataPoints2; // number of data points recorded
robotRecordData_t timedata1, distances1; // recorded time and distances for Mobot1
robotRecordData_t timedata2, distances2; // recorded time and distances for Mobot2
CPlot plot;                  // plotting class

/* connect to Mobots and move to the zero position at the same time. */
robot1.connect();        
robot2.connect();
robot1.resetToZeroNB();  robot2.resetToZeroNB();
robot1.moveWait();       robot2.moveWait();

/* set the speeds for robot1 and robot2 */
robot1.setTwoWheelRobotSpeed(speed1, radius1);
robot2.setTwoWheelRobotSpeed(speed2, radius2);

/* begin recording time and distance */
robot1.recordDistanceBegin(ROBOT_JOINT1, timedata1, distances1, radius1, timeInterval,
                           shiftData);
robot2.recordDistanceBegin(ROBOT_JOINT1, timedata2, distances2, radius2, timeInterval, 
                           shiftData);

/* robot1 moves first, 'delaytime' seconds later, robot2 moves.
   Both robots move for 'time-delaytime' seconds at the same time. Then, both robots stop */
robot1.setMovementStateNB(ROBOT_FORWARD, NaN, ROBOT_FORWARD);
delay(delaytime);
robot2.setMovementStateNB(ROBOT_FORWARD, NaN, ROBOT_FORWARD);
delay(time-delaytime);
robot1.setMovementStateNB(ROBOT_HOLD, NaN, ROBOT_HOLD);
robot2.setMovementStateNB(ROBOT_HOLD, NaN, ROBOT_HOLD);

/* end recording time and distance */
robot1.recordDistanceEnd(ROBOT_JOINT1, numDataPoints1);
robot2.recordDistanceEnd(ROBOT_JOINT1, numDataPoints2);

/* plot the data */
plot.mathCoord();
plot.title("Distances versus time");
plot.label(PLOT_AXIS_X, "Time (seconds)");
plot.label(PLOT_AXIS_Y, "Distances (inches)");
plot.data2DCurve(timedata1, distances1, numDataPoints1);
plot.legend("Distance for Mobot 1", 0);
plot.data2DCurve(timedata2, distances2, numDataPoints2);
plot.legend("Distance for Mobot 2", 1);
plot.plotting();
