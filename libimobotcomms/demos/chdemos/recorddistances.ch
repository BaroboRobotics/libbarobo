/* File: recorddistances.ch
   Record time and distances, plot the acquired data for two Mobots */
#include <mobot.h>
#include <chplot.h>
int i;
CMobotI mobot1, mobot2;
double speed1=1.5, speed2=3; // speeds of mobots in inches per second
double radius1=1.75, radius2=1.75; // the radii of two wheels of mobot1 and 2
double time=16;              // motion time in seconds for mobot1
double delaytime=8;          // delay time for mobot2 
double timeInterval = 0.1;   // time interval in 0.1 second 
int shiftData = 0;           // flag to disable the shifting of data
int numDataPoints1, numDataPoints2; // number of data points recorded
mobotRecordData_t timedata1, distances1; // recorded time and distances for Mobot1
mobotRecordData_t timedata2, distances2; // recorded time and distances for Mobot2
CPlot plot;                  // plotting class

/* connect to Mobots and move to the zero position at the same time. */
mobot1.connectWithAddress("HFRS", 1);        
mobot2.connectWithAddress("104C", 1);
printf("Resetting to zero...\n");
/*
mobot1.resetToZeroNB();  mobot2.resetToZeroNB();
mobot1.moveWait();       mobot2.moveWait();
*/
printf("Done.\n");

/* set the speeds for mobot1 and mobot2 */
mobot1.setTwoWheelRobotSpeed(speed1, radius1);
mobot2.setTwoWheelRobotSpeed(speed2, radius2);

/* begin recording time and distance */
mobot1.recordDistanceBegin(MOBOT_JOINT1, timedata1, distances1, radius1, timeInterval,
                           shiftData);
mobot2.recordDistanceBegin(MOBOT_JOINT1, timedata2, distances2, radius2, timeInterval, 
                           shiftData);

/* mobot1 moves first, 'delaytime' seconds later, mobot2 moves.
   Both mobots move for 'time-delaytime' seconds at the same time. Then, both mobots stop */
printf("Moving robots...\n");
mobot1.setMovementStateNB(MOBOT_FORWARD, NaN, MOBOT_FORWARD);
delay(delaytime);
printf("Moving robot 2...\n");
mobot2.setMovementStateNB(MOBOT_FORWARD, NaN, MOBOT_FORWARD);
delay(time-delaytime);
mobot1.setMovementStateNB(MOBOT_HOLD, NaN, MOBOT_HOLD);
mobot2.setMovementStateNB(MOBOT_HOLD, NaN, MOBOT_HOLD);
printf("Done.\n");

/* end recording time and distance */
mobot1.recordDistanceEnd(MOBOT_JOINT1, numDataPoints1);
mobot2.recordDistanceEnd(MOBOT_JOINT1, numDataPoints2);

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

for(i = 0; i < numDataPoints1; i++) {
	printf("%lf %lf\n", timedata1[i], distances1[i]);
}
printf("\n\n");
for(i = 0; i < numDataPoints2; i++) {
	printf("%lf %lf\n", timedata2[i], distances2[i]);
}
