/* Filename: dataAcquisition2.ch
 * Make a graph of the robot's joint angle versus time1 */

#include <mobot.h>
#include <chplot.h>
#include <numeric.h>
CMobot robot;

/* Connect to the robot */
robot.connect();

double timeInterval = 0.1; /* Seconds */

/* Record for 20 seconds */
double movementTime = 20;
int numDataPoints = movementTime / timeInterval; /* Unitless */

/* Initialize the arrays to be used to store data */
array double time1[numDataPoints];
array double angles1[numDataPoints];
array double angles2[numDataPoints];
array double angles3[numDataPoints];
array double angles4[numDataPoints];

/* Declare plotting variables */
CPlot plot1, plot2;
array double angles1_unwrapped[numDataPoints];
array double angles2_unwrapped[numDataPoints];
array double angles3_unwrapped[numDataPoints];
array double angles4_unwrapped[numDataPoints];

/* Start the motion. First, move robot to zero position */
robot.moveToZero();

/* Start capturing data */
robot.recordAngles(time1, angles1, angles2, angles3, angles4, numDataPoints, timeInterval);

/* Perform the standing and unstanding motions */
robot.motionTurnRight(360);
robot.motionInchwormLeft(2);

/* Wait for recording to finish */
robot.recordWait();

/* Plot the unwrapped data */
unwrapdeg(angles1_unwrapped, angles1);
unwrapdeg(angles2_unwrapped, angles2);
unwrapdeg(angles3_unwrapped, angles3);
unwrapdeg(angles4_unwrapped, angles4);
plot2.title("Unwrapped Data for Joint Angles versus Time");
plot2.label(PLOT_AXIS_X, "Time (seconds)");
plot2.label(PLOT_AXIS_Y, "Angle (degrees)");
plot2.data2D(time1, angles1_unwrapped);
plot2.data2D(time1, angles2_unwrapped);
plot2.data2D(time1, angles3_unwrapped);
plot2.data2D(time1, angles4_unwrapped);
plot2.legend("Joint 1", 0);
plot2.legend("Joint 2", 1);
plot2.legend("Joint 3", 2);
plot2.legend("Joint 4", 3);
plot2.grid(PLOT_ON);
plot2.plotting();
