/* Filename: dataAcquisition2.ch
 * Make a graph of the robot's joint angle versus time */

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
array double time[numDataPoints];
array double angles1[numDataPoints];
array double angles2[numDataPoints];
array double angles3[numDataPoints];
array double angles4[numDataPoints];

/* Declare plotting variables */
CPlot plot;
array double angles1_unwrapped[numDataPoints];
array double angles2_unwrapped[numDataPoints];
array double angles3_unwrapped[numDataPoints];
array double angles4_unwrapped[numDataPoints];
double tolerance = 1.0; /* 1 degree for time shifting */

/* Set all joint speeds to 45 degrees/second */
robot.setJointSpeeds(45, 45, 45, 45);

/* Start the motion. First, move robot to zero position */
robot.moveToZero();

/* Start capturing data */
robot.recordAngles(time, angles1, angles2, angles3, angles4, numDataPoints, timeInterval);

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
/* Shift the time so the movement starts at time 0 */
shiftTime(tolerance, numDataPoints, time, 
          angles1_unwrapped, 
          angles2_unwrapped, 
          angles3_unwrapped, 
          angles4_unwrapped);
plot.title("Unwrapped Data for Joint Angles versus Time");
plot.label(PLOT_AXIS_X, "Time (seconds)");
plot.label(PLOT_AXIS_Y, "Angle (degrees)");
plot.data2D(time, angles1_unwrapped);
plot.data2D(time, angles2_unwrapped);
plot.data2D(time, angles3_unwrapped);
plot.data2D(time, angles4_unwrapped);
plot.legend("Joint 1", 0);
plot.legend("Joint 2", 1);
plot.legend("Joint 3", 2);
plot.legend("Joint 4", 3);
plot.grid(PLOT_ON);
plot.plotting();
