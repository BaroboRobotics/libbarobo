/* Filename: dataAcquisition1.ch
 * Make a graph of the robot's joint angle versus time */

#include <mobot.h>
#include <chplot.h>
#include <numeric.h>
CMobot robot;

/* Connect to the robot */
robot.connect();

double speed = 45; /* Degrees/second */
double angle = 720; /* Degrees */
double timeInterval = 0.1; /* Seconds */

/* Figure out how many data points we will need. First, figure out the
 * approximate amount of time the movement should take. */
double movementTime = angle / speed; /* Seconds */
/* Add an extra second of recording time to make sure the entire movement is
 * recorded */
movementTime = movementTime + 1; 
int numDataPoints = movementTime / timeInterval; /* Unitless */

/* Initialize the arrays to be used to store data for time and angle */
array double time[numDataPoints];
array double angles1[numDataPoints];

/* Declare plotting variables */
CPlot plot;
array double angles1Unwrapped[numDataPoints];

/* Declare time shifted data */
double tolerance = 1.0; /* Degrees */

/* Start the motion. First, move robot to zero position */
robot.moveToZero();
/* Set the joint 1 speed to 45 degrees/second */
robot.setJointSpeed(MOBOT_JOINT1, speed);
robot.setJointSpeed(MOBOT_JOINT4, speed);

/* Start capturing data */
robot.recordAngle(MOBOT_JOINT1, time, angles1, numDataPoints, timeInterval);

/* Move the joint 720 degrees */
robot.move(angle, 0, 0, angle);

/* Wait for recording to finish */
robot.recordWait();

/* Plot the unwrapped data */
unwrapdeg(angles1Unwrapped, angles1);

/* Adjust the time delay */
/* Shift the time so the movement starts at time 0 */
shiftTime(tolerance, numDataPoints, time, angles1Unwrapped);

/* Plot the data */
plot.title("Unwrapped and shifted Data for Joint Angle 1 versus Time");
plot.label(PLOT_AXIS_X, "Time (seconds)");
plot.label(PLOT_AXIS_Y, "Angle (degrees)");
plot.data2D(time, angles1Unwrapped);
plot.grid(PLOT_ON);
plot.plotting();

