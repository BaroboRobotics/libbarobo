/* Filename: dataAcquisition.ch
 * Make a graph of the mobot's joint angle versus time */

#include <mobot.h>
#include <chplot.h>
CMobot mobot;

/* Connect to the mobot */
mobot.connect();

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
CPlot plot1, plot2, plot3;

/* Declare time shifted data */
double tolerance = 1.0; /* Degrees */

/* Start the motion. First, move mobot to zero position */
mobot.moveToZero();
/* Set the joint 1 speed to 45 degrees/second */
mobot.setJointSpeed(MOBOT_JOINT1, speed);
mobot.setJointSpeed(MOBOT_JOINT4, speed);

/* Start capturing data */
mobot.recordAngle(MOBOT_JOINT1, time, angles1, numDataPoints, timeInterval);

/* Move the joint 720 degrees */
mobot.move(angle, 0, 0, angle);

/* Wait for recording to finish */
mobot.recordWait();

/* Plot the data */
plot1.title("Original Data for Joint Angle 1 versus Time");
plot1.label(PLOT_AXIS_X, "Time (seconds)");
plot1.label(PLOT_AXIS_Y, "Angle (degrees)");
plot1.data2DCurve(time, angles1, numDataPoints);
plot1.grid(PLOT_ON);
plot1.plotting();

/* Plot the data */
plot2.title("Data for Joint Angle 1 versus Time");
plot2.label(PLOT_AXIS_X, "Time (seconds)");
plot2.label(PLOT_AXIS_Y, "Angle (degrees)");
plot2.data2DCurve(time, angles1, numDataPoints);
plot2.grid(PLOT_ON);
plot2.plotting();

/* Adjust the time delay */
/* Shift the time so the movement starts at time 0 */
shiftTime(tolerance, numDataPoints, time, angles1);

/* Plot the data */
plot3.title("Shifted Data for Joint Angle 1 versus Time");
plot3.label(PLOT_AXIS_X, "Time (seconds)");
plot3.label(PLOT_AXIS_Y, "Angle (degrees)");
plot3.data2DCurve(time, angles1, numDataPoints);
plot3.grid(PLOT_ON);
plot3.plotting();

