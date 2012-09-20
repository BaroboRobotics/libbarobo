/* Filename: dataAcquisition.ch
 * Make a graph of the mobot's joint angle versus time */

#include <mobot.h>
#include <chplot.h>
#include <numeric.h>
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

double tolerance = 2.0; /* Degrees */

/* Start the motion. First, move mobot to zero position */
mobot.resetToZero();
/* Set the joint 1 speed to 45 degrees/second */
mobot.setJointSpeed(MOBOT_JOINT1, speed);
mobot.setJointSpeed(MOBOT_JOINT4, speed);

/* Start capturing data */
mobot.recordAngle(MOBOT_JOINT1, time, angles1, numDataPoints, timeInterval, tolerance);

/* Move the joint 720 degrees */
mobot.move(angle, 0, 0, angle);

/* Wait for recording to finish */
mobot.recordWait();

/* Plot the data */
plot1.title("Data for Joint Angle 1 versus Time");
plot1.label(PLOT_AXIS_X, "Time (seconds)");
plot1.label(PLOT_AXIS_Y, "Angle (degrees)");
plot1.data2DCurve(time, angles1, numDataPoints);
plot1.grid(PLOT_ON);
plot1.plotting();

