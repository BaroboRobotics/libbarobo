/* Filename: dataAcquisition3.ch
 * Make a graph of the mobot's joint angle versus time */

#include <mobot.h>
#include <chplot.h>
CMobot mobot;

/* Connect to the mobot */
mobot.connect();

double speed = 2.5; /* inches / second */
double distance = 12; /* inches */
double radius = 3.5/2.0; /* inches */
double angle = distance2angle(radius, distance); /* degrees */

/* Figure out how many data points we will need. First, figure out the
 * approximate amount of time the movement should take. */
double movementTime = distance / speed; /* Seconds */
/* Add an extra second of recording time to make sure the entire movement is
 * recorded */
movementTime = movementTime + 1; 
double timeInterval = 0.1; /* seconds */
int numDataPoints = movementTime / timeInterval; /* Unitless */

/* Initialize the arrays to be used to store data */
array double time[numDataPoints];
array double angles1[numDataPoints];
array double distances[numDataPoints];

/* Declare plotting variables */
CPlot plot;

/* Start the motion. First, move mobot to zero position */
mobot.resetToZero();
/* Set mobot wheel speed */
mobot.setTwoWheelRobotSpeed(speed, radius);

/* Start capturing data */
mobot.recordAngle(ROBOT_JOINT1, time, angles1, numDataPoints, timeInterval);

/* Roll the mobot the calculated distance */
mobot.motionRollForward(angle);

/* Wait for recording to finish */
mobot.recordWait();

/* Convert angles to displacement */
distances = angle2distance(radius, angles1);

/* Plot the data */
plot.title("Displacement versus Time");
plot.label(PLOT_AXIS_X, "Time (seconds)");
plot.label(PLOT_AXIS_Y, "Displacement (inches)");
plot.data2DCurve(time, distances, numDataPoints);
plot.grid(PLOT_ON);
plot.plotting();
