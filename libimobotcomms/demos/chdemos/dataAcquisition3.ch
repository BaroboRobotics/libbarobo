/* Filename: dataAcquisition3.ch
 * Make a graph of the robot's joint angle versus time */

#include <mobot.h>
#include <chplot.h>
#include <numeric.h>
CMobot robot;

/* Connect to the robot */
robot.connect();

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
array double angles1_unwrapped[numDataPoints];

/* Start the motion. First, move robot to zero position */
robot.moveToZero();
/* Set robot wheel speed */
robot.setTwoWheelRobotSpeed(speed, radius);

/* Start capturing data */
robot.recordAngle(ROBOT_JOINT1, time, angles1, numDataPoints, timeInterval);

/* Roll the robot the calculated distance */
robot.motionRollForward(angle);

/* Wait for recording to finish */
robot.recordWait();

/* Unwrap the data */
unwrapdeg(angles1_unwrapped, angles1);
/* Shift the data */
double startTime;
int i;
for(i = 0; i < numDataPoints; i++) {
  if(abs(angles1_unwrapped[i]) > 1) {
    break;
  }
}
startTime = time[i];
/* Subtract the start time from all time stamps */
for(i = 0; i < numDataPoints; i++) {
  time[i] = time[i] - startTime;
}
/* Convert angles to displacement */
distances = angle2distance(radius, angles1_unwrapped);

/* Plot the unwrapped data */
plot.title("Displacement versus Time");
plot.label(PLOT_AXIS_X, "Time (seconds)");
plot.label(PLOT_AXIS_Y, "Displacement (inches)");
plot.data2D(time, distances);
plot.grid(PLOT_ON);
plot.plotting();
