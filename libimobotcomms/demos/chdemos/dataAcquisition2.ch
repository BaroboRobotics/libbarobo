/* Filename: dataAcquisition2.ch
 * Make a graph of the mobot's joint angle versus time */

#include <mobot.h>
#include <chplot.h>
CMobot mobot;

/* Connect to the mobot */
mobot.connect();

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
double tolerance = 2.0; /* 1 degree for time shifting */

/* Set all joint speeds to 45 degrees/second */
mobot.setJointSpeeds(45, 45, 45, 45);

/* Start the motion. First, move mobot to zero position */
mobot.resetToZero();

/* Start capturing data */
mobot.recordAngles( time, 
                    angles1, 
                    angles2, 
                    angles3, 
                    angles4, 
                    numDataPoints, 
                    timeInterval, 
                    tolerance);

/* Perform the standing and unstanding motions */
mobot.motionTurnRight(360);
mobot.motionInchwormLeft(2);

/* Wait for recording to finish */
mobot.recordWait();

plot.title("Unwrapped Data for Joint Angles versus Time");
plot.label(PLOT_AXIS_X, "Time (seconds)");
plot.label(PLOT_AXIS_Y, "Angle (degrees)");
plot.data2DCurve(time, angles1, numDataPoints);
plot.data2DCurve(time, angles2, numDataPoints);
plot.data2DCurve(time, angles3, numDataPoints);
plot.data2DCurve(time, angles4, numDataPoints);
plot.legend("Joint 1", 0);
plot.legend("Joint 2", 1);
plot.legend("Joint 3", 2);
plot.legend("Joint 4", 3);
plot.grid(PLOT_ON);
plot.plotting();
