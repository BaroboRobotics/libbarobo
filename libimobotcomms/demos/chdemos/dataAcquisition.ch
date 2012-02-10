/* Filename: dataAcquisition.ch
 * Make a graph of the robot's joint angle versus time1 */

#include <mobot.h>
#include <chplot.h>
#include <numeric.h>

int main() {
  CMobot robot;
  robot.connect();

  double speed = 45; /* Degrees / second */
  double angle = 720; /* Degrees */
  double timeInterval = 0.1; /* Seconds */

  /* Figure out how many data points we will need. First, figure out the
   * approximate amount of time1 the movement should take. */
  double movementTime = angle / speed; /* Seconds */
  /* Add an extra second of recording time to make sure the entire movement is
   * recorded */
  movementTime = movementTime + 1; 
  int numDataPoints = movementTime / timeInterval; /* Unitless */

  /* Initialize the arrays to be used to store data */
  array double time1[numDataPoints];
  array double angles1[numDataPoints];

  /* Declare plotting variables */
  CPlot plot1, plot2;
  array double angles1_unwrapped[numDataPoints];

  /* Start the motion. First, move robot to zero position */
  robot.moveToZero();
  /* Set the joint 1 speed to 45 degrees/second */
  robot.setJointSpeed(ROBOT_JOINT1, speed);

  /* Start capturing data */
  robot.recordAngle(ROBOT_JOINT1, time1, angles1, numDataPoints, timeInterval * 1000);

  /* Move the joint 720 degrees */
  robot.moveNB(angle, 0, 0, angle);
  robot.moveWait();
  
  /* Wait for recording to finish */
  robot.recordWait();

  /* Plot the data */
  plot1.title("Original Data for Joint Angle 1 versus Time");
  plot1.label(PLOT_AXIS_X, "Time (seconds)");
  plot1.label(PLOT_AXIS_Y, "Angle (degrees)");
  plot1.data2D(time1, angles1);
  plot1.plotting();

  /* Plot the unwrapped data */
  unwrapdeg(angles1_unwrapped, angles1);
  plot2.title("Unwrapped Data for Joint Angle 1 versus Time");
  plot2.label(PLOT_AXIS_X, "Time (seconds)");
  plot2.label(PLOT_AXIS_Y, "Angle (degrees)");
  plot2.data2D(time1, angles1_unwrapped);
  plot2.plotting();
  return 0;
}
