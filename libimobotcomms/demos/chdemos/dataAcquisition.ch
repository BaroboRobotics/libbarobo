/* Filename: dataAcquisition.ch
 * Make a graph of the robot's joint angle versus time */

#include <mobot.h>
#include <chplot.h>
#include <time.h>

int main() {
  CMobot robot;
  robot.connect();

  double speed = 45; /* Degrees / second */
  double angle = 720; /* Degrees */
  double timeInterval = 0.1; /* Seconds */

  /* Figure out how many data points we will need. First, figure out the
   * approximate amount of time the movement should take. */
  double movementTime = angle / speed; /* Seconds */
  int numDataPoints = movementTime / timeInterval; /* Unitless */
  /* Add 20 more data points to make sure we capture the end of the motion */
  numDataPoints += 20;

  /* Initialize the arrays to be used to store data */
  array double x[numDataPoints];
  array double y[numDataPoints];

  /* Start the motion. First, move robot to zero position */
  robot.moveToZero();
  /* Wait for joints to settle */
  msleep(1000);
  /* Set the joint 1 speed to 45 degrees/second */
  robot.setJointSpeed(ROBOT_JOINT1, speed);
  /* Move the joint 720 degrees */
  robot.moveNB(angle, 0, 0, angle);

  /* Start capturing data */
  int i;
  double time = 0;
  for(i = 0; i < numDataPoints; i++) {
    x[i] = time;
    robot.getJointAngle(ROBOT_JOINT1, angle);
    y[i] = angle;
    msleep(100-44);
    //usleep(50000);
    time += 0.1;
  }

  /* Plot the data */
  CPlot plot;
  /* Convert y to radians */
  y = y * M_PI / 180.0;
  array double y_unwrapped[numDataPoints];
  unwrap(y_unwrapped, y);
  /* Convert back to degrees */
  y_unwrapped = y_unwrapped * 180.0 / M_PI;
  plot.data2D(x, y_unwrapped);
  plot.plotting();
  return 0;
}
