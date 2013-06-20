#include <linkbot.h>

int main()
{
  CLinkbotI master, slave;
  master.connect();
  slave.connect();
  double accel_x, accel_y, accel_z;
  int motorPower, motorOffset;
  slave.stop(); // Make the slave relax motors first
  while(1) {
    /* Get the accelerometer values */
    master.getAccelerometerData(accel_x, accel_y, accel_z);
    /* Calculate forward-backward */
    motorPower = 500 * accel_y;
    motorOffset = 256 * accel_x;
    slave.setMotorPower(ROBOT_JOINT1, -motorPower - motorOffset);
    slave.setMotorPower(ROBOT_JOINT2, motorPower - motorOffset);
  }
  return 0;
}
