#include <mobot.h>

int main()
{
  CLinkbotI master, slave;
  master.connect();
  slave.connect();
  double accel_x, accel_y, accel_z;
  int motorPower, motorOffset;
  while(1) {
    /* Get the accelerometer values */
    master.getAccelerometerData(accel_x, accel_y, accel_z);
    /* Calculate forward-backward */
    motorPower = 500 * accel_y;
    motorOffset = 256 * accel_x;
    slave.setMotorPower(MOBOT_JOINT1, -motorPower - motorOffset);
    slave.setMotorPower(MOBOT_JOINT2, motorPower - motorOffset);
  }
  return 0;
}
