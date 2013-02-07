/* Test a continuous motion.
 *
 * Strategy: Begin rotating faceplates at known speed for X amount of time.
 * Record the motion. Perform a linear regression on recorded data and make
 * sure the slope is correct, R^2 value is acceptable, etc. */

#include <mobot.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ANGLE_TOLERANCE 2
#define ABS(x) ((x) < 0? -1*(x) : (x))
int checkAngle(double measuredAngle, double expectedAngle)
{
  if(ABS(measuredAngle-expectedAngle) > ANGLE_TOLERANCE) {
    return -1;
  } else {
    return 0;
  }
}

#define ERRMSG(msg) errMsg(msg, __FILE__, __LINE__)
void errMsg(const char* msg, const char* file, int line)
{
  char* buf;
  buf = (char*)malloc(sizeof(char)*(strlen(msg)+200));
  sprintf(buf, "Error: %s %s:%d\n", msg, file, line);
  fprintf(stderr, "%s", buf);
  free(buf);
}

int main()
{
  double *time;
  double **angles;
  int n;
  double slope;
  double intercept;
  CMobot mobot;
  angles = (double**)malloc(sizeof(double*)*4);
  mobot.connect();
  if(!mobot.isConnected()) {
    ERRMSG("Connect failed");
    return -1;
  }
  mobot.setJointSpeeds(45, 45, 45, 45);
  mobot.resetToZero();
  mobot.moveContinuousNB(MOBOT_FORWARD, MOBOT_HOLD, MOBOT_HOLD, MOBOT_BACKWARD);
  mobot.recordAnglesBegin(time, 
                          angles[0], 
                          angles[1], 
                          angles[2], 
                          angles[3], 
                          0.1);
#ifndef _WIN32
  sleep(12);
#else
  Sleep(12000);
#endif
  printf("Sleep done.\n");
  mobot.recordAnglesEnd(n);
  printf("Ended Recording.\n");
  mobot.stopAllJoints();
  printf("Joints Stopped.\n");

  /* Check end angle. */
  double angle;
  mobot.getJointAngle(MOBOT_JOINT1, angle);
  if(ABS(angle - 45*12) > 20) {
    ERRMSG("Mobot movement speed incorrect");
    return -1;
  }
  
  /* Perform least squares regression */
  double sum_y = 0;
  double sum_y_2 = 0; // sum(y^2)
  double sum_x = 0;
  double sum_x_2 = 0;
  double sum_xy = 0;
  double angle_mean = 0;
  int i;
  for(i = 0; i < n; i++) {
    sum_y += angles[0][i];
    sum_y_2 += angles[0][i]*angles[0][i];
    sum_x += time[i];
    sum_x_2 += time[i] * time[i];
    sum_xy += angles[0][i] * time[i];
    angle_mean += angles[0][i]/n;
  }
  intercept = ((sum_y * sum_x_2) - (sum_x * sum_xy)) / (((double)n)*sum_x_2 - sum_x*sum_x);
  slope = ((double)n * sum_xy - sum_x * sum_y)/ (((double)n)*sum_x_2 - sum_x*sum_x);
  printf("Slope is %lf\n", slope);
  if(checkAngle(slope, 45)) {
    ERRMSG("Slope incorrect");
    return -1;
  }
  /* Check the residual */
  double SS_tot = 0;
  double SS_err = 0;
  double R_2 = 0;
  double diff;
  for(i = 0; i < n; i++) {
    SS_tot += pow(angles[0][i] - angle_mean, 2);
    SS_err += pow(angles[0][i] - (slope*time[i] + intercept), 2);
  }
  R_2 =  1 - SS_err/SS_tot;
  printf("R squared: %lf\n", R_2);
  if(R_2 < 0.97) {
    ERRMSG("Recorded data too noisy");
  }

  /* Now do it again but faster */
  mobot.setJointSpeeds(60, 60, 60, 60);
  mobot.resetToZero();
  mobot.moveContinuousNB(MOBOT_FORWARD, MOBOT_HOLD, MOBOT_HOLD, MOBOT_BACKWARD);
  mobot.recordAnglesBegin(time, 
                          angles[0], 
                          angles[1], 
                          angles[2], 
                          angles[3], 
                          0.1);
#ifndef _WIN32
  sleep(12);
#else
  Sleep(12);
#endif
  printf("Sleep done.\n");
  mobot.recordAnglesEnd(n);
  printf("Ended Recording.\n");
  mobot.stopAllJoints();
  printf("Joints Stopped.\n");

  /* Check end angle. */
  mobot.getJointAngle(MOBOT_JOINT1, angle);
  if(ABS(angle - 60*12) > 30) {
    ERRMSG("Mobot movement speed incorrect");
    return -1;
  }
  
  /* Perform least squares regression */
  sum_y = 0;
  sum_y_2 = 0; // sum(y^2)
  sum_x = 0;
  sum_x_2 = 0;
  sum_xy = 0;
  angle_mean = 0;
  for(i = 0; i < n; i++) {
    sum_y += angles[0][i];
    sum_y_2 += angles[0][i]*angles[0][i];
    sum_x += time[i];
    sum_x_2 += time[i] * time[i];
    sum_xy += angles[0][i] * time[i];
    angle_mean += angles[0][i]/n;
  }
  intercept = ((sum_y * sum_x_2) - (sum_x * sum_xy)) / (((double)n)*sum_x_2 - sum_x*sum_x);
  slope = ((double)n * sum_xy - sum_x * sum_y)/ (((double)n)*sum_x_2 - sum_x*sum_x);
  printf("Slope is %lf\n", slope);
  if(checkAngle(slope, 60)) {
    ERRMSG("Slope incorrect");
    return -1;
  }
  /* Check the residual */
  SS_tot = 0;
  SS_err = 0;
  R_2 = 0;
  for(i = 0; i < n; i++) {
    SS_tot += pow(angles[0][i] - angle_mean, 2);
    SS_err += pow(angles[0][i] - (slope*time[i] + intercept), 2);
  }
  R_2 =  1 - SS_err/SS_tot;
  printf("R squared: %lf\n", R_2);
  if(R_2 < 0.97) {
    ERRMSG("Recorded data too noisy");
  }

  return 0;  
}
