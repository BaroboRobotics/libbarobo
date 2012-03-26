/* Filename: tank.ch
   Control three modules as a tank.
     |---------|--------|             
1st 1|    2    |   3    | 4    
     |---------|--------|             
            |------|
            |  1   | 3rd 
            |------|
     |---------|--------|       
2nd 1|    2    |   3    | 4    
     |---------|--------|     
*/
#include <mobot.h>
CMobot robot1;
CMobot robot2;
CMobot robot3;
int i;
/* Connect robot variables to the robot modules. */
robot1.connect();
robot2.connect();
robot3.connect();

/* Set the robot to "home" position, where all joint angles are 0 degrees. */
robot1.moveToZeroNB();
robot2.moveToZeroNB();
robot3.moveToZeroNB();
robot1.moveWait();
robot2.moveWait();
robot3.moveWait();

/* gun ready */
robot3.moveJoint(ROBOT_JOINT2, -90);

/* gun scan */
robot3.moveJointNB(ROBOT_JOINT1, 90);
robot3.moveJointToNB(ROBOT_JOINT2, 0);
robot3.moveWait();

robot3.moveJointNB(ROBOT_JOINT1, 90);
robot3.moveJointToNB(ROBOT_JOINT2, -90);
robot3.moveWait();

robot3.moveJointNB(ROBOT_JOINT1, 90);
robot3.moveJointToNB(ROBOT_JOINT2, 0);
robot3.moveWait();

robot3.moveJointNB(ROBOT_JOINT1, 90);
robot3.moveJointToNB(ROBOT_JOINT2, -90);
robot3.moveWait();

/* move forward */
robot1.motionRollForwardNB(360);
robot2.motionRollForwardNB(360);
robot1.motionWait();
robot2.motionWait();

/* move backward */
robot1.motionRollForwardNB(360);
robot2.motionRollForwardNB(360);
robot1.motionWait();
robot2.motionWait();

/* turn right */
robot1.motionTurnRightNB(90);
robot2.motionTurnRightNB(90);
robot1.motionWait();
robot2.motionWait();

/* move forward */
robot1.motionRollForwardNB(360);
robot2.motionRollForwardNB(360);
robot1.motionWait();
robot2.motionWait();

/* turn left */
robot1.motionTurnLeftNB(90);
robot2.motionTurnLeftNB(90);
robot1.motionWait();
robot2.motionWait();
