/* Filename: fourWheelDrive.ch
   Control one module two wheels.
    |                      |
    | |---------|--------| |            
    |1|    2    |   3    |4|    
    | |---------|--------| |
    |        -------       |
                |

Joint 2 and joint 3 are fixed by a connector with a metal caster.
Joint 1 and joint 4 have wheels attached. */
#include <mobot.h>
CMobot mobot;

/* Connect mobot variables to the mobot modules. */
mobot.connect();

/* Set the mobot to "home" position, where all joint angles are 0 degrees. */
mobot.moveToZero();

mobot.setJointSpeedRatios(0.4, 0.5, 0.5, 0.4);

/* rolling forward*/
mobot.motionRollForward(360);

/* rolling backward*/
mobot.motionRollForward(-360);

/* turn right and left */
mobot.motionTurnRight(360);
mobot.motionTurnLeft(360);
