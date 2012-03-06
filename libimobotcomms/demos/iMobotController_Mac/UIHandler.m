//
//  UIHandler.m
//  iMobotController_Mac
//
//  Created by dko on 3/5/12.
//  Copyright 2012 __MyCompanyName__. All rights reserved.
//

#import "UIHandler.h"


@implementation UIHandler

@synthesize sliderSpeed1;
@synthesize sliderSpeed2;
@synthesize sliderSpeed3;
@synthesize sliderSpeed4;

@synthesize sliderPosition1;
@synthesize sliderPosition2;
@synthesize sliderPosition3;
@synthesize sliderPosition4;

- (id) init {
	if (self = [super init]) {
		/* Initialize sliders */
		[self initSliders];
	}
	return self;
}

- (id) initWithBRComms:(br_comms_t*)br_comms {
	comms = br_comms;
	return [self init];
}

- (br_comms_t*) comms {
	return comms;
}

- (void) initSliders {
	sliderSpeeds[0] = sliderSpeed1;
	sliderSpeeds[1] = sliderSpeed2;
	sliderSpeeds[2] = sliderSpeed3;
	sliderSpeeds[3] = sliderSpeed4;
	int i;
	for(i = 0; i < 4; i++) {
		[sliderSpeeds[i] setMaxValue:120];
		[sliderSpeeds[i] setMinValue:0];
		[sliderSpeeds[i] setFloatValue:45];
	}
	[sliderSpeed1 setDoubleValue:45];
	
	sliderPositions[0] = sliderPosition1;
	sliderPositions[1] = sliderPosition2;
	sliderPositions[2] = sliderPosition3;
	sliderPositions[3] = sliderPosition4;
	for(i = 0; i < 4; i += 3) {
		[sliderPositions[i] setMinValue:-180];
		[sliderPositions[i] setMaxValue:180];
	}
	for(i = 1; i < 3; i++) {
		[sliderPositions[i] setMinValue:-90];
		[sliderPositions[i] setMaxValue:90];
	}
}

- (void) main {
}

/* Button Handlers */

- (IBAction) onButtonMoveToZero:(id)sender {
	Mobot_moveToZeroNB([self comms]);
}

- (IBAction) onButtonJoint1Forward:(id)sender {
	Mobot_moveJointContinuousNB(comms, ROBOT_JOINT1, ROBOT_FORWARD);
}

- (IBAction) onButtonJoint2Forward:(id)sender {
	Mobot_moveJointContinuousNB(comms, ROBOT_JOINT2, ROBOT_FORWARD);
}

- (IBAction) onButtonJoint3Forward:(id)sender {
	Mobot_moveJointContinuousNB(comms, ROBOT_JOINT3, ROBOT_FORWARD);
}

- (IBAction) onButtonJoint4Forward:(id)sender {
	Mobot_moveJointContinuousNB(comms, ROBOT_JOINT4, ROBOT_FORWARD);
}

- (IBAction) onButtonJoint1Stop:(id)sender {
	Mobot_moveJointContinuousNB(comms, ROBOT_JOINT1, ROBOT_HOLD);
}

- (IBAction) onButtonJoint2Stop:(id)sender {
	Mobot_moveJointContinuousNB(comms, ROBOT_JOINT2, ROBOT_HOLD);
}

- (IBAction) onButtonJoint3Stop:(id)sender {
	Mobot_moveJointContinuousNB(comms, ROBOT_JOINT3, ROBOT_HOLD);
}

- (IBAction) onButtonJoint4Stop:(id)sender {
	Mobot_moveJointContinuousNB(comms, ROBOT_JOINT4, ROBOT_HOLD);
}

- (IBAction) onButtonJoint1Backward:(id)sender {
	Mobot_moveJointContinuousNB(comms, ROBOT_JOINT1, ROBOT_BACKWARD);
}

- (IBAction) onButtonJoint2Backward:(id)sender {
	Mobot_moveJointContinuousNB(comms, ROBOT_JOINT2, ROBOT_BACKWARD);
}

- (IBAction) onButtonJoint3Backward:(id)sender {
	Mobot_moveJointContinuousNB(comms, ROBOT_JOINT3, ROBOT_BACKWARD);
}

- (IBAction) onButtonJoint4Backward:(id)sender {
	Mobot_moveJointContinuousNB(comms, ROBOT_JOINT4, ROBOT_BACKWARD);
}

- (IBAction) onButtonRollForward:(id)sender {
	Mobot_motionRollForwardNB(comms, DEG2RAD(360));
}

- (IBAction) onButtonRollBackward:(id)sender {
	Mobot_motionRollBackwardNB(comms, DEG2RAD(360));
}

- (IBAction) onButtonRollLeft:(id)sender {
	Mobot_motionTurnLeftNB(comms, DEG2RAD(360));
}

- (IBAction) onButtonRollRight:(id)sender {
	Mobot_motionTurnRightNB(comms, DEG2RAD(360));
}

- (IBAction) onButtonStop:(id)sender {
	Mobot_stop(comms);
}


@end
