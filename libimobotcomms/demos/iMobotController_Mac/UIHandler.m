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

@synthesize textFieldSpeed1;
@synthesize textFieldSpeed2;
@synthesize textFieldSpeed3;
@synthesize textFieldSpeed4;

@synthesize textFieldSetSpeed1;
@synthesize textFieldSetSpeed2;
@synthesize textFieldSetSpeed3;
@synthesize textFieldSetSpeed4;

@synthesize textFieldPosition1;
@synthesize textFieldPosition2;
@synthesize textFieldPosition3;
@synthesize textFieldPosition4;

@synthesize textFieldSetPosition1;
@synthesize textFieldSetPosition2;
@synthesize textFieldSetPosition3;
@synthesize textFieldSetPosition4;

@synthesize tableViewMotions;

- (id) init {
	if (self = [super init]) {
		/* Initialize sliders */
		[self initSliders];
	}
	motions = [[Motions alloc] init];
	[motions retain];
	[tableViewMotions setDataSource:motions];
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
	
	textFieldSpeeds[0] = textFieldSpeed1;
	textFieldSpeeds[1] = textFieldSpeed2;
	textFieldSpeeds[2] = textFieldSpeed3;
	textFieldSpeeds[3] = textFieldSpeed4;
	
	textFieldSetSpeeds[0] = textFieldSetSpeed1;
	textFieldSetSpeeds[1] = textFieldSetSpeed2;
	textFieldSetSpeeds[2] = textFieldSetSpeed3;
	textFieldSetSpeeds[3] = textFieldSetSpeed4;
	
	textFieldPositions[0] = textFieldPosition1;
	textFieldPositions[1] = textFieldPosition2;
	textFieldPositions[2] = textFieldPosition3;
	textFieldPositions[3] = textFieldPosition4;
	
	textFieldSetPositions[0] = textFieldSetPosition1;
	textFieldSetPositions[1] = textFieldSetPosition2;
	textFieldSetPositions[2] = textFieldSetPosition3;
	textFieldSetPositions[3] = textFieldSetPosition4;
}

- (void) main {
	/* Main UI Processing Loop */
	double timestamp;
	double positions[4];
	int i;
	
	for(i = 0; i < 4; i++) {
		[textFieldSpeeds[i] setDoubleValue:45];
	}

	while(![self isCancelled]) {
		/* Get motor positions */
		Mobot_getJointAnglesTime(comms, &timestamp, &positions[0], &positions[1], &positions[2], &positions[3]);

		/* Check the slider positions. If the mouse is down, move the motors. */
		for(i = 0; i < 4; i++) {
			[textFieldPositions[i] setDoubleValue:RAD2DEG(positions[i])];
			if( [sliderPositions[i] isMouseDown] ) {
				Mobot_moveJointToPIDNB(comms, i+1, DEG2RAD([sliderPositions[i] doubleValue]));
			} else {
				/* Set the slider positions */
				[sliderPositions[i] setDoubleValue:RAD2DEG(positions[i])];
			}
			
			if( [sliderSpeeds[i] isMouseDown] ) {
				Mobot_setJointSpeed(comms, i+1, DEG2RAD([sliderSpeeds[i] doubleValue]));
				[textFieldSpeeds[i] setDoubleValue:[sliderSpeeds[i] doubleValue]];
			}
		}
	}
	/* Reset text fields, sliders */
	[self initSliders];
	for(i = 0; i < 4; i++) {
		[textFieldSpeeds[i] setStringValue:@""];
		[textFieldPositions[i] setStringValue:@""];
	}
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

- (IBAction) onButtonSetSpeed:(id)sender {
	int i;
	for(i = 0; i < 4; i++) {
		if( [[textFieldSetSpeeds[i] stringValue] length] > 0) {
			Mobot_setJointSpeed(comms, i+1, [textFieldSetSpeeds[i] doubleValue]);
			[textFieldSpeeds[i] setDoubleValue:[textFieldSetSpeeds[i] doubleValue]];
			[sliderSpeeds[i] setDoubleValue:[textFieldSetSpeeds[i] doubleValue]];
		}
	}
}

- (IBAction) onButtonMoveTo:(id)sender {
	int i;
	for(i = 0; i < 4; i++) {
		if( [[textFieldSetPositions[i] stringValue] length] > 0) {
			Mobot_moveJointToNB(comms, i+1, DEG2RAD([textFieldSetPositions[i] doubleValue]));
		}
	}
}

- (IBAction) onButtonMove:(id)sender {
	int i;
	for(i = 0; i < 4; i++) {
		if( [[textFieldSetPositions[i] stringValue] length] > 0) {
			Mobot_moveJointNB(comms, i+1, DEG2RAD([textFieldSetPositions[i] doubleValue]));
		}
	}
}

- (IBAction) onButtonPlay:(id)sender {
	NSInteger index = [tableViewMotions selectedRow];
	switch(index) {
		case MOTION_ARCH: 
			Mobot_motionArchNB(comms, DEG2RAD(30));
			break;
		case MOTION_INCHWORMLEFT:
			Mobot_motionInchwormLeftNB(comms, 1);
			break;
		case MOTION_INCHWORMRIGHT:
			Mobot_motionInchwormRightNB(comms, 1);
			break;
		case MOTION_ROLLBACKWARD:
			Mobot_motionRollBackwardNB(comms, DEG2RAD(360));
			break;
		case MOTION_ROLLFORWARD:
			Mobot_motionRollForwardNB(comms, DEG2RAD(360));
			break;
		case MOTION_SKINNY:
			Mobot_motionSkinnyNB(comms, DEG2RAD(90));
			break;
		case MOTION_STAND:
			Mobot_motionStandNB(comms);
			break;
		case MOTION_TURNLEFT:
			Mobot_motionTurnLeftNB(comms, DEG2RAD(360));
			break;
		case MOTION_TURNRIGHT:
			Mobot_motionTurnRightNB(comms, DEG2RAD(360));
			break;
		case MOTION_TUMBLEBACKWARD:
			Mobot_motionTumbleBackwardNB(comms, 1);
			break;
		case MOTION_TUMBLEFORWARD:
			Mobot_motionTumbleForwardNB(comms, 1);
			break;
		case MOTION_UNSTAND:
			Mobot_motionUnstandNB(comms);
			break;
		default:
			return;
	}
}


@end
