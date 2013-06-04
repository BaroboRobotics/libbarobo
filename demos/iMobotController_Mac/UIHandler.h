//
//  UIHandler.h
//  iMobotController_Mac
//
//  Created by dko on 3/5/12.
//  Copyright 2012 __MyCompanyName__. All rights reserved.
//

#import <Cocoa/Cocoa.h>
#import "UISlider.h"
#import "Motions.h"
#include <mobot.h>

@interface UIHandler : NSThread {
	UISlider *sliderSpeed1;
	UISlider *sliderSpeed2;
	UISlider *sliderSpeed3;
	UISlider *sliderSpeed4;
	UISlider *sliderSpeeds[4];
	
	UISlider *sliderPosition1;
	UISlider *sliderPosition2;
	UISlider *sliderPosition3;
	UISlider *sliderPosition4;
	UISlider *sliderPositions[4];

	NSTextField *textFieldSpeed1;
	NSTextField *textFieldSpeed2;
	NSTextField *textFieldSpeed3;
	NSTextField *textFieldSpeed4;
	NSTextField *textFieldSpeeds[4];

	NSTextField *textFieldSetSpeed1;
	NSTextField *textFieldSetSpeed2;
	NSTextField *textFieldSetSpeed3;
	NSTextField *textFieldSetSpeed4;
	NSTextField *textFieldSetSpeeds[4];

	NSTextField *textFieldPosition1;
	NSTextField *textFieldPosition2;
	NSTextField *textFieldPosition3;
	NSTextField *textFieldPosition4;
	NSTextField *textFieldPositions[4];

	NSTextField *textFieldSetPosition1;
	NSTextField *textFieldSetPosition2;
	NSTextField *textFieldSetPosition3;
	NSTextField *textFieldSetPosition4;
	NSTextField *textFieldSetPositions[4];
	
	NSTableView *tableViewMotions;
	Motions *motions;

	mobot_t *comms;
}

@property (retain) IBOutlet UISlider *sliderSpeed1;
@property (retain) IBOutlet UISlider *sliderSpeed2;
@property (retain) IBOutlet UISlider *sliderSpeed3;
@property (retain) IBOutlet UISlider *sliderSpeed4;

@property (retain) IBOutlet UISlider *sliderPosition1;
@property (retain) IBOutlet UISlider *sliderPosition2;
@property (retain) IBOutlet UISlider *sliderPosition3;
@property (retain) IBOutlet UISlider *sliderPosition4;

@property (retain) IBOutlet NSTextField *textFieldSpeed1;
@property (retain) IBOutlet NSTextField *textFieldSpeed2;
@property (retain) IBOutlet NSTextField *textFieldSpeed3;
@property (retain) IBOutlet NSTextField *textFieldSpeed4;

@property (retain) IBOutlet NSTextField *textFieldSetSpeed1;
@property (retain) IBOutlet NSTextField *textFieldSetSpeed2;
@property (retain) IBOutlet NSTextField *textFieldSetSpeed3;
@property (retain) IBOutlet NSTextField *textFieldSetSpeed4;

@property (retain) IBOutlet NSTextField *textFieldPosition1;
@property (retain) IBOutlet NSTextField *textFieldPosition2;
@property (retain) IBOutlet NSTextField *textFieldPosition3;
@property (retain) IBOutlet NSTextField *textFieldPosition4;

@property (retain) IBOutlet NSTextField *textFieldSetPosition1;
@property (retain) IBOutlet NSTextField *textFieldSetPosition2;
@property (retain) IBOutlet NSTextField *textFieldSetPosition3;
@property (retain) IBOutlet NSTextField *textFieldSetPosition4;

@property (retain) IBOutlet NSTableView *tableViewMotions;

- (mobot_t*) comms;

- (id) init;
- (id) initWithBRComms:(mobot_t*)br_comms;
- (void) initSliders;
- (void) main;

/* UI Button Handlers */
- (IBAction) onButtonMoveToZero:(id)sender;
- (IBAction) onButtonJoint1Forward:(id)sender;
- (IBAction) onButtonJoint2Forward:(id)sender;
- (IBAction) onButtonJoint3Forward:(id)sender;
- (IBAction) onButtonJoint4Forward:(id)sender;
- (IBAction) onButtonJoint1Stop:(id)sender;
- (IBAction) onButtonJoint2Stop:(id)sender;
- (IBAction) onButtonJoint3Stop:(id)sender;
- (IBAction) onButtonJoint4Stop:(id)sender;
- (IBAction) onButtonJoint1Backward:(id)sender;
- (IBAction) onButtonJoint2Backward:(id)sender;
- (IBAction) onButtonJoint3Backward:(id)sender;
- (IBAction) onButtonJoint4Backward:(id)sender;
- (IBAction) onButtonRollForward:(id)sender;
- (IBAction) onButtonRollBackward:(id)sender;
- (IBAction) onButtonRollLeft:(id)sender;
- (IBAction) onButtonRollRight:(id)sender;
- (IBAction) onButtonStop:(id)sender;
- (IBAction) onButtonSetSpeed:(id)sender;
- (IBAction) onButtonMoveTo:(id)sender;
- (IBAction) onButtonMove:(id)sender;
- (IBAction) onButtonPlay:(id)sender;

@end
