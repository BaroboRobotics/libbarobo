//
//  UIHandler.h
//  iMobotController_Mac
//
//  Created by dko on 3/5/12.
//  Copyright 2012 __MyCompanyName__. All rights reserved.
//

#import <Cocoa/Cocoa.h>
#include <mobot.h>

@interface UIHandler : NSThread {
	NSSlider *sliderSpeed1;
	NSSlider *sliderSpeed2;
	NSSlider *sliderSpeed3;
	NSSlider *sliderSpeed4;
	NSSlider *sliderSpeeds[4];
	
	NSSlider *sliderPosition1;
	NSSlider *sliderPosition2;
	NSSlider *sliderPosition3;
	NSSlider *sliderPosition4;
	NSSlider *sliderPositions[4];
	br_comms_t *comms;
}

@property (retain) IBOutlet NSSlider *sliderSpeed1;
@property (retain) IBOutlet NSSlider *sliderSpeed2;
@property (retain) IBOutlet NSSlider *sliderSpeed3;
@property (retain) IBOutlet NSSlider *sliderSpeed4;

@property (retain) IBOutlet NSSlider *sliderPosition1;
@property (retain) IBOutlet NSSlider *sliderPosition2;
@property (retain) IBOutlet NSSlider *sliderPosition3;
@property (retain) IBOutlet NSSlider *sliderPosition4;

- (br_comms_t*) comms;

- (id) init;
- (id) initWithBRComms:(br_comms_t*)br_comms;
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

@end
