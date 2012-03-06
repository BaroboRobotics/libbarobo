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
}

@property (retain) IBOutlet NSSlider *sliderSpeed1;
@property (retain) IBOutlet NSSlider *sliderSpeed2;
@property (retain) IBOutlet NSSlider *sliderSpeed3;
@property (retain) IBOutlet NSSlider *sliderSpeed4;

@property (retain) IBOutlet NSSlider *sliderPosition1;
@property (retain) IBOutlet NSSlider *sliderPosition2;
@property (retain) IBOutlet NSSlider *sliderPosition3;
@property (retain) IBOutlet NSSlider *sliderPosition4;

- (id) init;
- (void) initSliders;
- (void) main;

@end
