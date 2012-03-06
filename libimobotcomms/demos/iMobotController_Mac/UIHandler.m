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

@end
