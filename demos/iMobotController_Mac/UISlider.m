//
//  UISlider.m
//  iMobotController_Mac
//
//  Created by dko on 3/6/12.
//  Copyright 2012 __MyCompanyName__. All rights reserved.
//

#import "UISlider.h"


@implementation UISlider

- (id) init {
	if(self = [super init]) {
	}
	isMouseDown = NO;
	isDirty = NO;
	return self;
}

- (BOOL) isMouseDown {
	return isMouseDown;
}

- (BOOL) isDirty {
	if(isMouseDown) {
		return YES;
	}
	return isDirty;
}

- (void) resetDirty {
	isDirty = NO;
}

- (BOOL) startTrackingAt:(NSPoint)startPoint inView:(NSView *)controlView {
	isMouseDown = YES;
	isDirty = YES;
	return [super startTrackingAt:startPoint inView:controlView];
}

- (void) stopTracking:(NSPoint)lastPoint at:(NSPoint)stopPoint inView:(NSView *)controlView mouseIsUp:(BOOL)flag {
	isMouseDown = NO;
	[super stopTracking:lastPoint at:stopPoint inView:controlView mouseIsUp:flag];
}


@end
