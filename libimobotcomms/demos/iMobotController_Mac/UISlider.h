//
//  UISlider.h
//  iMobotController_Mac
//
//  Created by dko on 3/6/12.
//  Copyright 2012 __MyCompanyName__. All rights reserved.
//

#import <Cocoa/Cocoa.h>


@interface UISlider : NSSliderCell {
	BOOL isMouseDown;
	BOOL isDirty;
}

- (id) init;

- (BOOL) isMouseDown;
- (BOOL) isDirty;
- (void) resetDirty;

- (BOOL) startTrackingAt:(NSPoint)startPoint inView:(NSView *)controlView;
- (void) stopTracking:(NSPoint)lastPoint at:(NSPoint)stopPoint inView:(NSView *)controlView mouseIsUp:(BOOL)flag;

@end
