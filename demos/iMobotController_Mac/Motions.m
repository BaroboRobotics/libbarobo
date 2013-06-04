//
//  Motions.m
//  iMobotController_Mac
//
//  Created by dko on 3/7/12.
//  Copyright 2012 __MyCompanyName__. All rights reserved.
//

#import "Motions.h"

#define MOTION_X(a, b) b,
const char *motionStrings[32] = {
#include  "Motions.x.h"
	NULL };

@implementation Motions
/* NSTableViewDataSource Protocol Functions */
- (NSInteger) numberOfRowsInTableView:(NSTableView*) aTableView {
	return MOTION_NUMMOTIONS;
}

- (id)tableView:(NSTableView *)atableView 
objectValueForTableColumn:(NSTableColumn *)aTableColumn 
			row:(NSInteger)rowIndex {
	NSString *value = [[NSString alloc] initWithCString:motionStrings[rowIndex] encoding:NSASCIIStringEncoding];
	[value retain];
	return value;
}

- (void)tableView:(NSTableView *)aTableView 
   setObjectValue:(id)anObject 
   forTableColumn:(NSTableColumn*)aTableColumn 
			  row:(NSInteger)rowIndex {
}

@end
