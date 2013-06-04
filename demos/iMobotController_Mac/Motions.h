//
//  Motions.h
//  iMobotController_Mac
//
//  Created by dko on 3/7/12.
//  Copyright 2012 __MyCompanyName__. All rights reserved.
//

#import <Cocoa/Cocoa.h>

#define MOTION_X(a, b) a,
enum motionTypes_e {
#include "Motions.x.h"
	MOTION_NUMMOTIONS
};
#undef MOTION_X

@interface Motions : NSObject <NSTableViewDataSource>{
	NSArray *entries;
}

/* NSTableViewDataSource Protocol Functions */
- (NSInteger) numberOfRowsInTableView:(NSTableView*) aTableView;
- (id)tableView:(NSTableView *)atableView 
objectValueForTableColumn:(NSTableColumn *)aTableColumn 
			row:(NSInteger)rowIndex;
- (void)tableView:(NSTableView *)aTableView 
   setObjectValue:(id)anObject 
   forTableColumn:(NSTableColumn*)aTableColumn 
			  row:(NSInteger)rowIndex;

@end
