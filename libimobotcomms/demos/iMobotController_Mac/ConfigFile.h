//
//  ConfigFile.h
//  iMobotController_Mac
//
//  Created by dko on 2/29/12.
//  Copyright 2012 __MyCompanyName__. All rights reserved.
//

#import <Cocoa/Cocoa.h>


@interface ConfigFile : NSObject <NSTableViewDataSource>{
	NSString *configFileName;
	FILE *fp;
	NSArray *entries;
}

- (id) initWithFilename:(NSString*)fileName;
- (void) addAddress:(NSString*)address;
- (NSString*) getAddress:(int)index;
- (void) moveAddressUp:(int)index;
- (void) moveAddressDown:(int)index;
- (void) refresh;

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
