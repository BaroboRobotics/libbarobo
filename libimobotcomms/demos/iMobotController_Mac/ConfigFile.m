//
//  ConfigFile.m
//  iMobotController_Mac
//
//  Created by dko on 2/29/12.
//  Copyright 2012 __MyCompanyName__. All rights reserved.
//
#import <Foundation/NSString.h>
#import "ConfigFile.h"


@implementation ConfigFile

- (id) initWithFilename:(NSString *)fileName {
	configFileName = fileName;
	/* Open the file, read the entries */
	NSString *fileContents = 
			[NSString stringWithContentsOfFile:fileName
									  encoding:NSASCIIStringEncoding
										 error:NULL];
	entries = [fileContents componentsSeparatedByString:@"\n"];
	[entries retain];
	return self;
}

- (void) addAddress:(NSString*)address {
	/* We need to add the new address to the beginning of the array. 
	 Since there is no "insert" method, we create a new array of 
	 one element and append the original array. */
	NSArray *newArray = [NSArray arrayWithObject:address];
	entries = [newArray arrayByAddingObjectsFromArray:entries];
	[self refresh];
}

/* NSTableViewDataSource Protocol Functions */
- (NSInteger) numberOfRowsInTableView:(NSTableView*) aTableView {
	return [entries count];
}

- (id)tableView:(NSTableView *)atableView 
objectValueForTableColumn:(NSTableColumn *)aTableColumn 
			row:(NSInteger)rowIndex {
	return [entries objectAtIndex:rowIndex];
}

- (void)tableView:(NSTableView *)aTableView 
 setObjectValue:(id)anObject 
 forTableColumn:(NSTableColumn*)aTableColumn 
			row:(NSInteger)rowIndex {
	/* Get the object */
	NSString *object = [entries objectAtIndex:rowIndex];
	/* Set the object to the new value */
	[object initWithString:anObject];
}


@end
