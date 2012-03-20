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
	[configFileName initWithString:fileName];
	[configFileName retain];
	/* Open the file, read the entries */
	NSString *fileContents = 
			[NSString stringWithContentsOfFile:fileName
									  encoding:NSASCIIStringEncoding
										 error:NULL];
	entries = [NSMutableArray arrayWithArray:[fileContents componentsSeparatedByString:@"\n"]];
	[entries retain];
	return self;
}

- (void) addAddress:(NSString*)address {
	[entries insertObject:address atIndex:0];
}

- (void) removeAtIndex:(NSInteger)rowIndex {
	[entries removeObjectAtIndex:rowIndex];
}

- (void) insertString:(NSString*)aString atIndex:(NSInteger)index
{
	[entries insertObject:aString atIndex:index];
}

- (NSString*) getAddress:(NSInteger)index {
	return [entries objectAtIndex:index];
}

- (void) moveAddressUp:(NSInteger)index {
	/* Strategy: 
	 - Save the text at index
	 - Remove index
	 - insert saved text at index-1
	 */
	NSString* address;
	if (index >= ([entries count])) {
		return;
	}
	if(index < 1) { return; }
	address = [entries objectAtIndex:index];
	[address retain];
	[entries removeObjectAtIndex:index];
	[entries insertObject:address atIndex:(index-1)];
}

- (void) moveAddressDown:(NSInteger)index {
	/* Strategy: 
	 - Save the text at index
	 - Remove index
	 - insert saved text at index+1
	 */
	NSString* address;
	if (index >= ([entries count]-1)) {
		return;
	}
	if(index < 0) { return; }
	address = [entries objectAtIndex:index];
	[address retain];
	[entries removeObjectAtIndex:index];
	[entries insertObject:address atIndex:(index+1)];
}

- (void) writeFile {
	NSString *fileContents = [[NSString alloc] init];
	int i;
	for(i = 0 ; i < [entries count]; i++) {
		fileContents = [fileContents stringByAppendingString:[entries objectAtIndex:i]];
		fileContents = [fileContents stringByAppendingString:@"\n"];
	}
	[fileContents writeToFile:configFileName 
				   atomically:false
					 encoding:NSASCIIStringEncoding
						error:NULL];
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
	[entries replaceObjectAtIndex:rowIndex withObject:anObject];
}


@end
