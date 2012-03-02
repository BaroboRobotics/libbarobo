//
//  iMobotController_MacAppDelegate.m
//  iMobotController_Mac
//
//  Created by dko on 2/28/12.
//  Copyright 2012 __MyCompanyName__. All rights reserved.
//

#import "iMobotController_MacAppDelegate.h"

@implementation iMobotController_MacAppDelegate

@synthesize window;
@synthesize configWindow;
@synthesize addressesTableView;

- (void)applicationDidFinishLaunching:(NSNotification *)aNotification {
	// Insert code here to initialize your application 
	configFile = [[ConfigFile alloc] init];
	[configFile initWithFilename:@"/Users/dko/.Barobo.config"];
	[addressesTableView setDataSource:configFile];
}

/* Menu Handlers */
- (IBAction) onMenuConfigureConfigure:(id)sender {
	/* Open the configure dialog window */
	[addressesTableView reloadData];
	[configWindow display];
	[configWindow orderFront:self];
}


@end
