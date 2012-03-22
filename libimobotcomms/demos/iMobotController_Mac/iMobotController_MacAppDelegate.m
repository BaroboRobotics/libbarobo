//
//  iMobotController_MacAppDelegate.m
//  iMobotController_Mac
//
//  Created by dko on 2/28/12.
//  Copyright 2012 __MyCompanyName__. All rights reserved.
//

#import "iMobotController_MacAppDelegate.h"
#import <Foundation/NSURL.h>

@implementation iMobotController_MacAppDelegate

@synthesize window;
@synthesize configWindow;
@synthesize connectFailedWindow;
@synthesize addressesTableView;
@synthesize addAddressTextField;
@synthesize errorMessageLabel;
@synthesize uiHandler;

- (void)applicationDidFinishLaunching:(NSNotification *)aNotification {
	// Insert code here to initialize your application 
	NSString *path = @"~/.Barobo.config";
	NSString *spath = [path stringByStandardizingPath];
	configFile = [[ConfigFile alloc] initWithFilename:spath];
	[addressesTableView setDataSource:configFile];
	comms = (br_comms_t*)malloc(sizeof(br_comms_t));
	Mobot_init(comms);
}

- (void)applicationDidBecomeActive:(NSNotification *)notification {

}

/* Menu Handlers */
- (IBAction) onMenuConfigureConfigure:(id)sender {
	/* Open the configure dialog window */
	[addressesTableView reloadData];
	//[configWindow display];
	[configWindow orderFront:self];
}

- (IBAction) onMenuConnectConnect:(id)sender {
	[connectFailedWindow orderOut:sender];
	if(Mobot_connect(comms)) {
		/* Error message */
		[errorMessageLabel setStringValue:[NSString stringWithCString:strerror(errno) encoding:NSASCIIStringEncoding]];
		[connectFailedWindow orderFront:sender];
	} else {
		[uiHandler initWithBRComms:comms];
		[uiHandler start];
	}
}

- (IBAction) onMenuConnectDisconnect:(id)sender {
	[uiHandler cancel];
	while( [uiHandler isExecuting] );
	Mobot_disconnect(comms);
}

- (IBAction) onMenuHelpHelp:(id)sender {
	NSURL *myUrl = [NSURL URLWithString:@"file:///usr/local/ch/package/chmobot/docs/index.html"];
	[[NSWorkspace sharedWorkspace] openURL:myUrl];
}

- (IBAction) onMenuHelpDemos:(id)sender {
	[NSTask launchedTaskWithLaunchPath:@"/usr/local/ch/bin/chide" 
							 arguments:[NSArray arrayWithObjects:@"-d", @"/usr/local/ch/package/chmobot/demos", nil]];
}

/* Config Dialog Button Handlers */
- (IBAction) onButtonConfigOK:(id)sender {
	[configFile writeFile];
	[configWindow orderOut:self];
}

- (IBAction) onButtonConfigAdd:(id)sender {
	/* Get the text entry */
	NSString *textEntry = [addAddressTextField stringValue];
	[configFile addAddress:textEntry];
	[addressesTableView reloadData];
}

- (IBAction) onButtonConfigRemove:(id)sender {
	NSInteger rowIndex;
	rowIndex = [addressesTableView selectedRow];
	if(rowIndex < 0) {
		return;
	}
	[configFile removeAtIndex:rowIndex];
	[addressesTableView reloadData];
}

- (IBAction) onButtonMoveDown:(id)sender {
	NSInteger rowIndex;
	rowIndex = [addressesTableView selectedRow];
	if(rowIndex < 0) return;
	[configFile moveAddressDown:rowIndex];
	[addressesTableView reloadData];
}

- (IBAction) onButtonMoveUp:(id)sender {
	NSInteger rowIndex;
	rowIndex = [addressesTableView selectedRow];
	if(rowIndex < 0) return;
	[configFile moveAddressUp:rowIndex];
	[addressesTableView reloadData];
}

@end
