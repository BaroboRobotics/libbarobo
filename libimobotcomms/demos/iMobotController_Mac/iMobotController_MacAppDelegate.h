//
//  iMobotController_MacAppDelegate.h
//  iMobotController_Mac
//
//  Created by dko on 2/28/12.
//  Copyright 2012 __MyCompanyName__. All rights reserved.
//

#import <Cocoa/Cocoa.h>
#import "ConfigFile.h"
#import "UIHandler.h"

@interface iMobotController_MacAppDelegate : NSObject <NSApplicationDelegate> {
    NSWindow *window;
	NSWindow *configWindow;
	NSWindow *connectFailedWindow;
	NSTableView *addressesTableView;
	NSTextField *addAddressTextField;
	
	ConfigFile *configFile;
	UIHandler *uiHandler;
	
	br_comms_t *comms;
}

@property (assign) IBOutlet NSWindow *window;
@property (retain) IBOutlet NSWindow *configWindow;
@property (retain) IBOutlet NSWindow *connectFailedWindow;
@property (retain) IBOutlet NSTableView *addressesTableView;
@property (retain) IBOutlet NSTextField *addAddressTextField;
@property (retain) IBOutlet UIHandler *uiHandler;

/* Menu Handlers */
- (IBAction) onMenuConfigureConfigure:(id)sender;
- (IBAction) onMenuConnectConnect:(id)sender;
- (IBAction) onMenuConnectDisconnect:(id)sender;
- (IBAction) onMenuHelpHelp:(id)sender;

/* Config dialog handlers */
- (IBAction) onButtonConfigOK:(id)sender;
- (IBAction) onButtonConfigAdd:(id)sender;
- (IBAction) onButtonConfigRemove:(id)sender;
- (IBAction) onButtonMoveDown:(id)sender;
- (IBAction) onButtonMoveUp:(id)sender;

@end
