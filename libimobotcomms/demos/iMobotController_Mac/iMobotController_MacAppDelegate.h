//
//  iMobotController_MacAppDelegate.h
//  iMobotController_Mac
//
//  Created by dko on 2/28/12.
//  Copyright 2012 __MyCompanyName__. All rights reserved.
//

#import <Cocoa/Cocoa.h>
#import "ConfigFile.h"

@interface iMobotController_MacAppDelegate : NSObject <NSApplicationDelegate> {
    NSWindow *window;
	NSWindow *configWindow;
	NSTableView *addressesTableView;
	NSTextField *addAddressTextField;
	
	ConfigFile *configFile;
}

@property (assign) IBOutlet NSWindow *window;
@property (retain) IBOutlet NSWindow *configWindow;
@property (retain) IBOutlet NSTableView *addressesTableView;
@property (retain) IBOutlet NSTextField *addAddressTextField;

/* Menu Handlers */
- (IBAction) onMenuConfigureConfigure:(id)sender;

/* Config dialog handlers */
- (IBAction) onButtonConfigOK:(id)sender;
- (IBAction) onButtonConfigAdd:(id)sender;
- (IBAction) onButtonConfigRemove:(id)sender;
- (IBAction) onButtonMoveDown:(id)sender;
- (IBAction) onButtonMoveUp:(id)sender;

@end
