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
	
	ConfigFile *configFile;
}

@property (assign) IBOutlet NSWindow *window;
@property (assign) IBOutlet NSWindow *configWindow;
@property (assign) IBOutlet NSTableView *addressesTableView;

/* Menu Handlers */
- (IBAction) onMenuConfigureConfigure:(id)sender;

@end
