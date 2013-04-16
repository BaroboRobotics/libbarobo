#ifndef _COMMANDS_H_
#define _COMMANDS_H_

/* NOTES:
 * - All multibyte data are sent MSB first 
 * - All command messages follow this general format:
 *   [1 byte command] [1 byte message size] [optional bytes message data] [0x00]
 *   The message size includes the command byte and the 0x00 byte.
 * - All motor id's are zero-based indices
 *   */

/* General messages */
#define MSG_SENDEND 0x00

#define MSG_ESCAPE 0xAA

/* Response Messages */
#define RESP_OK 0x10
#define RESP_END 0x11
#define RESP_ERR 0xff
#define RESP_ALREADY_PAIRED 0xfe

/* Button Callback Messages */
/* Command format:
 * [CMD] [0x0A] [4 byte timestamp millis] [1 byte events triggered mask] [1 byte buttons down mask] [1 byte buttons up mask] [0x00]
 * No Response Expected */
#define EVENT_BUTTON 0x20

/* Callback message for a remote Mobot reporting its address and serial number */
/* Format:
   [CMD] [0x09] [zigbee addr high] [zigbee addr low] [4 bytes id] [0x00] */
#define EVENT_REPORTADDRESS 0x21

#define CMD_START 0x30
#define CMD_IDLE  0x01

#define BTCMD(cmd) ((cmd) + CMD_START)

enum protocol_commands_e {
/* 0x30 CMD_STATUS: Get the Mobot's Status [ 0 ]
 * Command format: [CMD] [0x03] [0x00]
 * Expected response: [0x10] [0x03] [0x11]*/
  CMD_STATUS,

/* 0x31 CMD_DEMO: Begin the built-in demo routine. [ 1 ]
 * Command format: [CMD] [0x03] [0x00]
 * Expected response: [0x10] [0x03] [0x11]*/
  CMD_DEMO,

/* 0x32 CMD_SETMOTORDIR: Set the motor's direction. [ 2 ]
 * Command format: [CMD] [0x05] [motor_id: 1 byte] [direction: 1 byte] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_SETMOTORDIR,

/* 0x33 CMD_GETMOTORDIR: Get the motor's direction. [3]
 * Command format: [CMD] [0x04] [motor_id: 1 byte] [0x00]
 * Expected Response: [0x10] [0x04] [motor_dir: 1 byte] [0x11] */
  CMD_GETMOTORDIR,

/* 0x34 CMD_SETMOTORSPEED: Set the motor's speed. [4]
 * Command format: [CMD] [0x08] [motor_id: 1 byte] [motor_speed: 4 byte float] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_SETMOTORSPEED,

/* 0x35 CMD_GETMOTORSPEED: Get the motor's speed. [5]
 * Command format: [CMD] [0x04] [motor_id: 1 byte] [0x00]
 * Expected Response: [0x10] [0x07] [motor speed: 4 byte float] [0x11] */
  CMD_GETMOTORSPEED,

/* 0x36 CMD_SETMOTORANGLES: Set the motor joint angles. [6]
 * Command format: [CMD] [0x13] [0x????????] [0x????????] [0x????????] [0x????????]  [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_SETMOTORANGLES,

/* 0x37 CMD_SETMOTORANGLES: Set the motor joint angles. [7]
 * Command format: [CMD] [0x13] [0x????????] [0x????????] [0x????????] [0x????????]  [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_SETMOTORANGLESABS,

/* 0x38 CMD_SETMOTORANGLESDIRECT: Set the motor joint angles. Motors explicitely [8]
 * move in whatever direction is closest to the motor goal position. 
 * Command format: [CMD] [0x13] [0x????????] [0x????????] [0x????????] [0x????????]  [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_SETMOTORANGLESDIRECT,

/* 0x39 CMD_SETMOTORANGLESPID: Set the motor joint angles. [9]
 * Command format: [CMD] [0x13] [0x????????] [0x????????] [0x????????] [0x????????]  [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_SETMOTORANGLESPID,

/* 0x3A CMD_GETMOTORANGLES: Get the motor joint angles. [10]
 * Command format: [CMD] [0x03] [0x00]
 * Expected Response: [0x10] [0x13] [16 bytes of 4 float values] [0x11] */
  CMD_GETMOTORANGLES,

/* 0x3B CMD_GETMOTORANGLESABS: Get the motor joint angles. Angles are not [11]
 * normalized, so multi-rotational angles may be returned. 
 * Command format: [CMD] [0x03] [0x00]
 * Expected Response: [0x10] [0x13] [16 bytes of 4 float values] [0x11] */
  CMD_GETMOTORANGLESABS,

/* 0x3C CMD_GETMOTORANGLETIMESTAMP: Get the motor joint angles with a timestamp. [12]
 * Command format: [CMD] [0x03] [0x00]
 * Expected Response: [0x10] [0x17] [4 byte timestamp] [16 bytes motor data] [0x11] */
  CMD_GETMOTORANGLESTIMESTAMP,

/* 0x3D CMD_GETMOTORANGLETIMESTAMPABS: Get the motor joint angles with a timestamp. [13]
 * Command format: [CMD] [0x03] [0x00]
 * Expected Response: [0x10] [0x17] [4 byte timestamp] [16 bytes motor data] [0x11] */
  CMD_GETMOTORANGLESTIMESTAMPABS,

/* 0x3E CMD_SETMOTORANGLE: set a motor joint angle. [14]
 * Command format: [CMD] [0x08] [1 byte motor ID] [4 byte float angle] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_SETMOTORANGLE,

/* 0x3F CMD_SETMOTORANGLEABS: set a motor joint angle. [15]
 * Command format: [CMD] [0x08] [1 byte motor ID] [4 byte float angle] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_SETMOTORANGLEABS,

/* 0x40 CMD_SETMOTORANGLEDIRECT: set a motor joint angle. [16]
 * Command format: [CMD] [0x08] [1 byte motor ID] [4 byte float angle] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_SETMOTORANGLEDIRECT,

/* 0x41 CMD_SETMOTORANGLEPID: set a motor joint angle. [17]
 * Command format: [CMD] [0x08] [1 byte motor ID] [4 byte float angle] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_SETMOTORANGLEPID,

/* 0x42 CMD_GETMOTORANGLE: get a motor joint angle. [18]
 * Command format: [CMD] [0x04] [1 byte motor ID] [0x00]
 * Expected Response: [0x10] [0x07] [4 byte float angle] [0x11] */
  CMD_GETMOTORANGLE,

/* 0x43 CMD_GETMOTORANGLEABS: get a motor joint angle. [19]
 * Command format: [CMD] [0x04] [1 byte motor ID] [0x00]
 * Expected Response: [0x10] [0x07] [4 byte float angle] [0x11] */
  CMD_GETMOTORANGLEABS,

/* 0x44 CMD_GETMOTORANGLETIMESTAMP: get a motor joint angle with a timestamp. [20]
 * Command format: [CMD] [0x04] [1 byte motor ID] [0x00]
 * Expected Response: [0x10] [0x0b] [4 byte timestamp] [4 byte float angle] [0x11] */
  CMD_GETMOTORANGLETIMESTAMP,

/* 0x45 CMD_GETMOTORSTATE: get a motor's current state. [21]
 * Command format: [CMD] [0x04] [1 byte motor ID] [0x00]
 * Expected Response: [0x10] [0x04] [1 byte motor state] [0x11] 
 * Motor State: 0 - Idle
 *              1 - Goal Seeking
 *              2 - Moving Backward?
 *              3 - Goal Attained, holding
 *              */
  CMD_GETMOTORSTATE,

/* 0x46 CMD_GETMOTORMAXSPEED: Gets a motor's maximum allowed speed setting  [22]
 * Command Format: [CMD] [0x04] [1 byte motor ID] [0x00]
 * Expected Response: [0x10] [0x07] [4 bytes float radian value] [0x11]
 * */
  CMD_GETMOTORMAXSPEED,
  
/* 0x47 CMD_GETENCODERVOLTAGE: get a motor's current state. [23]
 * Command format: [CMD] [0x04] [1 byte encoder-pin ID] [0x00]
 * Expected Response: [0x10] [0x07] [4 byte float voltage] [0x11] */
  CMD_GETENCODERVOLTAGE,

/* 0x48 CMD_GETBUTTONVOLTAGE: get a motor's current state. [24]
 * Command format: [CMD] [0x03] [0x00]
 * Expected Response: [0x10] [0x07] [4 byte float voltage] [0x11] */
  CMD_GETBUTTONVOLTAGE,

/* 0x49 CMD_GETMOTORSAFETYLIMIT: get a motor's safety angle limit. [25]
 * Command format: [CMD] [0x03] [0x00]
 * Expected Response: [0x10] [0x07] [4 byte float voltage] [0x11] */
  CMD_GETMOTORSAFETYLIMIT,

/* 0x4A CMD_SETMOTORSAFETYLIMIT: set a motor's safety angle limit. [26]
 * Command format: [CMD] [0x08] [4 byte float angle] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_SETMOTORSAFETYLIMIT,

/* 0x4B CMD_GETMOTORSAFETYTIMEOUT: get a motor's safety angle timeout duration. [27]
 * Command format: [CMD] [0x04] [0x00]
 * Expected Response: [0x10] [0x07] [4 byte float] [0x11] */
  CMD_GETMOTORSAFETYTIMEOUT,

/* 0x4C CMD_SETMOTORSAFETYTIMEOUT: set a motor's safety angle timeout duration. [28]
 * Command format: [CMD] [0x08] [4 byte float] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_SETMOTORSAFETYTIMEOUT,

/* 0x4D CMD_STOP: Emergency stop all motors. [29]
 * Command format: [CMD] [0x03] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_STOP,

/* 0x4E CMD_GETVERSION: Gets the protocol version. Currently, this value is just CMD_NUMCOMMANDS. [30]
 * Command format: [CMD] [0x03] [0x00]
 * Expected Response: [0x10] [0x04] [1byte version] [0x11] */
  CMD_GETVERSION,

/* 0x4F CMD_BLINKLED: Blink the LED with a certain millisecond delay a certain [31]
 * number of times 
 * Command format: [CMD] [0x08] [4 byte unsigned long millisecond delay] [1 byte number of blinks] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_BLINKLED,

/* 0x50 CMD_ENABLEBUTTONHANDLER: Disable the default button handler routines and [32]
 * instead send button events to the server host. 
 * Command Format: [CMD] [0x04] [1 byte true/false] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_ENABLEBUTTONHANDLER,

/* 0x51 CMD_RESETABSCOUNTER: Reset the counter which keeps track of [33]
 * multiple-rotation angles. 
 * Command Format: [CMD] [0x03] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_RESETABSCOUNTER,

/* 0x52 CMD_GETHWREV: Get the hardware revision number [34]
 * Command Format: [CMD] [0x03] [0x00]
 * Expected Response: [0x10] [0x04] [1 byte version] [0x11] */
  CMD_GETHWREV,

/* 0x53 CMD_SETHWREV: Set the hardware revision number [35]
 * Command Format: [CMD] [0x04] [1 byte Rev number] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_SETHWREV,

/* 0x54 CMD_TIMEDACTION: Sets up a timed action on a number of motors. Each timed [36]
 * action comes as a pair: The action to begin immediately, and the motor state
 * to enable once the action is finished. 
 * Command Format: [CMD] [1byte size] [1byte bitmask] [6 bytes per enabled bitmask] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] 
 * The bitmask represents which motors to set up the new timeout. 1<<0
 * represents the first motor, 1<<1 the second, etc. For each motor, we expect
 * six bytes; the beginning and ending motor modes (1 byte each), and a timeout
 * in milliseconds (4 byte unsigned int)  */
  CMD_TIMEDACTION,

/* 0x55 CMD_GETBIGSTATE: Gets a wide variety of encoder and motor data. The data [37]
 * retrieved includes a time stamp, 4 motor angles, and 4 motor states 
 * Command Format: [CMD] [1byte size] [0x00]
 * Expected Response: [0x10] [27] [4 byte timestamp] [4x4 bytes motor angles] [1x4 bytes motor states] [0x11]
 * */
  CMD_GETBIGSTATE,

/* 0x56 CMD_SETFOURIERCOEFS: Set the coefficients for the fourier series. [38]
 * Command Format: [CMD] [1byte size] [1byte motor id] [5 x 1-byte a-coefficients] [5 x 1-byte b-coefficients] [0x00]
 * Expected Response: [0x10] [0x03] [0x11]
 * */
  CMD_SETFOURIERCOEFS,

/* 0x57 CMD_STARTFOURIER: Start the fourier based control of the Mobot. [39]
 * Command Format: [CMD] [0x04] [1 byte motor mask] [0x00]
 * Expected Response: [0x10] [0x03] [0x11]
 * */
  CMD_STARTFOURIER,

/* 0x58 CMD_LOADMELODY: Transfer a melody into EEPROM  [40]
 * Command Format: [CMD] [1byte size] [1 byte Melody slot (0-3)] [data] [0x00]
 * Expected Response: [0x10] [0x03] [0x11]
 */
  CMD_LOADMELODY,

/* 0x59 CMD_PLAYMELODY: Play a preloaded melody  [41]
 * Command Format: [CMD] [0x04] [1 byte melody slot] [0x00]
 * Expected Response: [0x10] [0x03] [0x11]
 */
  CMD_PLAYMELODY,

/* 0x5A CMD_GETADDRESS: Request the zigbee address of this robot [42]
   Command Format: [CMD] [0x03] [0x00]
   Expected Response: [0x10] [0x05] [2 bytes address] [0x11]
 */
  CMD_GETADDRESS,

/* 0x5B CMD_QUERYADDRESSES: Send a broadcast message out asking all devices within [43]
 * range for their zigbee addresses.
 * Command Format: [CMD] [0x03] [0x00]
 * Expected Response: [0x10] [0x03] [0x11]
 */
  CMD_QUERYADDRESSES,

/* 0x5C CMD_GETQUERIEDADDRESSES: Get a list of all addresses that have been reported so far. [44]
 * Command Format: [CMD] [0x03] [0x00]
 * Expected Response: [0x10] [message size] [list of 6 byte identifiers: 2-byte addresses, high bytes first, 4 byte serial number] [0x11]
 */
  CMD_GETQUERIEDADDRESSES,

/* 0x5D CMD_CLEARQUERIEDADDRESSES: Clear the list of queried addresses. [45]
 * Command Format: [CMD] [0x03] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_CLEARQUERIEDADDRESSES,

/* 0x5E CMD_REQUESTADDRESS: Request another Mobot to send a CMD_REPORTADDRESS to you [46]
 * as a separate command ASAP
 * Command Format: [CMD] [0x03] [0x00]
 * Expected Response: None */
  CMD_REQUESTADDRESS,

/* 0x5F CMD_REPORTADDRESS: Send your address info to another Mobot. Typically, [47]
 * this message is sent after receiving a CMD_REQUESTADDRESS command from another Mobot.
 * Command Format: [CMD] [0x09] [Address high byte] [Address low byte] [4 bytes id] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_REPORTADDRESS,

/* 0x60 CMD_REBOOT: Make the robot reboot  [48]
   Command Format: [CMD] [0x03] [0x00]
   Expected Response: None
   */
  CMD_REBOOT, 

/* 0x61 CMD_REQUESTSERIALID: Request the serial number of a remote robot [49]
   Command Format: [CMD] [0x03] [0x11] 
   Expected Response: [0x10] [0x07] [4 bytes serial number] [0x00]
   */
  CMD_GETSERIALID,

/* 0x62 CMD_SETSERIALID: Set the serial number for a Mobot [50]
   Command Format: [CMD] [0x07] [4 byte id] [0x00]
   Expected Response: [0x10] [0x03] [0x11] */
  CMD_SETSERIALID,

/* 0x63 CMD_SETRFCHANNEL: Set the Zigbee RF Channel. Valid values are between 0x0b and 0x1a  [51]
   Command Format: [CMD] [0x04] [1 byte channel] [0x00]
   Expected Response: [0x10] [0x03] [0x11] 
   Note: The response is sent _after_ the channel change */
  CMD_SETRFCHANNEL,

/* 0x64 CMD_FINDMOBOT: This is a broadcasted command searching for a specific Mobot [52]
 * of a certain serial address. If a Mobot receives this message and finds a
 * match on its serial address, it will respond with a REPORTADDRESS command in
 * an appropriate amount of time. 
 * Command Format: [CMD] [0x07] [4 byte serial id] [0x00]
 * Expected Response: None */
  CMD_FINDMOBOT,

/* 0x65 CMD_PAIRPARENT: After receiving this message, the receiving Mobot should [53]
 * consider itself paired to the requesting parent. All button events, etc.
 * should be sent to the parent, and further pairing requests should be
 * ignored.
 * Command Format: [CMD] [0x05] [parent address high byte] [parent address low byte] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_PAIRPARENT,

/* 0x66 CMD_UNPAIRPARENT: After receiving this message, the Mobot should consider [54]
 * itself unpaired.
 * Command Format: [CMD] [0x03] [0x00]
 * Expected Response: [0x10] [0x03] 0x11] */
  CMD_UNPAIRPARENT,

/* 0x67 CMD_RGBLED: Modify the RGBLED colors, flashing, etc. [55]
 * Command Format: [CMD] [0x08] [3 bytes mask 0xrrggbb] [3 bytes values] [0x11]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_RGBLED,

/* 0x68 CMD_SETMOTORPOWER: Set the motor at a certain power [56]
   Command Format: [CMD] [0x0A] [1 byte mask] [3x2 byte int16_t PWM values] [0x11]
   Expected Response: [0x10] [0x03] [0x11] */
  CMD_SETMOTORPOWER,

/* 0x69 CMD_GETBATTERYVOLTAGE: Get the battery voltage. [57]
   Command Format: [CMD] [0x03] [0x00]
   Expected Response: [0x10] [0x07] [4 byte float] [0x11] */
  CMD_GETBATTERYVOLTAGE,

/* 0x6A CMD_BUZZERFREQ: Play a frequency on the buzzer. If the frequency given is 0, [58]
 * stop the buzzer.
 * Command Format: [CMD] [0x05] [2 bytes uint16 frequency, msb first] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_BUZZERFREQ,

/* 0x6B CMD_GETACCEL: Get the accelerometer data. [59]
 * Command Format: [CMD] [0x03] [0x00]
 * Expected Response: [0x10] [0x09] [3*2 bytes float data, x,y,z] [0x11] */
  CMD_GETACCEL,

/* 0x6C CMD_GETFORMFACTOR: Get the mobot's form factor. [60]
 * Command Format: [CMD] [0x03] [0x00]
 * Expected Response: [0x10] [0x04] [1 byte identifier] [0x11] */
  CMD_GETFORMFACTOR,

/* 0x6D CMD_GETRGB: Get the current RGB values for the LED [61]
   Command Format: [CMD] [0x03] [0x00]
   Expected Response: [0x10] [0x06] [1 byte r] [1 byte g] [1 byte b] [0x11]
   */
  CMD_GETRGB,

/* 0x6E Placeholder to increase version number [62] */
  CMD_PLACEHOLDER201303291416,

/* 0x6F Placeholder to increase version number [63] */
  CMD_PLACEHOLDER201304121823,

/* 0x70 Placeholder to increase version number [64] */
  CMD_PLACEHOLDER201304152311,

  CMD_NUMCOMMANDS
};

#endif
