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

/* Response Messages */
#define RESP_OK 0x10
#define RESP_END 0x11
#define RESP_ERR 0xff

/* Button Callback Messages */
/* Command format:
 * [CMD] [0x0A] [4 byte timestamp millis] [1 byte events triggered mask] [1 byte buttons down mask] [1 byte buttons up mask] [0x00]
 * No Response Expected */
#define EVENT_BUTTON 0x20

/* Callback message for a remote Mobot reporting its address and serial number */
#define EVENT_REPORTADDRESS 0x21

#define CMD_START 0x30
#define CMD_IDLE  0x01

#define BTCMD(cmd) ((cmd) + CMD_START)

enum protocol_commands_e {
/* CMD_STATUS: Get the Mobot's Status [ 0 ]
 * Command format: [CMD] [0x03] [0x00]
 * Expected response: [0x10] [0x03] [0x11]*/
  CMD_STATUS,

/* CMD_DEMO: Begin the built-in demo routine. [ 1 ]
 * Command format: [CMD] [0x03] [0x00]
 * Expected response: [0x10] [0x03] [0x11]*/
  CMD_DEMO,

/* CMD_SETMOTORDIR: Set the motor's direction. [ 2 ]
 * Command format: [CMD] [0x05] [motor_id: 1 byte] [direction: 1 byte] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_SETMOTORDIR,

/* CMD_GETMOTORDIR: Get the motor's direction. [3]
 * Command format: [CMD] [0x04] [motor_id: 1 byte] [0x00]
 * Expected Response: [0x10] [0x04] [motor_dir: 1 byte] [0x11] */
  CMD_GETMOTORDIR,

/* CMD_SETMOTORSPEED: Set the motor's speed. [4]
 * Command format: [CMD] [0x08] [motor_id: 1 byte] [motor_speed: 4 byte float] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_SETMOTORSPEED,

/* CMD_GETMOTORSPEED: Get the motor's speed. [5]
 * Command format: [CMD] [0x04] [motor_id: 1 byte] [0x00]
 * Expected Response: [0x10] [0x07] [motor speed: 4 byte float] [0x11] */
  CMD_GETMOTORSPEED,

/* CMD_SETMOTORANGLES: Set the motor joint angles. [6]
 * Command format: [CMD] [0x13] [0x????????] [0x????????] [0x????????] [0x????????]  [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_SETMOTORANGLES,

/* CMD_SETMOTORANGLES: Set the motor joint angles. [7]
 * Command format: [CMD] [0x13] [0x????????] [0x????????] [0x????????] [0x????????]  [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_SETMOTORANGLESABS,

/* CMD_SETMOTORANGLESDIRECT: Set the motor joint angles. Motors explicitely [8]
 * move in whatever direction is closest to the motor goal position. 
 * Command format: [CMD] [0x13] [0x????????] [0x????????] [0x????????] [0x????????]  [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_SETMOTORANGLESDIRECT,

/* CMD_SETMOTORANGLESPID: Set the motor joint angles. [9]
 * Command format: [CMD] [0x13] [0x????????] [0x????????] [0x????????] [0x????????]  [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_SETMOTORANGLESPID,

/* CMD_GETMOTORANGLES: Get the motor joint angles. [10]
 * Command format: [CMD] [0x03] [0x00]
 * Expected Response: [0x10] [0x13] [16 bytes of 4 float values] [0x11] */
  CMD_GETMOTORANGLES,

/* CMD_GETMOTORANGLESABS: Get the motor joint angles. Angles are not [11]
 * normalized, so multi-rotational angles may be returned. 
 * Command format: [CMD] [0x03] [0x00]
 * Expected Response: [0x10] [0x13] [16 bytes of 4 float values] [0x11] */
  CMD_GETMOTORANGLESABS,

/* CMD_GETMOTORANGLETIMESTAMP: Get the motor joint angles with a timestamp. [12]
 * Command format: [CMD] [0x03] [0x00]
 * Expected Response: [0x10] [0x17] [4 byte timestamp] [16 bytes motor data] [0x11] */
  CMD_GETMOTORANGLESTIMESTAMP,

/* CMD_GETMOTORANGLETIMESTAMPABS: Get the motor joint angles with a timestamp. [13]
 * Command format: [CMD] [0x03] [0x00]
 * Expected Response: [0x10] [0x17] [4 byte timestamp] [16 bytes motor data] [0x11] */
  CMD_GETMOTORANGLESTIMESTAMPABS,

/* CMD_SETMOTORANGLE: set a motor joint angle. [14]
 * Command format: [CMD] [0x08] [1 byte motor ID] [4 byte float angle] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_SETMOTORANGLE,

/* CMD_SETMOTORANGLEABS: set a motor joint angle. [15]
 * Command format: [CMD] [0x08] [1 byte motor ID] [4 byte float angle] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_SETMOTORANGLEABS,

/* CMD_SETMOTORANGLEDIRECT: set a motor joint angle. [16]
 * Command format: [CMD] [0x08] [1 byte motor ID] [4 byte float angle] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_SETMOTORANGLEDIRECT,

/* CMD_SETMOTORANGLEPID: set a motor joint angle. [17]
 * Command format: [CMD] [0x08] [1 byte motor ID] [4 byte float angle] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_SETMOTORANGLEPID,

/* CMD_GETMOTORANGLE: get a motor joint angle. [18]
 * Command format: [CMD] [0x04] [1 byte motor ID] [0x00]
 * Expected Response: [0x10] [0x07] [4 byte float angle] [0x11] */
  CMD_GETMOTORANGLE,

/* CMD_GETMOTORANGLEABS: get a motor joint angle. [19]
 * Command format: [CMD] [0x04] [1 byte motor ID] [0x00]
 * Expected Response: [0x10] [0x07] [4 byte float angle] [0x11] */
  CMD_GETMOTORANGLEABS,

/* CMD_GETMOTORANGLETIMESTAMP: get a motor joint angle with a timestamp. [20]
 * Command format: [CMD] [0x04] [1 byte motor ID] [0x00]
 * Expected Response: [0x10] [0x0b] [4 byte timestamp] [4 byte float angle] [0x11] */
  CMD_GETMOTORANGLETIMESTAMP,

/* CMD_GETMOTORSTATE: get a motor's current state. [21]
 * Command format: [CMD] [0x04] [1 byte motor ID] [0x00]
 * Expected Response: [0x10] [0x04] [1 byte motor state] [0x11] 
 * Motor State: 0 - Idle
 *              1 - Goal Seeking
 *              2 - Moving Backward?
 *              3 - Goal Attained, holding
 *              */
  CMD_GETMOTORSTATE,

/* CMD_GETMOTORMAXSPEED: Gets a motor's maximum allowed speed setting  [22]
 * Command Format: [CMD] [0x04] [1 byte motor ID] [0x00]
 * Expected Response: [0x10] [0x07] [4 bytes float radian value] [0x11]
 * */
  CMD_GETMOTORMAXSPEED,
  
/* CMD_GETENCODERVOLTAGE: get a motor's current state. [23]
 * Command format: [CMD] [0x04] [1 byte encoder-pin ID] [0x00]
 * Expected Response: [0x10] [0x07] [4 byte float voltage] [0x11] */
  CMD_GETENCODERVOLTAGE,

/* CMD_GETBUTTONVOLTAGE: get a motor's current state. [24]
 * Command format: [CMD] [0x03] [0x00]
 * Expected Response: [0x10] [0x07] [4 byte float voltage] [0x11] */
  CMD_GETBUTTONVOLTAGE,

/* CMD_GETMOTORSAFETYLIMIT: get a motor's safety angle limit. [25]
 * Command format: [CMD] [0x03] [0x00]
 * Expected Response: [0x10] [0x07] [4 byte float voltage] [0x11] */
  CMD_GETMOTORSAFETYLIMIT,

/* CMD_SETMOTORSAFETYLIMIT: set a motor's safety angle limit. [26]
 * Command format: [CMD] [0x08] [4 byte float angle] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_SETMOTORSAFETYLIMIT,

/* CMD_GETMOTORSAFETYTIMEOUT: get a motor's safety angle timeout duration. [27]
 * Command format: [CMD] [0x04] [0x00]
 * Expected Response: [0x10] [0x07] [4 byte float] [0x11] */
  CMD_GETMOTORSAFETYTIMEOUT,

/* CMD_SETMOTORSAFETYTIMEOUT: set a motor's safety angle timeout duration. [28]
 * Command format: [CMD] [0x08] [4 byte float] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_SETMOTORSAFETYTIMEOUT,

/* CMD_STOP: Emergency stop all motors. [29]
 * Command format: [CMD] [0x03] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_STOP,

/* CMD_GETVERSION: Gets the protocol version. Currently, this value is just CMD_NUMCOMMANDS. [30]
 * Command format: [CMD] [0x03] [0x00]
 * Expected Response: [0x10] [0x04] [1byte version] [0x11] */
  CMD_GETVERSION,

/* CMD_BLINKLED: Blink the LED with a certain millisecond delay a certain [31]
 * number of times 
 * Command format: [CMD] [0x08] [4 byte unsigned long millisecond delay] [1 byte number of blinks] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_BLINKLED,

/* CMD_ENABLEBUTTONHANDLER: Disable the default button handler routines and [32]
 * instead send button events to the server host. 
 * Command Format: [CMD] [0x04] [1 byte true/false] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_ENABLEBUTTONHANDLER,

/* CMD_RESETABSCOUNTER: Reset the counter which keeps track of [33]
 * multiple-rotation angles. 
 * Command Format: [CMD] [0x03] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_RESETABSCOUNTER,

/* CMD_GETHWREV: Get the hardware revision number [34]
 * Command Format: [CMD] [0x03] [0x00]
 * Expected Response: [0x10] [0x04] [1 byte version] [0x11] */
  CMD_GETHWREV,

/* CMD_SETHWREV: Set the hardware revision number [35]
 * Command Format: [CMD] [0x04] [1 byte Rev number] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_SETHWREV,

/* CMD_TIMEDACTION: Sets up a timed action on a number of motors. Each timed [36]
 * action comes as a pair: The action to begin immediately, and the motor state
 * to enable once the action is finished. 
 * Command Format: [CMD] [1byte size] [1byte bitmask] [6 bytes per enabled bitmask] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] 
 * The bitmask represents which motors to set up the new timeout. 1<<0
 * represents the first motor, 1<<1 the second, etc. For each motor, we expect
 * six bytes; the beginning and ending motor modes (1 byte each), and a timeout
 * in milliseconds (4 byte unsigned int)  */
  CMD_TIMEDACTION,

/* CMD_GETBIGSTATE: Gets a wide variety of encoder and motor data. The data [37]
 * retrieved includes a time stamp, 4 motor angles, and 4 motor states 
 * Command Format: [CMD] [1byte size] [0x00]
 * Expected Response: [0x10] [27] [4 byte timestamp] [4x4 bytes motor angles] [1x4 bytes motor states] [0x11]
 * */
  CMD_GETBIGSTATE,

/* CMD_SETFOURIERCOEFS: Set the coefficients for the fourier series. [38]
 * Command Format: [CMD] [1byte size] [1byte motor id] [5 x 1-byte a-coefficients] [5 x 1-byte b-coefficients] [0x00]
 * Expected Response: [0x10] [0x03] [0x11]
 * */
  CMD_SETFOURIERCOEFS,

/* CMD_STARTFOURIER: Start the fourier based control of the Mobot. [39]
 * Command Format: [CMD] [0x04] [1 byte motor mask] [0x00]
 * Expected Response: [0x10] [0x03] [0x11]
 * */
  CMD_STARTFOURIER,

/* CMD_LOADMELODY: Transfer a melody into EEPROM  [40]
 * Command Format: [CMD] [1byte size] [1 byte Melody slot (0-3)] [data] [0x00]
 * Expected Response: [0x10] [0x03] [0x11]
 */
  CMD_LOADMELODY,

/* CMD_PLAYMELODY: Play a preloaded melody  [41]
 * Command Format: [CMD] [0x04] [1 byte melody slot] [0x00]
 * Expected Response: [0x10] [0x03] [0x11]
 */
  CMD_PLAYMELODY,

/* CMD_GETADDRESS: Request the zigbee address of this robot [42]
   Command Format: [CMD] [0x03] [0x00]
   Expected Response: [0x10] [0x05] [2 bytes address] [0x11]
 */
  CMD_GETADDRESS,

/* CMD_QUERYADDRESSES: Send a broadcast message out asking all devices within [43]
 * range for their zigbee addresses.
 * Command Format: [CMD] [0x03] [0x00]
 * Expected Response: [0x10] [0x03] [0x11]
 */
  CMD_QUERYADDRESSES,

/* CMD_GETQUERIEDADDRESSES: Get a list of all addresses that have been reported so far. [44]
 * Command Format: [CMD] [0x03] [0x00]
 * Expected Response: [0x10] [message size] [list of 6 byte identifiers: 2-byte addresses, high bytes first, 4 byte serial number] [0x11]
 */
  CMD_GETQUERIEDADDRESSES,

/* CMD_CLEARQUERIEDADDRESSES: Clear the list of queried addresses. [45]
 * Command Format: [CMD] [0x03] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_CLEARQUERIEDADDRESSES,

/* CMD_REQUESTADDRESS: Request another Mobot to send a CMD_REPORTADDRESS to you [46]
 * as a separate command ASAP
 * Command Format: [CMD] [0x03] [0x00]
 * Expected Response: None */
  CMD_REQUESTADDRESS,

/* CMD_REPORTADDRESS: Send your address info to another Mobot. Typically, [47]
 * this message is sent after receiving a CMD_REQUESTADDRESS command from another Mobot.
 * Command Format: [CMD] [0x09] [Address high byte] [Address low byte] [4 bytes id] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_REPORTADDRESS,

/* CMD_REBOOT: Make the robot reboot  [48]
   Command Format: [CMD] [0x03] [0x00]
   Expected Response: None
   */
  CMD_REBOOT, 

/* CMD_REQUESTSERIALID: Request the serial number of a remote robot [49]
   Command Format: [CMD] [0x03] [0x11] 
   Expected Response: [0x10] [0x07] [4 bytes serial number] [0x00]
   */
  CMD_GETSERIALID,

/* CMD_SETSERIALID: Set the serial number for a Mobot [50]
   Command Format: [CMD] [0x07] [4 byte id] [0x00]
   Expected Response: [0x10] [0x03] [0x11] */
  CMD_SETSERIALID,

/* CMD_SETRFCHANNEL: Set the Zigbee RF Channel. Valid values are between 0x0b and 0x1a  [51]
   Command Format: [CMD] [0x04] [1 byte channel] [0x00]
   Expected Response: [0x10] [0x03] [0x11] 
   Note: The response is sent _after_ the channel change */
  CMD_SETRFCHANNEL,

/* CMD_FINDMOBOT: This is a broadcasted command searching for a specific Mobot
 * of a certain serial address. If a Mobot receives this message and finds a
 * match on its serial address, it will respond with a REPORTADDRESS command in
 * an appropriate amount of time. 
 * Command Format: [CMD] [0x07] [4 byte serial id] [0x00]
 * Expected Response: None */
  CMD_FINDMOBOT,

/* CMD_PAIRPARENT: After receiving this message, the receiving Mobot should
 * consider itself paired to the requesting parent. All button events, etc.
 * should be sent to the parent, and further pairing requests should be
 * ignored.
 * Command Format: [CMD] [0x05] [parent address high byte] [parent address low byte] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_PAIRPARENT,

/* CMD_UNPAIRPARENT: After receiving this message, the Mobot should consider
 * itself unpaired.
 * Command Format: [CMD] [0x03] [0x00]
 * Expected Response: [0x10] [0x03] 0x11] */
  CMD_UNPAIRPARENT,

/* CMD_RGBLED: Modify the RGBLED colors, flashing, etc.
 * Command Format: [CMD] [0x08] [3 bytes mask 0xrrggbb] [3 bytes values] [0x11]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_RGBLED,

/* CMD_SETMOTORPOWER: Set the motor at a certain power
   Command Format: [CMD] [0x0A] [1 byte mask] [3x2 byte int16_t PWM values] [0x11]
   Expected Response: [0x10] [0x03] [0x11] */
  CMD_SETMOTORPOWER,

/* CMD_GETBATTERYVOLTAGE: Get the battery voltage.
   Command Format: [CMD] [0x03] [0x00]
   Expected Response: [0x10] [0x07] [4 byte float] [0x11] */
  CMD_GETBATTERYVOLTAGE,

/* CMD_BUZZERFREQ: Play a frequency on the buzzer. If the frequency given is 0,
 * stop the buzzer.
 * Command Format: [CMD] [0x05] [2 bytes uint16 frequency, msb first] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_BUZZERFREQ,

/* CMD_GETACCEL: Get the accelerometer data.
 * Command Format: [CMD] [0x03] [0x00]
 * Expected Response: [0x10] [0x09] [3*2 bytes float data, x,y,z] [0x11] */
  CMD_GETACCEL,

/* CMD_GETFORMFACTOR: Get the mobot's form factor.
 * Command Format: [CMD] [0x03] [0x00]
 * Expected Response: [0x10] [0x04] [1 byte identifier] [0x11] */
  CMD_GETFORMFACTOR,

/* CMD_GETRGB: Get the current RGB values for the LED
   Command Format: [CMD] [0x03] [0x00]
   Expected Response: [0x10] [0x06] [1 byte r] [1 byte g] [1 byte b] [0x11]
   */
  CMD_GETRGB,

  CMD_NUMCOMMANDS
};

#endif
