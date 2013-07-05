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

/* When the Linkbot sends TWI commands to the breakout board, there are 2
 * general types of messages. There are messages that are responses to commands
 * that need to be forwarded to the Bluetooth module, but there are also direct
 * register acces commands. The MSG_REGACCESS command is used to set/get
 * register/memory values on a slave breakoutboard.
   Format:
   send([0x22] [reg addr] [maybe write bytes]) + maybe_recv([data])
 */
#define MSG_REGACCESS 0x22

/* General message to cause a debugging message to be printed on the PC side 
Format:
  [0x23] [size] [message] [0x00]
  */
#define EVENT_DEBUG_MSG 0x23

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

/* 0x71 Placeholder to increase version number [65] */
  CMD_PLACEHOLDER201304161605,

/* 0x72 Placeholder to increase version number [66] */
  CMD_PLACEHOLDER201304181705,

/* 0x73 Placeholder to increase version number [67] */
  CMD_PLACEHOLDER201304181425,

/* 0x74 CMD_SET_GRP_MASTER: If the robot is currently a slave in a group, it [68]
 * becomes a master.
 * Command Format: [CMD] [0x03] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_SET_GRP_MASTER,

/* 0x75 CMD_SET_GRP_SLAVE: Sets the robot to be a group slave [69]
 * Command Format: [CMD] [0x03] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_SET_GRP_SLAVE,

/* 0x76 CMD_SET_GRP: Sets the robot to be in a group. If the robot is not [70]
 * currently in a group, it defaults to being a slave in the group.
 * Command Format: [CMD] [size] [2 byte group id msb first] [3 byte rgb] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_SET_GRP,

/* 0x77 CMD_SAVE_POSE: Save the current positions to an internal pose. [71]
   Command Format: [CMD] [4] [uint8_t pose index] [0x00]
   Expected Response: [0x10] [0x03] [0x11] */
  CMD_SAVE_POSE,

/* 0x78 CMD_MOVE_TO_POSE: Move the motors to an indexed pose [72]
   Command Format: [CMD] [size] [uint8_t pose index] [0x00]
   Expected Response: [0x10] [0x03] [0x11] */
  CMD_MOVE_TO_POSE,

/* 0x79 CMD_IS_MOVING: Is the robot moving? [73]
   Command Format: [CMD] [0x03] [0x00]
   Expected Response: [0x10] [0x04] [1 byte truth value] [0x11] */
  CMD_IS_MOVING,

/* 0x7A CMD_GET_MOTOR_ERRORS: How far are the motors away from their goal? [74]
   Command Format: [CMD] [3] [0x00]
   Expected Response: [0x10] [19] [4x4 byte floats] [0x00] */
  CMD_GET_MOTOR_ERRORS,

/* 0x7B CMD_MOVE_MOTORS: Move motors from their current positions [75]
   Command Format: [CMD] [19] [4*4 bytes floats, motor displacement angles] [0x00]
   Expected Response: [0x10] [0x03] [0x11] */
  CMD_MOVE_MOTORS,

/* 0x7C CMD_TWI_SEND: Send data to a TWI slave [76]
   Command Format: [CMD] [size] [TWI addr] [data send size] [data] [0x00]
   Expected Response: [0x10] [0x03] [0x11] */
  CMD_TWI_SEND,

/* 0x7D CMD_TWI_RECV: Receive data from a TWI slave [77]
   Command Format: [CMD] [size] [TWI addr] [recv size] [0x00]
   Expected Response: [0x10] [size] [data] [0x11] */
  CMD_TWI_RECV,

/* 0x7E CMD_TWI_SENDRECV: Send and receive data from a TWI slave [78]
   Command Format: [CMD] [size] [TWI addr] [send size] [send data] [recv size] [0x00]
   Expected Response: [0x10] [size] [data] [0x11] */
  CMD_TWI_SENDRECV,

/* 0x7F CMD_SET_ACCEL: Set a joint to move at a constant acceleration. Stops
 * accelerating at specified speed or the max speed. (all units in rads/sec)
 * Command Format: [CMD] [size] [1 byte id] [4 byte accel] [4 byte max speed] [4 bytes timeout millis MSB] [0x00] */
  CMD_SET_ACCEL,

/* 0x80 CMD_SMOOTHMOVE: move a joint smoothly from its current position to a new position.
   Command format: 
      [CMD] 
      [size] 
      [1 byte id] 
      [4 byte Accel0] 
      [4 byte Accelf] 
      [4 byte vmax] 
      [4 byte joint angle] 
      [0x00]
      */
  CMD_SMOOTHMOVE,


  CMD_NUMCOMMANDS
};

#define GRP_CMD_START 0xD0
#define GRP_CMD_END 0x12
#define GRPCMD(cmd) ((cmd) + GRP_CMD_START)
enum group_protocol_commands_e {
  /* 0xD0 The first command is a general wrapper containing a normal CMD_ within.
     Command Format: 
        [CMD]
        [size] 
        [group ID high byte] 
        [" " low byte] 
        [1 byte response requested]
        [X bytes CMD_] 
        [0x12]
     */
  GRP_CMD_WRAPPER,

  /* 0xD1 GRP_CMD_RESP_WRAPPER: If a group member receives a GRP_CMD_WRAPPER
   * addressed specifically to it, it will send a wrappred response back to the
   * sender.
   * Command Format: [CMD] [size] [group ID high] [group ID low] [X bytes response] [0x12] */
  GRP_CMD_RESP_WRAPPER,

  /* 0xD2 GRP_CMD_SYNCSHOCK_ARBITRATE_MASTER: Synchronize two unpaired modules by using shock detection.
     Command Format: 
          0 [CMD] 
          1 [size] 
          2 [4 bytes millis passed since shock] 
          6 [4 bytes millis passed since button down] 
          10 [2 byte group id] 
          12 [3 bytes RGB] 
          [0x12]
     */
  GRP_CMD_SYNCSHOCK_ARBITRATE_MASTER,

  /* 0xD3 GRP_CMD_SYNCSHOCK_SET_SLAVE: Synchronize a paired and unpaired
   * master, setting the unpaired robot to be a slave.
   * Command Format: [CMD] [size] [4 bytes millis passed since shock] [2 byte group id] [3 byte RGB] [0x12]
   */
  GRP_CMD_SYNCSHOCK_SET_SLAVE,

/* 0xD4 CMD_SET_GRP_MASTER: If the robot is currently a slave in a group, it
 * becomes a master.
 * Command Format: [CMD] [0x03] [0x12]
 * Expected Response: None */
  GRP_CMD_SET_GRP_MASTER,

/* 0xD5 CMD_SET_GRP_SLAVE: Sets the robot to be a group slave
 * Command Format: [CMD] [0x03] [0x12]
 * Expected Response: None */
  GRP_CMD_SET_GRP_SLAVE,

/* 0xD6 GRP_CMD_SLAVE_BEACON: All new slaves will broadcast this beacon so that
 * the master can add them to the list of known slaves.
 * Command Format: [CMD] [0x05] [group_id high byte] [group_id low byte] [0x12]
 * Expected Response: None */
  GRP_CMD_SLAVE_BEACON,

/* 0xD7 GRP_CMD_PLAY_POSES: Causes the group master to begin coordinated playing of poses.
   Command Format: [CMD] [0x05] [group higb byte] [group low byte] [0x12]
   Expected Response: None */
  GRP_CMD_PLAY_POSES,

  GRP_CMD_NUMCOMMANDS
};

#endif
