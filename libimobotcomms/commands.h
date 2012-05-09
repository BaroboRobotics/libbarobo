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

#define CMD_START 0x30

#define BTCMD(cmd) ((cmd) + CMD_START)

enum protocol_commands_e {
/* CMD_STATUS: Get the Mobot's Status 
 * Command format: [CMD] [0x03] [0x00]
 * Expected response: [0x10] [0x03] [0x11]*/
  CMD_STATUS,

/* CMD_DEMO: Begin the built-in demo routine.
 * Command format: [CMD] [0x03] [0x00]
 * Expected response: [0x10] [0x03] [0x11]*/
  CMD_DEMO,

/* CMD_SETMOTORDIR: Set the motor's direction.
 * Command format: [CMD] [0x05] [motor_id: 1 byte] [direction: 1 byte] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_SETMOTORDIR,

/* CMD_GETMOTORDIR: Get the motor's direction.
 * Command format: [CMD] [0x04] [motor_id: 1 byte] [0x00]
 * Expected Response: [0x10] [0x04] [motor_dir: 1 byte] [0x11] */
  CMD_GETMOTORDIR,

/* CMD_SETMOTORSPEED: Set the motor's speed.
 * Command format: [CMD] [0x08] [motor_id: 1 byte] [motor_speed: 4 byte float] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_SETMOTORSPEED,

/* CMD_GETMOTORSPEED: Get the motor's speed.
 * Command format: [CMD] [0x04] [motor_id: 1 byte] [0x00]
 * Expected Response: [0x10] [0x07] [motor speed: 4 byte float] [0x11] */
  CMD_GETMOTORSPEED,

/* CMD_SETMOTORANGLES: Set the motor joint angles.
 * Command format: [CMD] [0x13] [0x????????] [0x????????] [0x????????] [0x????????]  [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_SETMOTORANGLES,

/* CMD_SETMOTORANGLESPID: Set the motor joint angles.
 * Command format: [CMD] [0x13] [0x????????] [0x????????] [0x????????] [0x????????]  [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_SETMOTORANGLESPID,

/* CMD_GETMOTORANGLES: Get the motor joint angles.
 * Command format: [CMD] [0x03] [0x00]
 * Expected Response: [0x10] [0x13] [16 bytes of 4 float values] [0x11] */
  CMD_GETMOTORANGLES,

/* CMD_GETMOTORANGLETIMESTAMP: Get the motor joint angles with a timestamp.
 * Command format: [CMD] [0x03] [0x00]
 * Expected Response: [0x10] [0x17] [4 byte timestamp] [16 bytes motor data] [0x11] */
  CMD_GETMOTORANGLESTIMESTAMP,

/* CMD_SETMOTORANGLE: set a motor joint angle.
 * Command format: [CMD] [0x08] [1 byte motor ID] [4 byte float angle] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_SETMOTORANGLE,

/* CMD_SETMOTORANGLEPID: set a motor joint angle.
 * Command format: [CMD] [0x08] [1 byte motor ID] [4 byte float angle] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_SETMOTORANGLEPID,

/* CMD_GETMOTORANGLE: set a motor joint angle.
 * Command format: [CMD] [0x04] [1 byte motor ID] [0x00]
 * Expected Response: [0x10] [0x07] [4 byte float angle] [0x11] */
  CMD_GETMOTORANGLE,

/* CMD_GETMOTORANGLETIMESTAMP: get a motor joint angle with a timestamp.
 * Command format: [CMD] [0x04] [1 byte motor ID] [0x00]
 * Expected Response: [0x10] [0x0b] [4 byte timestamp] [4 byte float angle] [0x11] */
  CMD_GETMOTORANGLETIMESTAMP,

/* CMD_GETMOTORSTATE: get a motor's current state.
 * Command format: [CMD] [0x04] [1 byte motor ID] [0x00]
 * Expected Response: [0x10] [0x04] [1 byte motor state] [0x11] 
 * Motor State: 0 - Idle
 *              1 - Goal Seeking
 *              2 - Moving Backward?
 *              3 - Goal Attained, holding
 *              */
  CMD_GETMOTORSTATE,

/* CMD_GETMOTORMAXSPEED: Gets a motor's maximum allowed speed setting 
 * Command Format: [CMD] [0x04] [1 byte motor ID] [0x00]
 * Expected Response: [0x10] [0x07] [4 bytes float radian value] [0x11]
 * */
  CMD_GETMOTORMAXSPEED,
  
/* CMD_GETENCODERVOLTAGE: get a motor's current state.
 * Command format: [CMD] [0x04] [1 byte encoder-pin ID] [0x00]
 * Expected Response: [0x10] [0x07] [4 byte float voltage] [0x11] */
  CMD_GETENCODERVOLTAGE,

/* CMD_GETBUTTONVOLTAGE: get a motor's current state.
 * Command format: [CMD] [0x03] [0x00]
 * Expected Response: [0x10] [0x07] [4 byte float voltage] [0x11] */
  CMD_GETBUTTONVOLTAGE,

/* CMD_GETMOTORSAFETYLIMIT: get a motor's safety angle limit.
 * Command format: [CMD] [0x03] [0x00]
 * Expected Response: [0x10] [0x07] [4 byte float voltage] [0x11] */
  CMD_GETMOTORSAFETYLIMIT,

/* CMD_SETMOTORSAFETYLIMIT: set a motor's safety angle limit.
 * Command format: [CMD] [0x08] [4 byte float angle] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_SETMOTORSAFETYLIMIT,

/* CMD_GETMOTORSAFETYTIMEOUT: get a motor's safety angle timeout duration.
 * Command format: [CMD] [0x04] [0x00]
 * Expected Response: [0x10] [0x07] [4 byte float] [0x11] */
  CMD_GETMOTORSAFETYTIMEOUT,

/* CMD_SETMOTORSAFETYTIMEOUT: set a motor's safety angle timeout duration.
 * Command format: [CMD] [0x08] [4 byte float] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_SETMOTORSAFETYTIMEOUT,

/* CMD_STOP: Emergency stop all motors.
 * Command format: [CMD] [0x03] [0x00]
 * Expected Response: [0x10] [0x03] [0x11] */
  CMD_STOP,

/* CMD_GETVERSION: Gets the protocol version. Currently, this value is just CMD_NUMCOMMANDS.
 * Command format: [CMD] [0x03] [0x00]
 * Expected Response: [0x10] [0x04] [1byte version] [0x11] */
  CMD_GETVERSION,

  CMD_NUMCOMMANDS
};

#endif
