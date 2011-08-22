#include <stdio.h>
#include "imobot.h"
#include "stdint.h"
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef _CH_
#pragma package <chi2cio>
#include <i2c-api.h>
#else
#include "libi2c/i2c-api.h"
#endif

#ifdef __cplusplus
}
#endif

#include <unistd.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <bluetooth/sdp.h>
#include <bluetooth/sdp_lib.h>

int iMobot_getMotorDirection(iMobot_t* iMobot, int id, int *dir)
{
  uint8_t byte;
  I2cSetSlaveAddress(iMobot->i2cDev, I2C_HC_ADDR, 0);
  I2cReadByte(iMobot->i2cDev, I2C_REG_MOTORDIR(id), &byte);
  *dir = byte;
  return 0;
}

int iMobot_getMotorPosition(iMobot_t* iMobot, int id, double *angle)
{
  uint8_t hibyte, lobyte;
  short s_angle;
  I2cSetSlaveAddress(iMobot->i2cDev, I2C_HC_ADDR, 0);
  I2cReadByte(iMobot->i2cDev, I2C_REG_MOTORPOS(id), &hibyte);
  I2cSetSlaveAddress(iMobot->i2cDev, I2C_HC_ADDR, 0);
  I2cReadByte(iMobot->i2cDev, I2C_REG_MOTORPOS(id)+1, &lobyte);
  s_angle = (hibyte << 8) + lobyte;
  *angle = (double)s_angle/10.0;
  return 0;
}

int iMobot_getMotorSpeed(iMobot_t* iMobot, int id, int *speed)
{
  uint8_t byte;
  I2cSetSlaveAddress(iMobot->i2cDev, I2C_HC_ADDR, 0);
  if(I2cReadByte(iMobot->i2cDev, I2C_REG_MOTORSPEED(id), &byte)) {
    return -1;
  } else {
    *speed = byte;
    return 0;
  }
}

int iMobot_getMotorState(iMobot_t* iMobot, int id, int *state)
{
  uint8_t byte;
  I2cSetSlaveAddress(iMobot->i2cDev, I2C_HC_ADDR, 0);
  if(I2cReadByte(iMobot->i2cDev, I2C_REG_MOTORSTATE(id), &byte)) {
    return -1;
  } else {
    *state = byte;
    return 0;
  }
}

int iMobot_init(iMobot_t* iMobot)
{
  const char *m_i2cDevName = "/dev/i2c-3";
  int i;
  uint8_t hibyte, lobyte;

  // Try to open the i2c device

  if (( iMobot->i2cDev = open( m_i2cDevName, O_RDWR )) < 0 )
  {
    return( -1 );
  }
  
  /* Get the positions */
  for( i = 0; i < 4; i++) {
    I2cSetSlaveAddress(iMobot->i2cDev, I2C_HC_ADDR, 0);
    I2cReadByte(iMobot->i2cDev, I2C_REG_MOTORPOS(i), &hibyte);
    I2cSetSlaveAddress(iMobot->i2cDev, I2C_HC_ADDR, 0);
    I2cReadByte(iMobot->i2cDev, I2C_REG_MOTORPOS(i)+1, &lobyte);
    iMobot->enc[i] = (hibyte << 8) + lobyte;
  }
  /* Set default speeds for the motors */
  for(i = 0; i < 4; i++) {
    iMobot_setMotorSpeed(iMobot, i, 30);
  }
  return 0;
}

sdp_session_t* register_service(uint8_t rfcomm_channel)
{
    //const char *service_uuid_string = "d1b58fd0-b3be-11e0-aff2-0800200c9a66";
    sdp_profile_desc_t profile[1];
    uint32_t service_uuid_int[] = { 0xd1b58fd0, 0xb3be11e0, 0xaff20800, 0x200c9a66};
    const char *service_name = "iMobot Control";
    const char *service_dsc = "The iMobot Bluetooth control connection";
    const char *service_prov = "iMobot";

    uuid_t root_uuid, l2cap_uuid, rfcomm_uuid, svc_uuid, svclass_uuid;
    sdp_list_t *l2cap_list = 0, 
               *rfcomm_list = 0,
               *root_list = 0,
               *proto_list = 0, 
               *access_proto_list = 0,
               *pfseq = 0,
               *svclass = 0;
    sdp_data_t *channel = 0, *psm = 0;

    sdp_record_t *record = sdp_record_alloc();

    // set the general service ID
    sdp_uuid128_create( &svc_uuid, &service_uuid_int );
    sdp_set_service_id( record, svc_uuid );

    // make the service record publicly browsable
    sdp_uuid16_create(&root_uuid, PUBLIC_BROWSE_GROUP);
    root_list = sdp_list_append(0, &root_uuid);
    sdp_set_browse_groups( record, root_list );

    // set l2cap information
    sdp_uuid16_create(&l2cap_uuid, L2CAP_UUID);
    l2cap_list = sdp_list_append( 0, &l2cap_uuid );
    proto_list = sdp_list_append( 0, l2cap_list );

    // set rfcomm information
    sdp_uuid16_create(&rfcomm_uuid, RFCOMM_UUID);
    channel = sdp_data_alloc(SDP_UINT8, &rfcomm_channel);
    rfcomm_list = sdp_list_append( 0, &rfcomm_uuid );
    sdp_list_append( rfcomm_list, channel );
    sdp_list_append( proto_list, rfcomm_list );

    // attach protocol information to service record
    access_proto_list = sdp_list_append( 0, proto_list );
    sdp_set_access_protos( record, access_proto_list );

    // Set up service class information
    sdp_uuid16_create(&svclass_uuid, SERIAL_PORT_SVCLASS_ID);
    svclass = sdp_list_append(NULL, &svclass_uuid);
    sdp_set_service_classes(record, svclass);

    // Set profile information
    sdp_uuid16_create(&profile[0].uuid, SERIAL_PORT_PROFILE_ID);
    profile[0].version = 0x0100;
    pfseq = sdp_list_append(NULL, &profile[0]);
    sdp_set_profile_descs(record, pfseq);

    // set the name, provider, and description
    sdp_set_info_attr(record, service_name, service_prov, service_dsc);
    int err = 0;
    sdp_session_t *session = 0;

    // connect to the local SDP server, register the service record, and 
    // disconnect
    session = sdp_connect( BDADDR_ANY, BDADDR_LOCAL, SDP_RETRY_IF_BUSY );
    err = sdp_record_register(session, record, 0);

    // cleanup
    sdp_data_free( channel );
    sdp_list_free( l2cap_list, 0 );
    sdp_list_free( rfcomm_list, 0 );
    sdp_list_free( root_list, 0 );
    sdp_list_free( access_proto_list, 0 );

    return session;
}

int iMobot_initListenerBluetooth(iMobot_t* iMobot, int channel)
{
  struct sockaddr_rc loc_addr = { 0 };
  int err;

  // Register the service
  register_service(channel);

  // allocate socket
  iMobot->socket = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
  if (iMobot->socket < 0) {
    fprintf(stderr, "Error allocating socket.\n");
    exit(-1);
  }

  // bind socket to port 1 of the first available 
  // local bluetooth adapter
  loc_addr.rc_family = AF_BLUETOOTH;
  loc_addr.rc_bdaddr.b[0] = 0;
  loc_addr.rc_bdaddr.b[1] = 0;
  loc_addr.rc_bdaddr.b[2] = 0;
  loc_addr.rc_bdaddr.b[3] = 0;
  loc_addr.rc_bdaddr.b[4] = 0;
  loc_addr.rc_bdaddr.b[5] = 0;
  loc_addr.rc_channel = (uint8_t) channel;
  err = bind(iMobot->socket, (struct sockaddr *)&loc_addr, sizeof(loc_addr));
  if(err < 0) {
    fprintf(stderr, "error binding. %s:%d\n", strerror(errno), errno);
    exit(-1);
  }

  // put socket into listening mode
  err = listen(iMobot->socket, 1);
  if(err < 0) {
    fprintf(stderr, "error listen. %s \n", strerror(err));
    exit(-1);
  }

  return 0;
}

int iMobot_isBusy(iMobot_t* iMobot)
{
  int i;
  uint8_t byte;
  I2cSetSlaveAddress(iMobot->i2cDev, I2C_HC_ADDR, 0);
  for(i = 0; i < 4; i++) {
    if(I2cReadByte(iMobot->i2cDev, I2C_REG_MOTORSTATE(i), &byte)) {
      return -1;
    } else {
      if(byte) {
        return 1;
      }
    }
  }
  return 0;
}

int iMobot_listenerMainLoop(iMobot_t* iMobot)
{
  struct sockaddr_rc rem_addr = { 0 };
  char buf[1024] = { 0 };
  int client, bytes_read, err;
  socklen_t opt = sizeof(rem_addr);
  // accept connections
  while(1) {
    client = accept(iMobot->socket, (struct sockaddr *)&rem_addr, &opt);
    if (client < 0) {
      fprintf(stderr, "Error accepting connection\n");
      exit(-1);
    }

    ba2str( &rem_addr.rc_bdaddr, buf );
    fprintf(stderr, "accepted connection from %s\n", buf);
    memset(buf, 0, sizeof(buf));

    // read data from the client
    err = 0;
    while (err == 0) {
      bytes_read = read(client, buf, sizeof(buf));
      if( bytes_read > 0 ) {
        printf("received [%s]\n", buf);
        err = iMobot_slaveProcessCommand(iMobot, client, bytes_read, buf);
      } else {
        err = -1;
      }
    }
    if(err == -1) {
      continue;
    }

    // close connection
    close(client);
  }
  close(iMobot->socket);
  return 0;
}

int iMobot_moveWait(iMobot_t* iMobot)
{
  int i;
  for(i = 0; i < 4; i++) {
    iMobot_waitMotor(iMobot, i);
  }
  return 0;
}

int iMobot_poseZero(iMobot_t* iMobot)
{
  int i;
  for(i = 0; i < 4; i++) {
    if(iMobot_setMotorPosition(iMobot, i, 0)) {
      return -1;
    }
  }
  return 0;
}

int iMobot_setMotorDirection(iMobot_t* iMobot, int id, int direction)
{
  int i;
  uint8_t data[2];
  memcpy(&data, &direction, 2);
  I2cSetSlaveAddress(iMobot->i2cDev, I2C_HC_ADDR, 0);
  I2cWriteByte(iMobot->i2cDev, I2C_REG_MOTORDIR(id), data[0]);
  return 0;
}

int iMobot_setMotorPosition(iMobot_t* iMobot, int id, double angle)
{
  /* Send the desired angles to the iMobot */
  /* Need to convert it to 2 bytes and send to motor*/
  int i;
  uint8_t data[2];
  /* Multiply by 10 to convert to 10*degrees, which are the units used by the
   * daughterboard. */
  short position = (short) angle*10;
  memcpy(&data, &position, 2);
  /* We should send the high byte first */
  I2cSetSlaveAddress(iMobot->i2cDev, I2C_HC_ADDR, 0);
  I2cWriteByte(iMobot->i2cDev, I2C_REG_MOTORPOS(i), data[1]);
  I2cSetSlaveAddress(iMobot->i2cDev, I2C_HC_ADDR, 0);
  I2cWriteByte(iMobot->i2cDev, I2C_REG_MOTORPOS(i)+1, data[0]);
  iMobot->enc[id] = position;

  return 0;
}

int iMobot_setMotorSpeed(iMobot_t* iMobot, int id, int speed)
{
  int i;
  uint8_t data[2];
  memcpy(&data, &speed, 2);
  I2cSetSlaveAddress(iMobot->i2cDev, I2C_HC_ADDR, 0);
  I2cWriteByte(iMobot->i2cDev, I2C_REG_MOTORSPEED(id), data[0]);
  return 0;
}

int iMobot_stop(iMobot_t* iMobot)
{ 
  /* Immediately set motor speeds to zero */
  int i;
  I2cSetSlaveAddress(iMobot->i2cDev, I2C_HC_ADDR, 0);
  for(i = 0; i < 4; i++) {
    I2cWriteByte(iMobot->i2cDev, I2C_REG_MOTORSPEED(i), 0);
  }
  return 0;
}

int iMobot_terminate(iMobot_t* iMobot)
{
  return close(iMobot->i2cDev);
}

int iMobot_waitMotor(iMobot_t* iMobot, int id)
{
  int state;
  if(iMobot_getMotorState(iMobot, id, &state)) {
    return -1;
  }
  while(state != 0) {
    usleep(100000);
    if(iMobot_getMotorState(iMobot, id, &state)) {
      return -1;
    }
  }
  return 0;
}

#define MATCHSTR(str) \
(strncmp(str, buf, strlen(str))==0)
int iMobot_slaveProcessCommand(iMobot_t* iMobot, int socket, int bytesRead, const char* buf)
{
  /* Possible Commands:
   * DEMO -- Run through a preprogrammed demo routine
   * SET_MOTOR_DIRECTION <id> <uint8_t dir>
   * GET_MOTOR_DIRECTION -- returns uint8_t
   * SET_MOTOR_SPEED    <id> <uint8_t speed>
   * GET_MOTOR_SPEED    <id> -- returns uint8_t
   * SET_MOTOR_POSITION <id> <double>
   * GET_MOTOR_POSITION <id> -- returns double 
   * GET_MOTOR_STATE    <id> -- returns uint8_t
   * STOP -- Stop all motors
   * GET_IMOBOT_STATUS -- Should return the string "IMOBOT READY"
   */
  int id;
  int32_t int32;
  char mybuf[80];
  unsigned short myshort;
  double mydouble;
#if DEBUG
  printf("Slave recv: %s\n", buf);
#endif

  /* **** *
   * DEMO *
   * **** */
  if(MATCHSTR("DEMO")) {
#if 0
    write(socket, "OK", 3);
    for(i = 0; i < 8; i++) {
      demo_move_forward(iMobot);
    }
    for(i = 0; i < 8; i++) {
      demo_move_right(iMobot);
    }
    for(i = 0; i < 8; i++) {
      demo_move_left(iMobot);
    }
    for(i = 0; i < 4; i++) {
      demo_move_backward(iMobot);
    }
    demo_turn_left(iMobot, 8);
    demo_sweep_left(iMobot, 8);
    demo_arch(iMobot);
    demo_sweep_left(iMobot, 8);
    demo_stand(iMobot);
    demo_turn_left(iMobot, 8);
    demo_stand_rotate(iMobot);
    demo_turn_left(iMobot, 8);
#endif
  } else if (MATCHSTR("SET_MOTOR_DIRECTION"))
  {
    sscanf(buf, "%*s %d %d", &id, &int32);
    if(iMobot_setMotorDirection(iMobot, id, int32)) {
      write(socket, "ERROR", 6);
    } else {
      write(socket, "OK", 6);
    }
  } else if (MATCHSTR("GET_MOTOR_DIRECTION"))
  {
    sscanf(buf, "%*s %d", &id);
    if(iMobot_getMotorDirection(iMobot, id, &int32)) {
      write(socket, "ERROR", 6);
    } else {
      sprintf(mybuf, "%d", int32);
      write(socket, mybuf, strlen(mybuf)+1);
    }
  } else if (MATCHSTR("SET_MOTOR_SPEED"))
  {
    sscanf(buf, "%*s %d %d", &id, &int32);
    if(iMobot_setMotorSpeed(iMobot, id, int32)) {
      write(socket, "ERROR", 6);
    } else {
      write(socket, "OK", 3);
    }
  } else if (MATCHSTR("GET_MOTOR_SPEED"))
  {
    sscanf(buf, "%*s %d", &id);
    if(iMobot_getMotorSpeed(iMobot, id, &int32)) {
      write(socket, "ERROR", 6);
    } else {
      sprintf(mybuf, "%d", int32);
      write(socket, mybuf, strlen(mybuf)+1);
    }
  } else if (MATCHSTR("SET_MOTOR_POSITION"))
  {
    sscanf(buf, "%*s %d %lf", &id, &mydouble);
    if(iMobot_setMotorPosition(iMobot, id, mydouble)) {
      write(socket, "ERROR", 6);
    } else {
      write(socket, "OK", 3);
    }
  } else if (MATCHSTR("GET_MOTOR_POSITION"))
  {
    sscanf(buf, "%*s %d", &id);
    if(iMobot_getMotorPosition(iMobot, id, &mydouble)) {
      write(socket, "ERROR", 6);
    } else {
      sprintf(mybuf, "%lf", mydouble);
      write(socket, mybuf, strlen(mybuf)+1);
    }
  } else if (MATCHSTR("STOP")) 
  {
    for(id = 0; id < 4; id++) {
      iMobot_setMotorSpeed(iMobot, id, 0);
    }
    write(socket, "OK", 3);
  }  else if (MATCHSTR("GET_MOTOR_STATE")) 
  {
    sscanf(buf, "%*s %d", &id);
    if(iMobot_getMotorState(iMobot, id, &int32)) {
      write(socket, "ERROR", 6);
    } else {
      sprintf(mybuf, "%d", int32);
      write(socket, mybuf, strlen(mybuf)+1);
    }
  } else if (MATCHSTR("GET_IMOBOT_STATUS"))
  {
    write(socket, "IMOBOT READY", strlen("IMOBOT READY") + 1);
  } else {
    fprintf(stderr, "Received unknown command from master: %s\n", buf);
    write(socket, "ERROR", 6);
  }
  return 0;
}
#undef MATCHSTR

/* C++ class function wrappers */
CiMobot::CiMobot() {
  iMobot_init(&_iMobot);
}

CiMobot::~CiMobot() {
}

int CiMobot::getMotorDirection(int id, int &direction)
{
  return iMobot_getMotorDirection(&_iMobot, id, &direction);
}

int CiMobot::getMotorPosition(int id, double &angle)
{
  return iMobot_getMotorPosition(&_iMobot, id, &angle);
}

int CiMobot::getMotorSpeed(int id, int &speed)
{
  return iMobot_getMotorSpeed(&_iMobot, id, &speed);
}

int CiMobot::getMotorState(int id, int &state)
{
  return iMobot_getMotorState(&_iMobot, id, &state);
}

int CiMobot::initListenerBluetooth(int channel)
{
  return iMobot_initListenerBluetooth(&_iMobot, channel);
}

int CiMobot::isBusy()
{
  return iMobot_isBusy(&_iMobot);
}

int CiMobot::listenerMainLoop()
{
  return iMobot_listenerMainLoop(&_iMobot);
}

int CiMobot::moveWait()
{
  return iMobot_moveWait(&_iMobot);
}

int CiMobot::poseZero()
{
  return iMobot_poseZero(&_iMobot);
}

int CiMobot::setMotorDirection(int id, int direction)
{
  return iMobot_setMotorDirection(&_iMobot, id, direction);
}

int CiMobot::setMotorPosition(int id, double angle)
{
  return iMobot_setMotorPosition(&_iMobot, id, angle);
}

int CiMobot::setMotorSpeed(int id, int speed)
{
  return iMobot_setMotorSpeed(&_iMobot, id, speed);
}

int CiMobot::stop()
{
  return iMobot_stop(&_iMobot);
}

int CiMobot::terminate()
{
  return iMobot_terminate(&_iMobot);
}

int CiMobot::waitMotor(int id)
{
  return iMobot_waitMotor(&_iMobot, id);
}

