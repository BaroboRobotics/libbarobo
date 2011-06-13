#include <stdio.h>
#include "barobo.h"
#include "stdint.h"
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

int BR_init(iMobot_t* iMobot)
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
  return 0;
}

int BR_pose(iMobot_t* iMobot, unsigned short enc[4], const char motorMask)
{
  /* Send the desired angles to the iMobot */
  /* Need to convert it to 2 bytes and send to motor*/
  float _angle;
  int i;
  uint8_t data[2];
  for(i = 0; i < 4; i++) {
    if((1<<i) & motorMask == 0) {
      continue;
    }
    memcpy(&data, &enc[i], 2);
    /* We should send the high byte first */
    I2cSetSlaveAddress(iMobot->i2cDev, I2C_HC_ADDR, 0);
    I2cWriteByte(iMobot->i2cDev, I2C_REG_MOTORPOS(i), data[1]);
    printf("Sent data 0x%x\n", data[1]);
    I2cSetSlaveAddress(iMobot->i2cDev, I2C_HC_ADDR, 0);
    I2cWriteByte(iMobot->i2cDev, I2C_REG_MOTORPOS(i)+1, data[0]);
    printf("Sent data 0x%x\n", data[0]);
    iMobot->enc[i] = enc[i];
  }
  
  return 0;
}

int BR_poseJoint(iMobot_t* iMobot, unsigned short id, unsigned short enc)
{
  unsigned short _enc[4];
  if(id > 3) {
    return -1;
  }
  iMobot->enc[id] = enc;
  return BR_pose(iMobot, iMobot->enc, (1<<id));
}

int BR_move(iMobot_t* iMobot, short enc[4], const char motorMask)
{
  return 0;
}

int BR_stop(iMobot_t* iMobot)
{ 
  /* Immediately set motor speeds to zero */
  int i;
  I2cSetSlaveAddress(iMobot->i2cDev, I2C_HC_ADDR, 0);
  for(i = 0; i < 4; i++) {
    I2cWriteByte(iMobot->i2cDev, I2C_REG_MOTORSPEED(i), 0);
  }
  return 0;
}

int BR_moveWait(iMobot_t* iMobot)
{
  return 0;
}

int BR_isBusy(iMobot_t* iMobot)
{
  return 0;
}

int BR_getJointAngles(iMobot_t* iMobot, double angle[4])
{
  return 0;
}

int BR_slaveProcessCommand(iMobot_t* iMobot, int socket, int bytesRead, const char* buf)
{
  /* Possible Commands:
   * DEMO -- Run through a preprogrammed demo routine
   * POSE <num> -- Tell the slave to assume a preprogrammed pose
   * GET_POS -- Tell the slave to report the recorded positions of a certain pose
   * SET_POS <id> <enc1> <enc2> <enc3> <enc4> Record the specified encoder 
   *    positions to the <id>'th pose
   * POSE_JOINT <id> <enc> Immediately move a joint to a position
   * GET_NUM_POS -- Get the number of registered poses
   * RESET_POS -- Resets all recorded poses
   * PLAY
   * PAUSE
   * STOP
   * QUIT
   * STATUS
   * CHK - returns "OK" if motors are idle, "BUSY" if one or more are moving
   * GET_ENC <num> -- Get the encoder value of motor number <num>, where <num>
   *    can be 0-3.
   * SET_COAST <id> <0|1> -- Sets a joint to coast or not
   */
  int id;
  int i;
  int16_t enc[4];
  uint8_t send_buf[40];
  uint8_t result;
  uint8_t addr;
  int rc; /* return code */
  uint8_t cmd; /* i2c command byte */
  int16_t pos1[2]; /* Master motor encoder positions */
  int16_t pos2[2]; /* Slave motor encoder positions */
  uint8_t bytes;
  char mybuf[80];
#if DEBUG
  printf("Slave recv: %s\n", buf);
#endif

  /* **** *
   * DEMO *
   * **** */
  if(!strncmp("DEMO", buf, 4)) {
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
  } else
  /* **** *
   * POSE *
   * **** */
#if 0
  if(!strncmp("POSE ", buf, 5)) {
    sscanf(buf, "%*s%d", &id);
    /* Check to see how many poses are programmed */
    I2cSetSlaveAddress( iMobot->i2cDev, I2C_BabyO1_Addr, I2C_NO_CRC );
    cmd = I2C_CMD_GET_POS_NUM;
    rc = I2cReadByte(iMobot->i2cDev, cmd, &result);
    if (id >= result) {
      write(socket, "ERROR", 6);
    } else {
      I2cSetSlaveAddress( iMobot->i2cDev, I2C_BabyO1_Addr, I2C_NO_CRC );
      cmd = I2C_CMD_POSE;
      rc = I2cWriteByte(iMobot->i2cDev, cmd, id);
      I2cSetSlaveAddress( iMobot->i2cDev, I2C_BabyO2_Addr, I2C_NO_CRC );
      cmd = I2C_CMD_POSE;
      rc = I2cWriteByte(iMobot->i2cDev, cmd, id);
      write(socket, "OK", 3);
    }
  } else
  /* ******* *
   * GET_POS *
   * ******* */
  if(!strncmp("GET_POS", buf, 7)) {
    sscanf(buf, "%*s%d", &id);
    /* Check to see how many poses are programmed */
    printf("Getting pose of position %d...\n", id);
    I2cSetSlaveAddress( iMobot->i2cDev, I2C_BabyO1_Addr, I2C_NO_CRC );
    cmd = I2C_CMD_GET_POS_NUM;
    rc = I2cReadByte(iMobot->i2cDev, cmd, &result);
    if (id >= result) {
      printf("Error retrieving pose %d. Only %d poses.\n", id, result);
      write(socket, "ERROR", 6);
    } else {
      /* Get positions from AVR chips */
      cmd = I2C_CMD_GET_POS_INFO;
      I2cSetSlaveAddress( iMobot->i2cDev, I2C_BabyO1_Addr, I2C_NO_CRC );
      rc = I2cWriteByte(iMobot->i2cDev, cmd, id);
      I2cTransfer( iMobot->i2cDev, id, NULL, 0, pos1, 4 , &bytes);
      I2cSetSlaveAddress( iMobot->i2cDev, I2C_BabyO2_Addr, I2C_NO_CRC );
      rc = I2cWriteByte(iMobot->i2cDev, cmd, id);
      I2cTransfer( iMobot->i2cDev, id, NULL, 0, pos2, 4 , &bytes);

      sprintf(mybuf, "%d %d %d %d", pos1[0], pos1[1], pos2[0], pos2[1]);

      write(socket, mybuf, strlen(mybuf)+1);
    }
  } else
  /* ******* *
   * SET_POS *
   * ******* */
  if(!strncmp("SET_POS ", buf, 8)) {
    sscanf(buf, "%*s %d %hd %hd %hd %hd", &id, &enc[0], &enc[1], &enc[2], &enc[3]);

    printf("Setting encoder positions of %d to %hd %hd %hd %hd\n", id, enc[0], enc[1], enc[2], enc[3]);
    cmd = I2C_CMD_SND_POS;
    send_buf[0] = id;
    memcpy(&send_buf[1], &enc[0], 2);
    memcpy(&send_buf[3], &enc[1], 2);
    I2cSetSlaveAddress( iMobot->i2cDev, I2C_BabyO1_Addr, I2C_NO_CRC );
    I2cWriteBytes(iMobot->i2cDev, cmd, send_buf, 5);

    memcpy(&send_buf[1], &enc[2], 2);
    memcpy(&send_buf[3], &enc[3], 2);
    I2cSetSlaveAddress( iMobot->i2cDev, I2C_BabyO2_Addr, I2C_NO_CRC );
    I2cWriteBytes(iMobot->i2cDev, cmd, send_buf, 5);

    write(socket, "OK", 3);
  } else 
#endif
    /* ********** *
     * POSE_JOINT *
     * ********** */
  if(!strncmp("POSE_JOINT ", buf, 11)) {
    sscanf(buf, "%*s %d %hd", &id, &enc[0]);
    if(BR_poseJoint(iMobot, id, enc[0])) {
      write(socket, "ERROR", 3);
    } else {
      write(socket, "OK", 3);
    }
  } 
#if 0
  else 
    /* *********** *
     * GET_NUM_POS *
     * *********** */
  if (!strncmp("GET_NUM_POS", buf, 11)) {
    result = 0;
    I2cSetSlaveAddress( iMobot->i2cDev, I2C_BabyO1_Addr, I2C_NO_CRC );
    cmd = I2C_CMD_GET_POS_NUM;
    rc = I2cReadByte(iMobot->i2cDev, cmd, &result);
    sprintf(mybuf, "%d", result);
    write(socket, mybuf, strlen(buf)+1);

    /* ********* *
     * RESET_POS *
     * ********* */
  } 
  else if (!strncmp("RESET_POS", buf, 9)) {
    cmd = I2C_CMD_RST;
    I2cSetSlaveAddress( iMobot->i2cDev, I2C_BabyO1_Addr, I2C_NO_CRC );
    I2cWriteBytes(iMobot->i2cDev, cmd, NULL, 0);
    I2cSetSlaveAddress( iMobot->i2cDev, I2C_BabyO2_Addr, I2C_NO_CRC );
    I2cWriteBytes(iMobot->i2cDev, cmd, NULL, 0);
    write(socket, "OK", 3);
  /* **** *
   * PLAY *
   * **** */
  } else if (!strncmp("PLAY", buf, 4)) {
    cmd = I2C_CMD_PLY;
    I2cSetSlaveAddress( iMobot->i2cDev, I2C_BabyO1_Addr, I2C_NO_CRC );
    I2cWriteBytes(iMobot->i2cDev, cmd, NULL, 0);
    write(socket, "OK", 3);
  }
#endif
  else if (!strncmp("STOP", buf, 4)) {
    BR_stop(iMobot);
    write(socket, "OK", 3);
  } else if (!strncmp("QUIT", buf, 4)) {
    write(socket, "OK", 3);
    return 1;
  } 
#if 0
  else if (!strncmp("STATUS", buf, 6)) {
    cmd = I2C_CMD_STR;
    strcpy(mybuf, "OK");
    I2cSetSlaveAddress( iMobot->i2cDev, I2C_BabyO1_Addr, I2C_NO_CRC );
    I2cReadByte(iMobot->i2cDev, cmd, &result);
    printf("Master: 0x%x\n", result);
    if(result != I2C_CMD_RDY) {
      strcpy(mybuf, "ERROR");
    }
    I2cSetSlaveAddress( iMobot->i2cDev, I2C_BabyO2_Addr, I2C_NO_CRC );
    I2cReadByte(iMobot->i2cDev, cmd, &result);
    printf("Slave: 0x%x\n", result);
    if(result != I2C_CMD_RDY) {
      strcpy(mybuf, "ERROR");
    }
    write(socket, mybuf, strlen(mybuf)+1);
  } else if (!strncmp("CHK", buf, 3)) {
    cmd = I2C_CMD_CHK;
    sprintf(mybuf, "OK");
    I2cSetSlaveAddress( iMobot->i2cDev, I2C_BabyO1_Addr, I2C_NO_CRC );
    I2cReadByte(iMobot->i2cDev, cmd, &result);
    printf("Master: 0x%x\n", result);
    if(result != I2C_CMD_RDY) {
      sprintf(mybuf, "BUSY");
    }
    I2cSetSlaveAddress( iMobot->i2cDev, I2C_BabyO2_Addr, I2C_NO_CRC );
    I2cReadByte(iMobot->i2cDev, cmd, &result);
    printf("Slave: 0x%x\n", result);
    if(result != I2C_CMD_RDY) {
      sprintf(mybuf, "BUSY");
    }
    write(socket, mybuf, strlen(mybuf)+1);
  } 
#endif
  else if (!strncmp("GET_ENC", buf, 7)) {
    /* TODO */
    write(socket, "TODO", strlen("TODO")+1);
  } else if (!strncmp("SET_MOTOR_SPEED", buf, strlen("SET_MOTOR_SPEED"))) {
#if 0
    int motor_num;
    int motor_speed;
    sscanf(buf, "%*s %d %d", &motor_num, &motor_speed);
    if(BR_setJointSpeed(motor_num, motor_speed)) {
      write(socket, "OK", 3);
    } else {
      write(socket, "ERROR", 5);
    }
#endif
  } 
#if 0
  else if (!strncmp("SET_COAST ", buf, 10)) {
    int coast;
    sscanf(buf, "SET_COAST %d %d", &id, &coast);
#ifdef DEBUG
    printf("Coast command, set motor %d to %d\n", id, coast);
#endif
    cmd = I2C_CMD_SET_MOTOR_COAST;
    if(id == 0 || id == 1) {
      addr = I2C_BabyO1_Addr;
    } else if (id == 2 || id == 3) {
      addr = I2C_BabyO2_Addr;
      id -= 2;
    } else {
      write(socket, "ERROR", 6);
      return -1;
    }
    send_buf[0] = id;
    send_buf[1] = coast;
    I2cSetSlaveAddress( iMobot->i2cDev, addr, I2C_NO_CRC );
    rc = I2cWriteBytes(iMobot->i2cDev, cmd, send_buf, 3);
    write(socket, "OK", 3);
  } 
#endif
  else {
    fprintf(stderr, "Received unknown command from master: %s\n", buf);
    write(socket, "ERROR", 6);
  }
  return 0;
}

int BR_terminate(iMobot_t* iMobot)
{
  return close(iMobot->i2cDev);
}
