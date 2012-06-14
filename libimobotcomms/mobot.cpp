
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "mobot.h"
#include "mobot_internal.h"
#ifndef _WIN32
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <libgen.h>
#include <signal.h>
#else
#include <windows.h>
#include <shlobj.h>
#endif

#ifdef _CH_
#include <stdarg.h>
#endif

#ifdef __MACH__
#include <mach/clock.h>
#include <mach/mach.h>
#endif

#include "commands.h"

int g_numConnected = 0;

double deg2rad(double deg)
{
  return deg * M_PI / 180.0;
}

double rad2deg(double rad)
{
  return rad * 180.0 / M_PI;
}

void* nullThread(void* arg)
{ 
  return NULL;
}

/* Return Error Codes:
   -1 : General Error
   -2 : Lockfile Exists
   -3 : Address Format Incorrect
   -4 : Not enough entries in the configuration file
   -5 : Bluetooth device not found
   -6 : Protocol version mismatch
   */
int finishConnect(mobot_t* comms);
int Mobot_connect(mobot_t* comms)
{
#ifdef _WIN32
  /* Find the user's local appdata directory */
  int i;
  FILE *fp;
  char* path;
  char buf[512];
  path = comms->configFilePath;
  fp = fopen(path, "r");
  if(fp == NULL) {
    /* The file doesn't exist. Gotta make the file */
    fprintf(stderr, 
        "ERROR: Your Barobo configuration file does not exist.\n"
        "Please create one by opening the MoBot remote control, clicking on\n"
        "the 'Robot' menu entry, and selecting 'Configure Robot Bluetooth'.\n");
    return -1;
  }
  /* Read the correct line */
  for(i = 0; i < g_numConnected+1; i++) {
    if(fgets(buf, MAX_PATH, fp) == NULL) {
      return -4;
    }
  }
  fclose(fp);
  /* Get rid of trailing newline and/or carriage return */
  while(
      (buf[strlen(buf)-1] == '\r' ) ||
      (buf[strlen(buf)-1] == '\n' ) 
      )
  {
    buf[strlen(buf)-1] = '\0';
  }
  /* Make sure the format is correct */
  if(strlen(buf) != 17) {
    return -3;
  }
  /* Pass it on to connectWithAddress() */
  if(i = Mobot_connectWithAddress(comms, buf, 1)) {
    return i;
  } else {
    g_numConnected++;
    return i;
  }
#else /* if not Windows */
#define MAX_PATH 512
  FILE *fp;
  int i;
  const char* path = comms->configFilePath;
  char buf[512];
  fp = fopen(path, "r");
  if(fp == NULL) {
    fprintf(stderr, 
        "ERROR: Your Barobo configuration file does not exist.\n"
        "Please create one by opening the MoBot remote control, clicking on\n"
        "the 'Robot' menu entry, and selecting 'Configure Robot Bluetooth'.\n");
    return -1;
  }
  /* Read the correct line */
  for(i = 0; i < g_numConnected+1; i++) {
    if(fgets(buf, MAX_PATH, fp) == NULL) {
      fprintf(stderr, "Error reading configuration file.\n");
      return -1;
    }
  }
  fclose(fp);
  /* Get rid of trailing newline and/or carriage return */
  while(
      (buf[strlen(buf)-1] == '\r' ) ||
      (buf[strlen(buf)-1] == '\n' ) 
      )
  {
    buf[strlen(buf)-1] = '\0';
  }
#ifndef __MACH__ /* If Unix */
  /* Pass it on to connectWithAddress() */
  if(i = Mobot_connectWithAddress(comms, buf, 1)) {
    return i;
  } else {
    g_numConnected++;
    return i;
  }
#else /* If Mac OS */
  /* The format for the device should be /dev/tty.MOBOT-XXXX-SPP */
  char chunk1[3];
  char chunk2[3];
  sscanf(buf, "%*c%*c:%*c%*c:%*c%*c:%*c%*c:%c%c:%c%c", 
      &chunk1[0], &chunk1[1],
      &chunk2[0], &chunk2[1]);
  chunk1[2] = '\0';
  chunk2[2] = '\0';
  sprintf(buf, "/dev/tty.MOBOT-%s%s-SPP", chunk1, chunk2);
  //printf("Connecting to %s...\n", buf);
  if(i = Mobot_connectWithTTY(comms, buf)) {
    return i;
  } else {
    g_numConnected++;
    return i;
  }
#endif /* Fi Linux/Mac */
#endif
}

int Mobot_connectWithAddress(mobot_t* comms, const char* address, int channel)
{
  int err = -1;
#ifndef __MACH__
  int status;
  int flags;
  char buf[256];
#ifndef _WIN32
  comms->socket = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
#else
  comms->socket = socket(AF_BTH, SOCK_STREAM, BTHPROTO_RFCOMM);
#endif

#ifdef _WIN32
  if(comms->socket == INVALID_SOCKET) {
    err = WSAGetLastError();
    printf("Could not bind to socket. Error %d\n", err);
    if(err == 10047) {
      fprintf(stderr, "A bluetooth device could not be found on this computer. You may need to attach\nan external Bluetooth dongle to continue.\n");
      return -5;
    } else {
      return -1;
    }
  }
#endif

  // set the connection parameters (who to connect to)
#ifndef _WIN32
  comms->addr->rc_family = AF_BLUETOOTH;
  comms->addr->rc_channel = (uint8_t) channel;
  str2ba( address, &comms->addr->rc_bdaddr );
#else
  comms->addr->addressFamily = AF_BTH;
  str2ba( address, (bdaddr_t*)&comms->addr->btAddr);
  comms->addr->port = channel;
#endif

  // connect to server
  status = -1;
  int tries = 0;
  while(status < 0) {
    if(tries > 2) {
      break;
    }
    status = connect(comms->socket, (const struct sockaddr *)comms->addr, sizeof(*comms->addr));
    if(status == 0) {
      comms->connected = 1;
    } 
    tries++;
  }
  if(status < 0) {
#ifndef _WIN32
    perror("Error connecting.");
#else
	  LPVOID lpMsgBuf;
	  FormatMessage( 
		  FORMAT_MESSAGE_ALLOCATE_BUFFER | 
		  FORMAT_MESSAGE_FROM_SYSTEM | 
		  FORMAT_MESSAGE_IGNORE_INSERTS,
		  NULL,
		  GetLastError(),
		  MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), // Default language
		  (LPTSTR) &lpMsgBuf,
		  0,
		  NULL 
		  );
	  // Process any inserts in lpMsgBuf.
	  // ...
	  // Display the string.
	  //MessageBox( NULL, (LPCTSTR)lpMsgBuf, "Error", MB_OK | MB_ICONINFORMATION );
	  fprintf(stderr, "Error Connecting: %s", lpMsgBuf);
    int wsaerror = WSAGetLastError();
	  if(wsaerror == 10048) {
		  fprintf(stderr, "Make sure there are no other programs currently connected to the Mobot.\n");
	  } else if (wsaerror == 10047 || wsaerror == 10050) {
      fprintf(stderr, "A bluetooth device could not be found on this computer. You may need to attach\nan external Bluetooth dongle to continue.\n");
      err = -5;
    }
	  // Free the buffer.
	  LocalFree( lpMsgBuf );
#endif
    return err;
  }
  /* Make the socket non-blocking */
  //flags = fcntl(comms->socket, F_GETFL, 0);
  //fcntl(comms->socket, F_SETFL, flags | O_NONBLOCK);
  /* Wait for the MoBot to get ready */
  //sleep(1);
  status = finishConnect(comms);
  return status;
#else
  fprintf(stderr, "ERROR: connectWithAddress() is currently unavailable on the Mac platform.\n");
  return -1;
#endif
}

#ifndef _WIN32
int Mobot_connectWithTTY(mobot_t* comms, const char* ttyfilename)
{
  FILE *lockfile;
  char *filename = strdup(ttyfilename);
  char lockfileName[MAX_PATH];
  int pid;
  int status;
  /* Open the lock file, if it exists */
  sprintf(lockfileName, "/tmp/%s.lock", basename(filename));
  lockfile = fopen(lockfileName, "r");
  if(lockfile == NULL) {
    /* Lock file does not exist. Proceed. */
  } else {
    /* Lockfile exists. Need to check PID in the lock file and see if that
     * process is still running. */
    fscanf(lockfile, "%d", &pid);
    if(pid > 0 && kill(pid,0) < 0 && errno == ESRCH) {
      /* Lock file is stale. Delete it */
      unlink(lockfileName);
    } else {
      /* The tty device is locked. Return error code. */
      fprintf(stderr, "Error: Another application is already connected to the Mobot.\n");
      free(filename);
      fclose(lockfile);
      return -2;
    }
  }
  fclose(lockfile);
  comms->socket = open(ttyfilename, O_RDWR | O_NOCTTY );
  if(comms->socket < 0) {
    perror("Unable to open tty port.");
    return -1;
  }
  comms->connected = 1;
  status = finishConnect(comms);
  if(status) return status;
  /* Finished connecting. Create the lockfile. */
  lockfile = fopen(lockfileName, "w");
  if(lockfile == NULL) {
    fprintf(stderr, "Fatal error. %s:%d\n", __FILE__, __LINE__);
    return -1;
  }
  fprintf(lockfile, "%d", getpid());
  fclose(lockfile);
  return 0;
}
#endif

/* finishConnect():
 * Perform final connecting tasks common to all connection methods */
int finishConnect(mobot_t* comms)
{
  int i;
  uint8_t buf[256];
  /* Start the comms engine */
  THREAD_CREATE(comms->commsThread, commsEngine, comms);
  while(1) {
    if(Mobot_getStatus(comms) == 0) {
      break;
    }
#ifndef _WIN32
    usleep(200000);
#else
    Sleep(200);
#endif
  }
  /* Get the protocol version; make sure it matches ours */
  int version;
  version = Mobot_getVersion(comms); 
  if(version != CMD_NUMCOMMANDS) {
    Mobot_disconnect(comms);
    fprintf(stderr, "Error. Bluetooth protocol version mismatch.\n");
    fprintf(stderr, "Mobot Firmware Protocol Version: %d\n", version);
    fprintf(stderr, "CMobot Library Protocol Version: %d\n", CMD_NUMCOMMANDS);
    return -6;
  }
  /* Get the joint max speeds */
  for(i = 4; i >= 1; i--) {
    if(Mobot_getJointMaxSpeed(comms, (robotJointId_t)i, &(comms->maxSpeed[i-1])) < 0) {
      i++;
    }
  }
  Mobot_setJointSpeeds( comms, 
      DEG2RAD(45), 
      DEG2RAD(45), 
      DEG2RAD(45), 
      DEG2RAD(45) );
  return 0;
}

int Mobot_blinkLED(mobot_t* comms, double delay, int numBlinks)
{
  uint8_t buf[8];
  float f;
  int status;
  uint32_t millis;
  millis = delay*1000.0;
  memcpy(&buf[0], &millis, 4);
  buf[4] = (uint8_t)numBlinks;
  status = SendToIMobot(comms, BTCMD(CMD_BLINKLED), buf, 5);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_disconnect(mobot_t* comms)
{
  comms->connected = 0;
#ifndef _WIN32
  shutdown(comms->socket, SHUT_RDWR);
  if(close(comms->socket)) {
    /* Error closing file descriptor */
    return -1;
  } 
#else
  closesocket(comms->socket);
  //CloseHandle((LPVOID)comms->socket);
#endif
  if(g_numConnected > 0) {
    g_numConnected--;
  }
  return 0;
}

int Mobot_enableButtonCallback(mobot_t* comms, void* data, void (*buttonCallback)(void* data, int button, int buttonDown))
{
  uint8_t buf[16];
  int status;
  MUTEX_LOCK(comms->callback_lock);
  /* Send a message to the Mobot */
  buf[0] = 1;
  status = SendToIMobot(comms, BTCMD(CMD_ENABLEBUTTONHANDLER), buf, 1);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    MUTEX_UNLOCK(comms->callback_lock);
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 0x03) {
    MUTEX_UNLOCK(comms->callback_lock);
    return -1;
  }

  comms->buttonCallback = (void(*)(void*,int,int))buttonCallback;
  comms->callbackEnabled = 1;
  comms->mobot = data;
  MUTEX_UNLOCK(comms->callback_lock);
  return 0;
}

int Mobot_disableButtonCallback(mobot_t* comms)
{
  uint8_t buf[16];
  int status;
  MUTEX_LOCK(comms->callback_lock);
  /* Send a message to the Mobot */
  buf[0] = 0;
  status = SendToIMobot(comms, BTCMD(CMD_ENABLEBUTTONHANDLER), buf, 1);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    MUTEX_UNLOCK(comms->callback_lock);
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 0x03) {
    MUTEX_UNLOCK(comms->callback_lock);
    return -1;
  }

  comms->buttonCallback = NULL;
  comms->callbackEnabled = 0;
  MUTEX_UNLOCK(comms->callback_lock);
  return 0;
}

int Mobot_init(mobot_t* comms)
{
  int i;
#ifndef __MACH__
  comms->addr = (sockaddr_t*)malloc(sizeof(sockaddr_t));
  memset(comms->addr, 0, sizeof(sockaddr_t));
#endif
  comms->connected = 0;
#ifdef _WIN32
  WSADATA wsd;
  if(WSAStartup (MAKEWORD(2,2), &wsd) != 0) {
    printf("WSAStartup failed with error %d\n", WSAGetLastError());
  }
#endif
  for(i = 0; i < 4; i++) {
    comms->jointSpeeds[i] = DEF_MOTOR_SPEED;
    comms->recordingInProgress[i] = 0;
    /* Set the default maximum speed to something reasonable */
    comms->maxSpeed[i] = DEF_MOTOR_MAXSPEED;
  }
  comms->thread = (THREAD_T*)malloc(sizeof(THREAD_T));
  comms->commsThread = (THREAD_T*)malloc(sizeof(THREAD_T));
  THREAD_CREATE(comms->thread, nullThread, NULL);
  comms->commsLock = (MUTEX_T*)malloc(sizeof(MUTEX_T));
  MUTEX_INIT(comms->commsLock);
  comms->motionInProgress = 0;
  MUTEX_NEW(comms->recvBuf_lock);
  MUTEX_INIT(comms->recvBuf_lock);
  COND_NEW(comms->recvBuf_cond);
  COND_INIT(comms->recvBuf_cond);
  comms->recvBuf_ready = 0;
  MUTEX_NEW(comms->commsBusy_lock);
  MUTEX_INIT(comms->commsBusy_lock);
  COND_NEW(comms->commsBusy_cond);
  COND_INIT(comms->commsBusy_cond);
  comms->commsBusy = 0;
  MUTEX_NEW(comms->callback_lock);
  MUTEX_INIT(comms->callback_lock);
  comms->callbackEnabled = 0;

  /* Find the configuration file path */
#ifdef _WIN32
  /* Find the user's local appdata directory */
  char path[MAX_PATH];
  if(SHGetFolderPathA(NULL, CSIDL_LOCAL_APPDATA, NULL, 0, path) != S_OK) 
  {
    /* Could not get the user's app data directory */
  } else {
    //MessageBox((LPCTSTR)path, (LPCTSTR)"Test");
    //fprintf(fp, "%s", path); 
  }
  strcat(path, "\\Barobo.config");
  comms->configFilePath = strdup(path);
#else
  /* Try to open the barobo configuration file. */
#define MAX_PATH 512
  char path[MAX_PATH];
  strcpy(path, getenv("HOME"));
  strcat(path, "/.Barobo.config");
  comms->configFilePath = strdup(path);
#endif

  return 0;
}

int Mobot_isConnected(mobot_t* comms)
{
  if(comms->connected > 0) {
    return 1;
  } else {
    return 0;
  }
}

int Mobot_isMoving(mobot_t* comms)
{
  int moving = 0;
  robotJointState_t state;
  int i;
  for(i = 1; i <= 4; i++) {
    Mobot_getJointState(comms, (robotJointId_t)i, &state);
    if( (state == ROBOT_FORWARD) ||
        (state == ROBOT_BACKWARD) ) 
    {
      moving = 1;
      break;
    }
  }
  return moving;
}

int Mobot_getButtonVoltage(mobot_t* comms, double *voltage)
{
  uint8_t buf[32];
  float f;
  int status;
  status = SendToIMobot(comms, BTCMD(CMD_GETBUTTONVOLTAGE), NULL, 0);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 7) {
    return -1;
  }
  /* Copy the data */
  memcpy(&f, &buf[2], 4);
  *voltage = f;
  return 0;
}

const char* Mobot_getConfigFilePath()
{
  static char path[512];
  /* Find the configuration file path */
#ifdef _WIN32
  /* Find the user's local appdata directory */
  if(SHGetFolderPathA(NULL, CSIDL_LOCAL_APPDATA, NULL, 0, path) != S_OK) 
  {
    /* Could not get the user's app data directory */
  } else {
    //MessageBox((LPCTSTR)path, (LPCTSTR)"Test");
    //fprintf(fp, "%s", path); 
  }
  strcat(path, "\\Barobo.config");
#else
  /* Try to open the barobo configuration file. */
#define MAX_PATH 512
  strcpy(path, getenv("HOME"));
  strcat(path, "/.Barobo.config");
#endif
  return path;
}

int Mobot_getEncoderVoltage(mobot_t* comms, int pinNumber, double *voltage)
{
  uint8_t buf[32];
  float f;
  int status;
  buf[0] = pinNumber;
  status = SendToIMobot(comms, BTCMD(CMD_GETENCODERVOLTAGE), buf, 1);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 7) {
    return -1;
  }
  /* Copy the data */
  memcpy(&f, &buf[2], 4);
  *voltage = f;
  return 0;
}

int Mobot_getJointAngle(mobot_t* comms, robotJointId_t id, double *angle)
{
  uint8_t buf[32];
  float f;
  int status;
  buf[0] = (uint8_t)id-1;
  status = SendToIMobot(comms, BTCMD(CMD_GETMOTORANGLE), buf, 1);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 7) {
    return -1;
  }
  /* Copy the data */
  memcpy(&f, &buf[2], 4);
  *angle = f;
  return 0;
}

int Mobot_getJointAngleAbs(mobot_t* comms, robotJointId_t id, double *angle)
{
  uint8_t buf[32];
  float f;
  int status;
  buf[0] = (uint8_t)id-1;
  status = SendToIMobot(comms, BTCMD(CMD_GETMOTORANGLEABS), buf, 1);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 7) {
    return -1;
  }
  /* Copy the data */
  memcpy(&f, &buf[2], 4);
  *angle = f;
  return 0;
}

int Mobot_getJointAngleTime(mobot_t* comms, robotJointId_t id, double *time, double *angle)
{
  uint8_t buf[32];
  float f;
  uint32_t millis;
  int status;
  buf[0] = (uint8_t)id-1;
  status = SendToIMobot(comms, BTCMD(CMD_GETMOTORANGLETIMESTAMP), buf, 1);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 0x0b) {
    return -1;
  }
  /* Copy the data */
  memcpy(&millis, &buf[2], 4);
  memcpy(&f, &buf[6], 4);
  *angle = f;
  *time = millis / 1000.0;
  return 0;
}

int Mobot_getJointAngles(mobot_t* comms, 
                             double *angle1,
                             double *angle2,
                             double *angle3,
                             double *angle4)
{
  uint8_t buf[32];
  float f;
  uint32_t millis;
  int status;
  status = SendToIMobot(comms, BTCMD(CMD_GETMOTORANGLESTIMESTAMP), NULL, 0);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 0x17) {
    return -1;
  }
  /* Copy 4 joint angles */
  memcpy(&f, &buf[6], 4);
  *angle1 = f;
  memcpy(&f, &buf[10], 4);
  *angle2 = f;
  memcpy(&f, &buf[14], 4);
  *angle3 = f;
  memcpy(&f, &buf[18], 4);
  *angle4 = f;
  return 0;
}

int Mobot_getJointAnglesTime(mobot_t* comms, 
                             double *time, 
                             double *angle1,
                             double *angle2,
                             double *angle3,
                             double *angle4)
{
  uint8_t buf[32];
  float f;
  uint32_t millis;
  int status;
  status = SendToIMobot(comms, BTCMD(CMD_GETMOTORANGLESTIMESTAMP), NULL, 0);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 0x17) {
    return -1;
  }
  /* Copy the data */
  memcpy(&millis, &buf[2], 4);
  *time = millis / 1000.0;
  /* Copy 4 joint angles */
  memcpy(&f, &buf[6], 4);
  *angle1 = f;
  memcpy(&f, &buf[10], 4);
  *angle2 = f;
  memcpy(&f, &buf[14], 4);
  *angle3 = f;
  memcpy(&f, &buf[18], 4);
  *angle4 = f;
  return 0;
}

int Mobot_getJointAnglesAbs(mobot_t* comms, 
                             double *angle1,
                             double *angle2,
                             double *angle3,
                             double *angle4)
{
  uint8_t buf[32];
  float f;
  uint32_t millis;
  int status;
  status = SendToIMobot(comms, BTCMD(CMD_GETMOTORANGLESTIMESTAMPABS), NULL, 0);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 0x17) {
    return -1;
  }
  /* Copy the data */
  /* Copy 4 joint angles */
  memcpy(&f, &buf[6], 4);
  *angle1 = f;
  memcpy(&f, &buf[10], 4);
  *angle2 = f;
  memcpy(&f, &buf[14], 4);
  *angle3 = f;
  memcpy(&f, &buf[18], 4);
  *angle4 = f;
  return 0;
}

int Mobot_getJointAnglesAbsTime(mobot_t* comms, 
                             double *time, 
                             double *angle1,
                             double *angle2,
                             double *angle3,
                             double *angle4)
{
  uint8_t buf[32];
  float f;
  uint32_t millis;
  int status;
  status = SendToIMobot(comms, BTCMD(CMD_GETMOTORANGLESTIMESTAMPABS), NULL, 0);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 0x17) {
    return -1;
  }
  /* Copy the data */
  memcpy(&millis, &buf[2], 4);
  *time = millis / 1000.0;
  /* Copy 4 joint angles */
  memcpy(&f, &buf[6], 4);
  *angle1 = f;
  memcpy(&f, &buf[10], 4);
  *angle2 = f;
  memcpy(&f, &buf[14], 4);
  *angle3 = f;
  memcpy(&f, &buf[18], 4);
  *angle4 = f;
  return 0;
}

int Mobot_getJointDirection(mobot_t* comms, robotJointId_t id, robotJointState_t *dir)
{
  uint8_t buf[32];
  int status;
  buf[0] = (uint8_t)id-1;
  status = SendToIMobot(comms, BTCMD(CMD_GETMOTORDIR), buf, 1);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 0x04) {
    return -1;
  }
  *dir = (robotJointState_t)buf[2];
  return 0;
}

int Mobot_getJointMaxSpeed(mobot_t* comms, robotJointId_t id, double *maxSpeed)
{
  float f;
  uint8_t buf[64];
  int status;
  int bytes_read;
  buf[0] = (uint8_t) id-1;
  status = SendToIMobot(comms, BTCMD(CMD_GETMOTORMAXSPEED), &buf[0], 1);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 7) {
    return -1;
  }
  /* Copy the data */
  memcpy(&f, &buf[2], 4);
  *maxSpeed = f;
  return 0;
}

int Mobot_getJointSafetyAngle(mobot_t* comms, double *angle) 
{
  uint8_t buf[32];
  float f;
  int status;
  status = SendToIMobot(comms, BTCMD(CMD_GETMOTORSAFETYLIMIT), NULL, 0);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 0x07) {
    return -1;
  }
  memcpy(&f, &buf[2], 4);
  *angle = f;
  return 0;
}

int Mobot_getJointSafetyAngleTimeout(mobot_t* comms, double *seconds) 
{
  uint8_t buf[32];
  float f;
  int status;
  status = SendToIMobot(comms, BTCMD(CMD_GETMOTORSAFETYTIMEOUT), NULL, 0);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 0x07) {
    return -1;
  }
  memcpy(&f, &buf[2], 4);
  *seconds = f;
  return 0;
}

int Mobot_getJointSpeed(mobot_t* comms, robotJointId_t id, double *speed)
{
  uint8_t buf[32];
  float f;
  int status;
  buf[0] = (uint8_t)id-1;
  status = SendToIMobot(comms, BTCMD(CMD_GETMOTORSPEED), buf, 1);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 0x07) {
    return -1;
  }
  memcpy(&f, &buf[2], 4);
  *speed = f;
  comms->jointSpeeds[id-1] = *speed;
  return 0;
}

int Mobot_getJointSpeedRatio(mobot_t* comms, robotJointId_t id, double *ratio)
{
  double speed;
  Mobot_getJointSpeed(comms, id, &speed);
  *ratio = speed / comms->maxSpeed[(int)id-1];
  return 0;
}

int Mobot_getJointSpeedRatios(mobot_t* comms, double *ratio1, double *ratio2, double *ratio3, double *ratio4)
{
  double *ratios[4];
  ratios[0] = ratio1;
  ratios[1] = ratio2;
  ratios[2] = ratio3;
  ratios[3] = ratio4;
  for(int i = 0; i < 4; i++) {
    Mobot_getJointSpeedRatio(comms, (robotJointId_t)(i+1), ratios[i]);
  }
  return 0;
}

int Mobot_getJointState(mobot_t* comms, robotJointId_t id, robotJointState_t *state)
{
  uint8_t buf[32];
  int status;
  buf[0] = (uint8_t)id-1;
  status = SendToIMobot(comms, BTCMD(CMD_GETMOTORSTATE), buf, 1);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 0x04) {
    return -1;
  }
  *state = (robotJointState_t)buf[2];
  return 0;
}

int Mobot_getJointSpeeds(mobot_t* comms, double *speed1, double *speed2, double *speed3, double *speed4)
{
  double *speeds[4];
  speeds[0] = speed1;
  speeds[1] = speed2;
  speeds[2] = speed3;
  speeds[3] = speed4;
  for(int i = 0; i < 4; i++) {
    Mobot_getJointSpeed(comms, (robotJointId_t)(i+1), speeds[i]);
  }
  return 0;
}

int Mobot_getStatus(mobot_t* comms)
{
  uint8_t buf[64];
  SendToIMobot(comms, BTCMD(CMD_STATUS), NULL, 0);
  RecvFromIMobot(comms, buf, 64);
  if(buf[0] != RESP_OK) {
    return -1;
  }
  if(buf[1] != 3 ) {
    return -1;
  }
  if(buf[2] != RESP_END) {
    return -1;
  }
  return 0;
}

int Mobot_getVersion(mobot_t* comms)
{
  uint8_t buf[16];
  int version;
  SendToIMobot(comms, BTCMD(CMD_GETVERSION), NULL, 0);
  RecvFromIMobot(comms, buf, 16);
  if(buf[0] != RESP_OK) {
    return -1;
  }
  if(buf[1] != 4 ) {
    return -1;
  }
  if(buf[3] != RESP_END) {
    return -1;
  }
  version = buf[2];
  return version;
}

int Mobot_move(mobot_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4)
{
  Mobot_moveNB(comms, 
      angle1,
      angle2,
      angle3,
      angle4);

  /* Wait for the motion to complete */
  return Mobot_moveWait(comms);
}

int Mobot_moveNB(mobot_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4)
{
  double angles[4];
  double curAngles[4];
  int i;
  double time;
  uint8_t buf[32];
  float f;
  angles[0] = angle1;
  angles[1] = angle2;
  angles[2] = angle3;
  angles[3] = angle4;
  /* Get the current joint angles */
  Mobot_getJointAnglesTime(comms, &time, 
      &curAngles[0],
      &curAngles[1],
      &curAngles[2],
      &curAngles[3] );
  /* Calculate new angles */
  for(i = 0; i < 4; i++) {
    angles[i] = curAngles[i] + angles[i];
  }
  /* Set up message buffer */
  for(i = 0; i < 4; i++) {
    f = angles[i];
    memcpy(&buf[i*4], &f, 4);
  }
  SendToIMobot(comms, BTCMD(CMD_SETMOTORANGLES), buf, 4*4);
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 0x03) {
    return -1;
  }
  return 0;
}

int Mobot_moveContinuousNB(mobot_t* comms,
                                  robotJointState_t dir1,
                                  robotJointState_t dir2,
                                  robotJointState_t dir3,
                                  robotJointState_t dir4)
{
  robotJointState_t dirs[4];
  int i;
  dirs[0] = dir1;
  dirs[1] = dir2;
  dirs[2] = dir3;
  dirs[3] = dir4;
  for(i = 0; i < 4; i++) {
    if(dirs[i] == ROBOT_FORWARD) {
      Mobot_setJointSpeed(comms, (robotJointId_t)(i+1), comms->jointSpeeds[i]);
      Mobot_setJointDirection(comms, (robotJointId_t)(i+1), ROBOT_FORWARD);
    } else if (dirs[i] == ROBOT_BACKWARD) {
      Mobot_setJointSpeed(comms, (robotJointId_t)(i+1), comms->jointSpeeds[i]);
      Mobot_setJointDirection(comms, (robotJointId_t)(i+1), ROBOT_BACKWARD);
    } else {
      Mobot_setJointDirection(comms, (robotJointId_t)(i+1), dirs[i]);
    }
  }
  return 0;
}

int Mobot_moveContinuousTime(mobot_t* comms,
                                  robotJointState_t dir1,
                                  robotJointState_t dir2,
                                  robotJointState_t dir3,
                                  robotJointState_t dir4,
                                  double seconds)
{
  int i;
  int msecs = seconds * 1000;
  Mobot_moveContinuousNB(comms, dir1, dir2, dir3, dir4);
#ifdef _WIN32
  Sleep(msecs);
#else
  usleep(msecs * 1000);
#endif
  /* Stop the motors */
  for(i = 0; i < 4; i++) {
    Mobot_setJointDirection(comms, (robotJointId_t)(i+1), ROBOT_NEUTRAL);
  }
  return 0;
}

int Mobot_moveJointContinuousNB(mobot_t* comms, robotJointId_t id, robotJointState_t dir)
{
  Mobot_setJointSpeed(comms, id, comms->jointSpeeds[(int)id-1]);
  Mobot_setJointDirection(comms, id, dir);
  return 0;
}

int Mobot_moveJointContinuousTime(mobot_t* comms, robotJointId_t id, robotJointState_t dir, double seconds)
{
  int msecs = seconds * 1000;
  Mobot_moveJointContinuousNB(comms, id, dir);
#ifdef _WIN32
  Sleep(msecs);
#else
  usleep(msecs * 1000);
#endif
  return 0;
}

int Mobot_moveJoint(mobot_t* comms, robotJointId_t id, double angle)
{
  double curAngle;
  if(Mobot_getJointAngle(comms, id, &curAngle)) {
    return -1;
  }
  return Mobot_moveJointTo(comms, id, curAngle + angle);
}

int Mobot_moveJointNB(mobot_t* comms, robotJointId_t id, double angle)
{
  double curAngle;
  if(Mobot_getJointAngle(comms, id, &curAngle)) {
    return -1;
  }
  return Mobot_moveJointToNB(comms, id, curAngle + angle);
}

int Mobot_moveJointTo(mobot_t* comms, robotJointId_t id, double angle)
{
  Mobot_moveJointToNB(comms, id, angle);
  /* Wait for the motion to finish */
  return Mobot_moveJointWait(comms, id);
}

int Mobot_moveJointToAbs(mobot_t* comms, robotJointId_t id, double angle)
{
  Mobot_moveJointToAbsNB(comms, id, angle);
  /* Wait for the motion to finish */
  return Mobot_moveJointWait(comms, id);
}

int Mobot_moveJointToDirect(mobot_t* comms, robotJointId_t id, double angle)
{
  Mobot_moveJointToDirectNB(comms, id, angle);
  /* Wait for the motion to finish */
  return Mobot_moveJointWait(comms, id);
}

int Mobot_moveJointToPIDNB(mobot_t* comms, robotJointId_t id, double angle)
{
  uint8_t buf[32];
  float f;
  int status;
  buf[0] = (uint8_t)id-1;
  f = angle;
  memcpy(&buf[1], &f, 4);
  status = SendToIMobot(comms, BTCMD(CMD_SETMOTORANGLEPID), buf, 5);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_moveJointToNB(mobot_t* comms, robotJointId_t id, double angle)
{
  uint8_t buf[32];
  float f;
  int status;
  if((id == ROBOT_JOINT2) || (id == ROBOT_JOINT3)) {
    if(angle > 90) {
      fprintf(stderr, "Warning: Angle for joint %d set beyond limits.\n", (int)(id + 1));
      angle = 90;
    }
    if(angle < -90) {
      fprintf(stderr, "Warning: Angle for joint %d set beyond limits.\n", (int)(id + 1));
      angle = -90;
    }
  }
  buf[0] = (uint8_t)id-1;
  f = angle;
  memcpy(&buf[1], &f, 4);
  status = SendToIMobot(comms, BTCMD(CMD_SETMOTORANGLE), buf, 5);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_moveJointToAbsNB(mobot_t* comms, robotJointId_t id, double angle)
{
  uint8_t buf[32];
  float f;
  int status;
  if((id == ROBOT_JOINT2) || (id == ROBOT_JOINT3)) {
    if(angle > 90) {
      fprintf(stderr, "Warning: Angle for joint %d set beyond limits.\n", (int)(id + 1));
      angle = 90;
    }
    if(angle < -90) {
      fprintf(stderr, "Warning: Angle for joint %d set beyond limits.\n", (int)(id + 1));
      angle = -90;
    }
  }
  buf[0] = (uint8_t)id-1;
  f = angle;
  memcpy(&buf[1], &f, 4);
  status = SendToIMobot(comms, BTCMD(CMD_SETMOTORANGLEABS), buf, 5);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_moveJointToDirectNB(mobot_t* comms, robotJointId_t id, double angle)
{
  uint8_t buf[32];
  float f;
  int status;
  if((id == ROBOT_JOINT2) || (id == ROBOT_JOINT3)) {
    if(angle > 90) {
      fprintf(stderr, "Warning: Angle for joint %d set beyond limits.\n", (int)(id + 1));
      angle = 90;
    }
    if(angle < -90) {
      fprintf(stderr, "Warning: Angle for joint %d set beyond limits.\n", (int)(id + 1));
      angle = -90;
    }
  }
  buf[0] = (uint8_t)id-1;
  f = angle;
  memcpy(&buf[1], &f, 4);
  status = SendToIMobot(comms, BTCMD(CMD_SETMOTORANGLEDIRECT), buf, 5);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_moveJointWait(mobot_t* comms, robotJointId_t id)
{
  robotJointState_t state;
  /* Make sure there is no non-blocking function running */
//  THREAD_JOIN(comms->thread);

  while(1)
  {
    if(Mobot_getJointState(comms, id, &state)) {
      return -1;
    }
    if(
      (state == ROBOT_NEUTRAL) ||
      (state == ROBOT_HOLD) )
    {
      return 0;
    } else {
#ifndef _WIN32
      usleep(200000);
#else
      Sleep(200);
#endif
    }
  }
  return 0;
}

int Mobot_moveToZero(mobot_t* comms)
{
  return Mobot_moveTo(comms, 0, 0, 0, 0);
}

int Mobot_moveToZeroNB(mobot_t* comms)
{
  return Mobot_moveToNB(comms, 0, 0, 0, 0);
}

int Mobot_moveTo(mobot_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4)
{
  Mobot_moveToNB(comms, 
      angle1, 
      angle2, 
      angle3, 
      angle4 );
  return Mobot_moveWait(comms);
}

int Mobot_moveToAbs(mobot_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4)
{
  Mobot_moveToAbsNB(comms, 
      angle1, 
      angle2, 
      angle3, 
      angle4 );
  return Mobot_moveWait(comms);
}

int Mobot_moveToDirect(mobot_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4)
{
  Mobot_moveToDirectNB(comms, 
      angle1, 
      angle2, 
      angle3, 
      angle4 );
  return Mobot_moveWait(comms);
}


int Mobot_moveToPID(mobot_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4)
{
  Mobot_moveToPIDNB(comms, 
      angle1, 
      angle2, 
      angle3, 
      angle4 );
  return Mobot_moveWait(comms);
}

int Mobot_moveToNB(mobot_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4)
{
  uint8_t buf[32];
  float f;
  int status;
  f = angle1;
  memcpy(&buf[0], &f, 4);
  f = angle2;
  memcpy(&buf[4], &f, 4);
  f = angle3;
  memcpy(&buf[8], &f, 4);
  f = angle4;
  memcpy(&buf[12], &f, 4);
  status = SendToIMobot(comms, BTCMD(CMD_SETMOTORANGLES), buf, 16);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_moveToAbsNB(mobot_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4)
{
  uint8_t buf[32];
  float f;
  int status;
  f = angle1;
  memcpy(&buf[0], &f, 4);
  f = angle2;
  memcpy(&buf[4], &f, 4);
  f = angle3;
  memcpy(&buf[8], &f, 4);
  f = angle4;
  memcpy(&buf[12], &f, 4);
  status = SendToIMobot(comms, BTCMD(CMD_SETMOTORANGLESABS), buf, 16);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_moveToDirectNB(mobot_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4)
{
  uint8_t buf[32];
  float f;
  int status;
  f = angle1;
  memcpy(&buf[0], &f, 4);
  f = angle2;
  memcpy(&buf[4], &f, 4);
  f = angle3;
  memcpy(&buf[8], &f, 4);
  f = angle4;
  memcpy(&buf[12], &f, 4);
  status = SendToIMobot(comms, BTCMD(CMD_SETMOTORANGLESDIRECT), buf, 16);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_moveToPIDNB(mobot_t* comms,
                               double angle1,
                               double angle2,
                               double angle3,
                               double angle4)
{
  uint8_t buf[32];
  float f;
  int status;
  f = angle1;
  memcpy(&buf[0], &f, 4);
  f = angle2;
  memcpy(&buf[4], &f, 4);
  f = angle3;
  memcpy(&buf[8], &f, 4);
  f = angle4;
  memcpy(&buf[12], &f, 4);
  status = SendToIMobot(comms, BTCMD(CMD_SETMOTORANGLESPID), buf, 16);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_moveWait(mobot_t* comms)
{
  int i;
  /* Make sure there is no non-blocking function running */
//  THREAD_JOIN(comms->thread);
  for(i = 0; i < 4; i++) {
    if(Mobot_moveJointWait(comms, (robotJointId_t)(i+1))) {
      return -1;
    }
  }
  return 0;
}

void* Mobot_recordAngleThread(void* arg);
int Mobot_recordAngle(mobot_t* comms, robotJointId_t id, double* time, double* angle, int num, double timeInterval)
{
  THREAD_T thread;
  recordAngleArg_t *rArg;
  int msecs = timeInterval * 1000;
  if(comms->recordingInProgress[id-1]) {
    return -1;
  }
  rArg = (recordAngleArg_t*)malloc(sizeof(recordAngleArg_t));
  rArg->comms = comms;
  rArg->id = id;
  rArg->time = time;
  rArg->angle = angle;
  rArg->num = num;
  rArg->msecs = msecs;
  comms->recordingInProgress[id-1] = 1;
  THREAD_CREATE(&thread, Mobot_recordAngleThread, rArg);
  return 0;
}

#ifndef _WIN32
unsigned int diff_msecs(struct timespec t1, struct timespec t2)
{
  unsigned int t;
  t = (t2.tv_sec - t1.tv_sec) * 1000;
  t += (t2.tv_nsec - t1.tv_nsec) / 1000000;
  return t;
}
#endif

void* Mobot_recordAngleThread(void* arg)
{
#ifndef _WIN32
  recordAngleArg_t *rArg = (recordAngleArg_t*) arg;
  int i;
  struct timespec cur_time, itime;
  unsigned int dt;
  double start_time;
  for(i = 0; i < rArg->num; i++) {
#ifndef __MACH__
    clock_gettime(CLOCK_REALTIME, &cur_time);
#else
    clock_serv_t cclock;
    mach_timespec_t mts;
    host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
    clock_get_time(cclock, &mts);
    mach_port_deallocate(mach_task_self(), cclock);
    cur_time.tv_sec = mts.tv_sec;
    cur_time.tv_nsec = mts.tv_nsec;
#endif
    Mobot_getJointAngleTime(rArg->comms, rArg->id, &rArg->time[i], &rArg->angle[i]);
    if(i == 0) {
      start_time = rArg->time[i];
    }
    rArg->time[i] = rArg->time[i] - start_time;
    /* Convert angle to degrees */
    rArg->angle[i] = RAD2DEG(rArg->angle[i]);
#ifndef __MACH__
    clock_gettime(CLOCK_REALTIME, &itime);
#else
    host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
    clock_get_time(cclock, &mts);
    mach_port_deallocate(mach_task_self(), cclock);
    cur_time.tv_sec = mts.tv_sec;
    cur_time.tv_nsec = mts.tv_nsec;
#endif
    dt = diff_msecs(cur_time, itime);
    if(dt < (rArg->msecs)) {
      usleep(rArg->msecs*1000 - dt*1000);
    }
  }
  rArg->comms->recordingInProgress[rArg->id-1] = 0;
#else
  recordAngleArg_t *rArg = (recordAngleArg_t*) arg;
  int i;
  DWORD cur_time, itime;
  unsigned int dt;
  double start_time;
  for(i = 0; i < rArg->num; i++) {
    cur_time = GetTickCount();
    Mobot_getJointAngleTime(rArg->comms, rArg->id, &rArg->time[i], &rArg->angle[i]);
    if(i == 0) {
      start_time = rArg->time[i];
    }
    rArg->time[i] = rArg->time[i] - start_time;
    /* Convert angle to degrees */
    rArg->angle[i] = RAD2DEG(rArg->angle[i]);
    itime = GetTickCount();
    dt = itime - cur_time;
    if(dt < (rArg->msecs)) {
      Sleep(rArg->msecs - dt);
    }
  }
  rArg->comms->recordingInProgress[rArg->id-1] = 0;
#endif
  return NULL;

}

void* recordAnglesThread(void* arg);
int Mobot_recordAngles(mobot_t* comms, 
                      double *time, 
                      double* angle1, 
                      double* angle2,
                      double* angle3,
                      double* angle4,
                      int num,
                      double timeInterval)
{
  int i;
  THREAD_T thread;
  recordAngleArg_t *rArg;
  int msecs = timeInterval * 1000;
  for(i = 0; i < 4; i++) {
    if(comms->recordingInProgress[i]) {
      return -1;
    }
  }
  rArg = (recordAngleArg_t*)malloc(sizeof(recordAngleArg_t));
  rArg->comms = comms;
  rArg->time = time;
  rArg->angle = angle1;
  rArg->angle2 = angle2;
  rArg->angle3 = angle3;
  rArg->angle4 = angle4;
  rArg->num = num;
  rArg->msecs = msecs;
  for(i = 0; i < 4; i++) {
    comms->recordingInProgress[i] = 1;
  }
  THREAD_CREATE(&thread, recordAnglesThread, rArg);
  return 0;
}

void* recordAnglesThread(void* arg)
{
#ifndef _WIN32
  recordAngleArg_t *rArg = (recordAngleArg_t*) arg;
  int i;
  struct timespec cur_time, itime;
  unsigned int dt;
  double start_time;
  for(i = 0; i < rArg->num; i++) {
#ifndef __MACH__
    clock_gettime(CLOCK_REALTIME, &cur_time);
#else
    clock_serv_t cclock;
    mach_timespec_t mts;
    host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
    clock_get_time(cclock, &mts);
    mach_port_deallocate(mach_task_self(), cclock);
    cur_time.tv_sec = mts.tv_sec;
    cur_time.tv_nsec = mts.tv_nsec;
#endif
    Mobot_getJointAnglesTime(
        rArg->comms, 
        &rArg->time[i], 
        &rArg->angle[i], 
        &rArg->angle2[i], 
        &rArg->angle3[i], 
        &rArg->angle4[i]);
    if(i == 0) {
      start_time = rArg->time[i];
    }
    rArg->time[i] = rArg->time[i] - start_time;
    /* Convert angle to degrees */
    rArg->angle[i] = RAD2DEG(rArg->angle[i]);
    rArg->angle2[i] = RAD2DEG(rArg->angle2[i]);
    rArg->angle3[i] = RAD2DEG(rArg->angle3[i]);
    rArg->angle4[i] = RAD2DEG(rArg->angle4[i]);
#ifndef __MACH__
    clock_gettime(CLOCK_REALTIME, &itime);
#else
    host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
    clock_get_time(cclock, &mts);
    mach_port_deallocate(mach_task_self(), cclock);
    cur_time.tv_sec = mts.tv_sec;
    cur_time.tv_nsec = mts.tv_nsec;
#endif
    dt = diff_msecs(cur_time, itime);
    if(dt < (rArg->msecs)) {
      usleep(rArg->msecs*1000 - dt*1000);
    }
  }
  for(i = 0; i < 4; i++) {
    rArg->comms->recordingInProgress[i] = 0;
  }
#else
  recordAngleArg_t *rArg = (recordAngleArg_t*) arg;
  int i;
  DWORD cur_time, itime;
  unsigned int dt;
  double start_time;
  for(i = 0; i < rArg->num; i++) {
    cur_time = GetTickCount();
    Mobot_getJointAnglesTime(
        rArg->comms, 
        &rArg->time[i], 
        &rArg->angle[i], 
        &rArg->angle2[i], 
        &rArg->angle3[i], 
        &rArg->angle4[i]);
    if(i == 0) {
      start_time = rArg->time[i];
    }
    rArg->time[i] = rArg->time[i] - start_time;
    /* Convert angle to degrees */
    rArg->angle[i] = RAD2DEG(rArg->angle[i]);
    rArg->angle2[i] = RAD2DEG(rArg->angle2[i]);
    rArg->angle3[i] = RAD2DEG(rArg->angle3[i]);
    rArg->angle4[i] = RAD2DEG(rArg->angle4[i]);
    itime = GetTickCount();
    dt = itime - cur_time;
    if(dt < (rArg->msecs)) {
      Sleep(rArg->msecs - dt);
    }
  }
  for(i = 0; i < 4; i++) {
    rArg->comms->recordingInProgress[i] = 0;
  }
#endif
  return NULL;

}

int Mobot_recordWait(mobot_t* comms)
{
  int i;
  for(i = 0; i < 4; i++) {
    while(comms->recordingInProgress[i]) {
#ifndef _WIN32
      usleep(100000);
#else
	  Sleep(100);
#endif
    }
  }
  return 0;
}

int Mobot_setJointDirection(mobot_t* comms, robotJointId_t id, robotJointState_t dir)
{
  uint8_t buf[32];
  int status;
  buf[0] = (uint8_t)id-1;
  buf[1] = (uint8_t) dir;
  status = SendToIMobot(comms, BTCMD(CMD_SETMOTORDIR), buf, 2);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_setJointSafetyAngle(mobot_t* comms, double angle)
{
  uint8_t buf[32];
  float f;
  int status;
  f = angle;
  memcpy(&buf[0], &f, 4);
  status = SendToIMobot(comms, BTCMD(CMD_SETMOTORSAFETYLIMIT), buf, 4);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_setJointSafetyAngleTimeout(mobot_t* comms, double seconds)
{
  uint8_t buf[32];
  float f;
  int status;
  f = seconds;
  memcpy(&buf[0], &f, 4);
  status = SendToIMobot(comms, BTCMD(CMD_SETMOTORSAFETYTIMEOUT), buf, 4);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_setJointSpeed(mobot_t* comms, robotJointId_t id, double speed)
{
  uint8_t buf[32];
  float f;
  int status;
  f = speed;
  buf[0] = (uint8_t)id-1;
  memcpy(&buf[1], &f, 4);
  status = SendToIMobot(comms, BTCMD(CMD_SETMOTORSPEED), buf, 5);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  comms->jointSpeeds[id-1] = speed;
  return 0;
}

int Mobot_setJointSpeedRatio(mobot_t* comms, robotJointId_t id, double ratio)
{
  if((ratio < 0) || (ratio > 1)) {
    return -1;
  }
  return Mobot_setJointSpeed(comms, id, ratio * comms->maxSpeed[(int)id-1]);
}

int Mobot_setJointSpeedRatios(mobot_t* comms, double ratio1, double ratio2, double ratio3, double ratio4)
{
  double ratios[4];
  ratios[0] = ratio1;
  ratios[1] = ratio2;
  ratios[2] = ratio3;
  ratios[3] = ratio4;
  for(int i = 0; i < 4; i++) {
    Mobot_setJointSpeedRatio(comms, (robotJointId_t)(i+1), ratios[i]);
  }
  return 0;
}

int Mobot_setMotorPower(mobot_t* comms, robotJointId_t id, int power)
{
  char buf[160];
  int status;
  int bytes_read;
  /*
  sprintf(buf, "SET_MOTOR_POWER %d %d", id, power);
  status = SendToIMobot(comms, buf, strlen(buf)+1);
  if(status < 0) return status;
  bytes_read = RecvFromIMobot(comms, buf, sizeof(buf));
  if(strcmp(buf, "OK")) return -1;
  */
  return 0;
}

int Mobot_setTwoWheelRobotSpeed(mobot_t* comms, double speed, double radius)
{
  double omega;
  omega = speed/radius;
  Mobot_setJointSpeed(comms, ROBOT_JOINT1, omega);
  Mobot_setJointSpeed(comms, ROBOT_JOINT4, omega);
  return 0;
}

int Mobot_setJointSpeeds(mobot_t* comms, double speed1, double speed2, double speed3, double speed4)
{
  double speeds[4];
  speeds[0] = speed1;
  speeds[1] = speed2;
  speeds[2] = speed3;
  speeds[3] = speed4;
  for(int i = 0; i < 4; i++) {
    Mobot_setJointSpeed(comms, (robotJointId_t)(i+1), speeds[i]);
  }
  return 0;
}

int Mobot_stop(mobot_t* comms)
{
  uint8_t buf[32];
  float f;
  int status;
  status = SendToIMobot(comms, BTCMD(CMD_STOP), NULL, 0);
  if(status < 0) return status;
  if(RecvFromIMobot(comms, buf, sizeof(buf))) {
    return -1;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_motionArch(mobot_t* comms, double angle)
{
  Mobot_moveJointToNB(comms, ROBOT_JOINT2, -angle/2.0);
  Mobot_moveJointToNB(comms, ROBOT_JOINT3, angle/2.0);
  Mobot_moveJointWait(comms, ROBOT_JOINT2);
  Mobot_moveJointWait(comms, ROBOT_JOINT3);
  return 0;
}

void* motionArchThread(void* arg)
{
  mobot_t* comms = (mobot_t*)arg;
  Mobot_motionArch(comms, comms->motionArgDouble);
  comms->motionInProgress--;
  return NULL;
}

int Mobot_motionArchNB(mobot_t* comms, double angle)
{
  comms->motionArgDouble = angle;
  comms->motionInProgress++;
  THREAD_CREATE(comms->thread, motionArchThread, comms);
  return 0;
}

int Mobot_motionInchwormLeft(mobot_t* comms, int num)
{
  int i;
  Mobot_moveJointToNB(comms, ROBOT_JOINT2, 0);
  Mobot_moveJointToNB(comms, ROBOT_JOINT3, 0);
  Mobot_moveWait(comms);

  for(i = 0; i < num; i++) {
    Mobot_moveJointTo(comms, ROBOT_JOINT2, DEG2RAD(-50));
    Mobot_moveJointTo(comms, ROBOT_JOINT3, DEG2RAD(50));
    Mobot_moveJointTo(comms, ROBOT_JOINT2, 0);
    Mobot_moveJointTo(comms, ROBOT_JOINT3, 0);
  }

  return 0;
}

void* motionInchwormLeftThread(void* arg)
{
  mobot_t* comms = (mobot_t*)arg;
  Mobot_motionInchwormLeft(comms, comms->motionArgInt);
  comms->motionInProgress--;
  return NULL;
}

int Mobot_motionInchwormLeftNB(mobot_t* comms, int num)
{
  /* Make sure the old thread has joined */
  comms->motionInProgress++;
  comms->motionArgInt = num;
  THREAD_CREATE(comms->thread, motionInchwormLeftThread, comms);
  return 0;
}

int Mobot_motionInchwormRight(mobot_t* comms, int num)
{
  int i;
  Mobot_moveJointToNB(comms, ROBOT_JOINT2, 0);
  Mobot_moveJointToNB(comms, ROBOT_JOINT3, 0);
  Mobot_moveWait(comms);

  for(i = 0; i < num; i++) {
    Mobot_moveJointTo(comms, ROBOT_JOINT3, DEG2RAD(50));
    Mobot_moveJointTo(comms, ROBOT_JOINT2, DEG2RAD(-50));
    Mobot_moveJointTo(comms, ROBOT_JOINT3, 0);
    Mobot_moveJointTo(comms, ROBOT_JOINT2, 0);
  }

  return 0;
}

void* motionInchwormRightThread(void* arg)
{
  mobot_t* comms = (mobot_t*)arg;
  Mobot_motionInchwormRight((mobot_t*)arg, comms->motionArgInt);
  ((mobot_t*)arg)->motionInProgress--;
  return NULL;
}

int Mobot_motionInchwormRightNB(mobot_t* comms, int num)
{
  comms->motionArgInt = num;
  comms->motionInProgress++;
  THREAD_CREATE(comms->thread, motionInchwormRightThread, comms);
  return 0;
}

int Mobot_motionRollBackward(mobot_t* comms, double angle)
{
  double motorPosition[2];
  Mobot_getJointAngle(comms, ROBOT_JOINT1, &motorPosition[0]);
  Mobot_getJointAngle(comms, ROBOT_JOINT4, &motorPosition[1]);
  Mobot_moveJointToNB(comms, ROBOT_JOINT1, motorPosition[0] - angle);
  Mobot_moveJointToNB(comms, ROBOT_JOINT4, motorPosition[1] - angle);
  Mobot_moveWait(comms);
  return 0;
}

void* motionRollBackwardThread(void* arg)
{
  mobot_t* comms = (mobot_t*)arg;
  Mobot_motionRollBackward((mobot_t*)arg, comms->motionArgDouble);
  ((mobot_t*)arg)->motionInProgress--;
  return NULL;
}

int Mobot_motionRollBackwardNB(mobot_t* comms, double angle)
{
  comms->motionArgDouble = angle;
  comms->motionInProgress++;
  THREAD_CREATE(comms->thread, motionRollBackwardThread, comms);
  return 0;
}

int Mobot_motionRollForward(mobot_t* comms, double angle)
{
  double motorPosition[2];
  Mobot_getJointAngle(comms, ROBOT_JOINT1, &motorPosition[0]);
  Mobot_getJointAngle(comms, ROBOT_JOINT4, &motorPosition[1]);
  Mobot_moveJointToNB(comms, ROBOT_JOINT1, motorPosition[0] + angle);
  Mobot_moveJointToNB(comms, ROBOT_JOINT4, motorPosition[1] + angle);
  Mobot_moveWait(comms);
  return 0;
}

void* motionRollForwardThread(void* arg)
{
  mobot_t* comms = (mobot_t*)arg;
  Mobot_motionRollForward((mobot_t*)arg, comms->motionArgDouble);
  ((mobot_t*)arg)->motionInProgress--;
  return NULL;
}

int Mobot_motionRollForwardNB(mobot_t* comms, double angle)
{
  comms->motionArgDouble = angle;
  comms->motionInProgress++;
  THREAD_CREATE(comms->thread, motionRollForwardThread, comms);
  return 0;
}

int Mobot_motionStand(mobot_t* comms)
{
  double speed;
  Mobot_moveToZero(comms);
  Mobot_moveJointTo(comms, ROBOT_JOINT2, DEG2RAD(-85));
  Mobot_moveJointTo(comms, ROBOT_JOINT3, DEG2RAD(70));
  Mobot_moveWait(comms);
  Mobot_moveJointTo(comms, ROBOT_JOINT1, DEG2RAD(45));
  /* Sleep for a second, wait for it to settle down */
#ifdef _WIN32
  Sleep(1000);
#else
  sleep(1);
#endif
  Mobot_moveJointTo(comms, ROBOT_JOINT2, DEG2RAD(20));
  //Mobot_setJointSpeed(comms, ROBOT_JOINT2, speed);
  return 0;
}

void* motionStandThread(void* arg)
{
  Mobot_motionStand((mobot_t*)arg);
  ((mobot_t*)arg)->motionInProgress--;
  return NULL;
}

int Mobot_motionSkinny(mobot_t* comms, double angle)
{
  Mobot_moveJointToNB(comms, ROBOT_JOINT2, angle);
  Mobot_moveJointToNB(comms, ROBOT_JOINT3, angle);
  Mobot_moveWait(comms);
  return 0;
}

void* motionSkinnyThread(void* arg)
{
  mobot_t* comms = (mobot_t*)arg;
  Mobot_motionSkinny((mobot_t*)arg, comms->motionArgDouble);
  ((mobot_t*)arg)->motionInProgress--;
  return NULL;
}

int Mobot_motionSkinnyNB(mobot_t* comms, double angle)
{
  comms->motionArgDouble = angle;
  comms->motionInProgress++;
  THREAD_CREATE(comms->thread, motionSkinnyThread, comms);
  return 0;
}

int Mobot_motionStandNB(mobot_t* comms)
{
  comms->motionInProgress++;
  THREAD_CREATE(comms->thread, motionStandThread, comms);
  return 0;
}

int Mobot_motionTurnLeft(mobot_t* comms, double angle)
{
  double motorPosition[2];
  Mobot_getJointAngle(comms, ROBOT_JOINT1, &motorPosition[0]);
  Mobot_getJointAngle(comms, ROBOT_JOINT4, &motorPosition[1]);
  Mobot_moveJointToNB(comms, ROBOT_JOINT1, motorPosition[0] - angle);
  Mobot_moveJointToNB(comms, ROBOT_JOINT4, motorPosition[1] + angle);
  Mobot_moveWait(comms);
  return 0;
}

void* motionTurnLeftThread(void* arg)
{
  mobot_t* comms = (mobot_t*)arg;
  Mobot_motionTurnLeft((mobot_t*)arg, comms->motionArgDouble);
  ((mobot_t*)arg)->motionInProgress--;
  return NULL;
}

int Mobot_motionTurnLeftNB(mobot_t* comms, double angle)
{
  comms->motionInProgress++;
  comms->motionArgDouble = angle;
  THREAD_CREATE(comms->thread, motionTurnLeftThread, comms);
  return 0;
}

int Mobot_motionTurnRight(mobot_t* comms, double angle)
{
  double motorPosition[2];
  Mobot_getJointAngle(comms, ROBOT_JOINT1, &motorPosition[0]);
  Mobot_getJointAngle(comms, ROBOT_JOINT4, &motorPosition[1]);
  Mobot_moveJointToNB(comms, ROBOT_JOINT1, motorPosition[0] + angle);
  Mobot_moveJointToNB(comms, ROBOT_JOINT4, motorPosition[1] - angle);
  Mobot_moveWait(comms);
  return 0;
}

void* motionTurnRightThread(void* arg)
{
  mobot_t* comms = (mobot_t*)arg;
  Mobot_motionTurnRight((mobot_t*)arg, comms->motionArgDouble);
  ((mobot_t*)arg)->motionInProgress--;
  return NULL;
}

int Mobot_motionTurnRightNB(mobot_t* comms, double angle)
{
  comms->motionArgDouble = angle;
  comms->motionInProgress++;
  THREAD_CREATE(comms->thread, motionTurnRightThread, comms);
  return 0;
}

int Mobot_motionTumbleRight(mobot_t* comms, int num)
{
  int i;
  Mobot_moveToZero(comms);
#ifndef _WIN32
  sleep(1);
#else
  Sleep(1000);
#endif

  for(i = 0; i < num; i++) {
    Mobot_moveJointTo(comms, ROBOT_JOINT3, DEG2RAD(85));
    Mobot_moveJointTo(comms, ROBOT_JOINT2, DEG2RAD(-80));
    Mobot_moveJointTo(comms, ROBOT_JOINT3, DEG2RAD(0));
    Mobot_moveJointTo(comms, ROBOT_JOINT2, DEG2RAD(0));
    Mobot_moveJointTo(comms, ROBOT_JOINT3, DEG2RAD(-80));
    Mobot_moveJointTo(comms, ROBOT_JOINT3, DEG2RAD(-45));
    Mobot_moveJointTo(comms, ROBOT_JOINT2, DEG2RAD(85));
    Mobot_moveJointTo(comms, ROBOT_JOINT3, DEG2RAD(-80));
    Mobot_moveJointTo(comms, ROBOT_JOINT2, DEG2RAD(0));
    Mobot_moveJointTo(comms, ROBOT_JOINT3, DEG2RAD(0));
    Mobot_moveJointTo(comms, ROBOT_JOINT2, DEG2RAD(-80));
    if(i != (num-1)) {
      Mobot_moveJointTo(comms, ROBOT_JOINT2, DEG2RAD(-45));
    }
  }
  Mobot_moveJointToNB(comms, ROBOT_JOINT3, 0);
  Mobot_moveJointToNB(comms, ROBOT_JOINT2, 0);
  Mobot_moveWait(comms);
  return 0;
}

void* motionTumbleRightThread(void* arg)
{
  mobot_t* comms = (mobot_t*)arg;
  Mobot_motionTumbleRight(comms, comms->motionArgInt);
  comms->motionInProgress--;
  return NULL;
}

int Mobot_motionTumbleRightNB(mobot_t* comms, int num)
{
  comms->motionArgInt = num;
  comms->motionInProgress++;
  THREAD_CREATE(comms->thread, motionTumbleRightThread, comms);
  return 0;
}

int Mobot_motionTumbleLeft(mobot_t* comms, int num)
{
  int i;
  Mobot_moveToZero(comms);
#ifndef _WIN32
  sleep(1);
#else
  Sleep(1000);
#endif

  for(i = 0; i < num; i++) {
    Mobot_moveJointTo(comms, ROBOT_JOINT2, DEG2RAD(-85));
    Mobot_moveJointTo(comms, ROBOT_JOINT3, DEG2RAD(80));
    Mobot_moveJointTo(comms, ROBOT_JOINT2, DEG2RAD(0));
    Mobot_moveJointTo(comms, ROBOT_JOINT3, DEG2RAD(0));
    Mobot_moveJointTo(comms, ROBOT_JOINT2, DEG2RAD(80));
    Mobot_moveJointTo(comms, ROBOT_JOINT2, DEG2RAD(45));
    Mobot_moveJointTo(comms, ROBOT_JOINT3, DEG2RAD(-85));
    Mobot_moveJointTo(comms, ROBOT_JOINT2, DEG2RAD(80));
    Mobot_moveJointTo(comms, ROBOT_JOINT3, DEG2RAD(0));
    Mobot_moveJointTo(comms, ROBOT_JOINT2, DEG2RAD(0));
    Mobot_moveJointTo(comms, ROBOT_JOINT3, DEG2RAD(80));
    if(i != (num-1)) {
      Mobot_moveJointTo(comms, ROBOT_JOINT3, DEG2RAD(45));
    }
  }
  Mobot_moveJointToNB(comms, ROBOT_JOINT2, 0);
  Mobot_moveJointToNB(comms, ROBOT_JOINT3, 0);
  Mobot_moveWait(comms);
  return 0;
}

void* motionTumbleLeftThread(void* arg)
{
  mobot_t* comms = (mobot_t*)arg;
  Mobot_motionTumbleLeft(comms, comms->motionArgInt);
  comms->motionInProgress--;
  return NULL;
}

int Mobot_motionTumbleLeftNB(mobot_t* comms, int num)
{
  comms->motionArgInt = num;
  comms->motionInProgress++;
  THREAD_CREATE(comms->thread, motionTumbleLeftThread, comms);
  return 0;
}

int Mobot_motionUnstand(mobot_t* comms)
{
  double speed;
  Mobot_moveToZero(comms);
  Mobot_moveJointToNB(comms, ROBOT_JOINT3, DEG2RAD(45));
  Mobot_moveJointToNB(comms, ROBOT_JOINT2, DEG2RAD(-85));
  Mobot_moveWait(comms);
  Mobot_moveToZero(comms);
  return 0;
}

void* motionUnstandThread(void* arg)
{
  Mobot_motionUnstand((mobot_t*)arg);
  ((mobot_t*)arg)->motionInProgress--;
  return NULL;
}

int Mobot_motionUnstandNB(mobot_t* comms)
{
  comms->motionInProgress++;
  THREAD_CREATE(comms->thread, motionUnstandThread, comms);
  return 0;
}

int Mobot_motionWait(mobot_t* comms)
{
  while(comms->motionInProgress) {
#ifdef _WIN32
    Sleep(200);
#else
    usleep(200000);
#endif
  }
  Mobot_moveWait(comms);
  return 0;
}

#ifdef _WIN32
void baswap(bdaddr_t *dst, const bdaddr_t *src)
{
	register unsigned char *d = (unsigned char *) dst;
	register const unsigned char *s = (const unsigned char *) src;
	register int i;

	for (i = 0; i < 6; i++)
		d[i] = s[5-i];
}

int str2ba(const char *str, bdaddr_t *ba)
{
	UINT8 b[6];
	const char *ptr = str;
	int i;

	for (i = 0; i < 6; i++) {
		b[i] = (UINT8) strtol(ptr, NULL, 16);
		if (i != 5 && !(ptr = strchr(ptr, ':')))
			ptr = ":00:00:00:00:00";
		ptr++;
	}

	baswap(ba, (bdaddr_t *) b);

	return 0;
}
#endif

int SendToIMobot(mobot_t* comms, uint8_t cmd, const void* data, int datasize)
{
  int err = 0;
  int i;
  int len;
  uint8_t str[64];
  if(comms->connected == 0) {
    return -1;
  }
  MUTEX_LOCK(comms->commsLock);
  comms->recvBuf_ready = 0;
  str[0] = cmd;
  str[1] = datasize + 3;
  if(datasize > 0) memcpy(&str[2], data, datasize);
  str[datasize+2] = MSG_SENDEND;
  len = datasize + 3;
#if 0
  char* str;
  str = (char*)malloc(strlen(buf)+2);
  strcpy(str, buf);
  strcat(str, "$");
  len++;
#endif
  //printf("SEND %d: <<%s>>\n", comms->socket, str);
  /*
  printf("SEND: ");
  for(i = 0; i < len; i++) {
    printf("0x%x ", str[i]);
  }
  printf("\n");
  */
  /* To send to the iMobot, we need to append the terminating character, '$' */
  if(comms->connected == 1) {
#ifdef _WIN32
    err = send(comms->socket, (const char*)str, len, 0);
#else
    err = write(comms->socket, str, len);
#endif
  } else if (comms->connected == 2) {
    err = -1;
  } else {
    err = -1;
  }
  if(err < 0) {
    return err;
  } else {
    return 0;
  }
}

int RecvFromIMobot(mobot_t* comms, uint8_t* buf, int size)
{
  /* Wait until transaction is ready */
  MUTEX_LOCK(comms->recvBuf_lock);
  while(!comms->recvBuf_ready) {
    COND_WAIT(comms->recvBuf_cond, comms->recvBuf_lock);
  }
  memcpy(buf, comms->recvBuf, comms->recvBuf_bytes);

  /* Print out results */
  /*
  int i;
  for(i = 0; i < buf[1]; i++) {
    printf("0x%2x ", buf[i]);
  }
  printf("\n");
  */

  MUTEX_UNLOCK(comms->recvBuf_lock);
  MUTEX_UNLOCK(comms->commsLock);
  return 0;
}

int RecvFromIMobot2(mobot_t* comms, char* buf, int size)
{
  int err = 0;
  int i = 0;
  int j = 0;
  int done = 0;
  int tries = 100;
  char tmp[256];
  buf[0] = '\0';
#ifdef _WIN32
  DWORD bytes;
#endif
  while(done == 0) {
    if(comms->connected == 1) {
#ifdef _WIN32
      fd_set fds;
      int n;
      struct timeval tv;
      /* Set up file descriptor set */
      FD_ZERO(&fds);
      FD_SET(comms->socket, &fds);
      /* Set up timeval for the timeout */
      tv.tv_sec = 1;
      tv.tv_usec = 0;
      /* Wait until timeout or data received */
      n = select(comms->socket, &fds, NULL, NULL, &tv);
      if(n == 0) {
        /* Timeout */
		MUTEX_UNLOCK(comms->commsLock);
        return -1;
      }
      if(n == -1) {
        /* Error */
		MUTEX_UNLOCK(comms->commsLock);
        return -1;
      }
      err = recvfrom(comms->socket, tmp, 256, 0, (struct sockaddr*)0, 0);
#else
      while(tries >= 0) {
        err = read(comms->socket, tmp, 255);
        if(err < 0) {
          tries--;
          //printf("*");
          usleep(1000);
        } else {
          break;
        }
      }
#endif
    } else if (comms->connected == 2) {
      err = -1;
    } else {
      err = -1;
      return -1;
    }
    if(err < 0) break;
    for(i = 0; i < err; i++, j++) {
      buf[j] = tmp[i];
      if(tmp[i] == '\0') 
      {
        done = 1;
        break;
      }
    }
    tries = 100;
  }
  //printf("RECV %d: <<%s>>\n", comms->socket, buf);
  MUTEX_UNLOCK(comms->commsLock);
  if(err < 0) {
    return err;
  } else {
    return 0;
  }
}

/* The comms engine will watch the incoming comm channel for any message. If a
 * message is expected, it will get the data to RecvFromIMobot(). If it was
 * triggered by an event, then the appropriate callback will be called. */
void* commsEngine(void* arg)
{
  mobot_t* comms = (mobot_t*)arg;
  uint8_t byte;
  int bytes = 0;
  int err;
  int isResponse;

  while(1) {
    /* Try and receive a byte */
#ifndef _WIN32
    err = read(comms->socket, &byte, 1);
#else
    err = recvfrom(comms->socket, (char*)&byte, 1, 0, (struct sockaddr*)0, 0);
#endif
    /* If we are no longer connected, just return */
    if(comms->connected == 0) {
      return NULL;
    }
    if(err < 0) {
      continue;
    }
    /* Received a byte. If it is the first one, check to see if it is a
     * response or a triggered event */
    if(bytes == 0) {
      MUTEX_LOCK(comms->commsBusy_lock);
      comms->commsBusy = 1;
      COND_SIGNAL(comms->commsBusy_cond);
      MUTEX_UNLOCK(comms->commsBusy_lock);
      if( (byte == RESP_OK) ||
          (byte == RESP_ERR)
          ) {
        isResponse = 1;
      } else {
        isResponse = 0;
      }
    }
    if(isResponse) {
      MUTEX_LOCK(comms->recvBuf_lock);
      comms->recvBuf[bytes] = byte;
      bytes++;
      MUTEX_UNLOCK(comms->recvBuf_lock);
      if( (bytes >= 2) &&
          (comms->recvBuf[1] == bytes) )
      {
        /* We have received the entire response */
        MUTEX_LOCK(comms->recvBuf_lock);
        comms->recvBuf_ready = 1;
        comms->recvBuf_bytes = bytes;
        COND_BROADCAST(comms->recvBuf_cond);
        MUTEX_UNLOCK(comms->recvBuf_lock);
        /* Reset state vars */
        bytes = 0;
        MUTEX_LOCK(comms->commsBusy_lock);
        comms->commsBusy = 0;
        COND_SIGNAL(comms->commsBusy_cond);
        MUTEX_UNLOCK(comms->commsBusy_lock);
      }
    } else {
      /* It was a user triggered event */
      MUTEX_LOCK(comms->recvBuf_lock);
      comms->recvBuf[bytes] = byte;
      bytes++;
      MUTEX_UNLOCK(comms->recvBuf_lock);
      if( (bytes >= 2) &&
          (comms->recvBuf[1] == bytes) )
      {
        /* We got the entire message */
        MUTEX_LOCK(comms->callback_lock);
        if(comms->callbackEnabled) {
          /* Call the callback multiple times depending on the events */
          int bit;
          uint8_t events = comms->recvBuf[6];
          uint8_t buttonDown = comms->recvBuf[7];
          THREAD_T callbackThreadHandle;
          callbackArg_t* callbackArg;
          for(bit = 0; bit < 2; bit++) {
            if(events & (1<<bit)) {
              callbackArg = (callbackArg_t*)malloc(sizeof(callbackArg_t));
              callbackArg->comms = comms;
              callbackArg->button = bit;
              callbackArg->buttonDown = (buttonDown & (1<<bit)) ? 1 : 0;
              //comms->buttonCallback(bit, (buttonDown & (1<<bit)) ? 1 : 0 );
              THREAD_CREATE(&callbackThreadHandle, callbackThread, callbackArg);
            }
          }
        }
        /* Reset state vars */
        bytes = 0;
        MUTEX_LOCK(comms->commsBusy_lock);
        comms->commsBusy = 0;
        COND_SIGNAL(comms->commsBusy_cond);
        MUTEX_UNLOCK(comms->commsBusy_lock);
        MUTEX_UNLOCK(comms->callback_lock);
      }
    }
  }
  return NULL;
}

void* callbackThread(void* arg)
{
  callbackArg_t *cbArg = (callbackArg_t*)arg;
  /* Just run the callback function */
  cbArg->comms->buttonCallback(
      cbArg->comms->mobot,
      cbArg->button,
      cbArg->buttonDown );
  free(cbArg);
  return NULL;
}

#ifndef C_ONLY

CMobot::CMobot()
{
  _comms = (mobot_t*)malloc(sizeof(mobot_t));
  Mobot_init(_comms);
}

CMobot::~CMobot()
{
  stop();
  if(_comms->connected) {
    disconnect();
  }
}

int CMobot::blinkLED(double delay, int numBlinks)
{
  return Mobot_blinkLED(_comms, delay, numBlinks);
}

int CMobot::connect()
{
  return Mobot_connect(_comms);
}

int CMobot::connectWithAddress(const char* address, int channel)
{
  return Mobot_connectWithAddress(_comms, address, channel);
}

#ifndef _WIN32
int CMobot::connectWithTTY(const char* ttyfilename)
{
  return Mobot_connectWithTTY(_comms, ttyfilename);
}
#endif

int CMobot::disconnect()
{
  return Mobot_disconnect(_comms);
}

int CMobot::enableButtonCallback(void (*buttonCallback)(CMobot* robot, int button, int buttonDown))
{
  return Mobot_enableButtonCallback(
      _comms,
      this,
      (void(*)(void*,int,int))buttonCallback);
}

int CMobot::disableButtonCallback()
{
  return Mobot_disableButtonCallback(_comms);
}

int CMobot::isConnected()
{
  return Mobot_isConnected(_comms);
}

int CMobot::isMoving()
{
  return Mobot_isMoving(_comms);
}

const char* CMobot::getConfigFilePath()
{
  return Mobot_getConfigFilePath();
}

int CMobot::getJointAngle(robotJointId_t id, double &angle)
{
  int err;
  err = Mobot_getJointAngle(_comms, id, &angle);
  angle = RAD2DEG(angle);
  return err;
}

int CMobot::getJointAngleAbs(robotJointId_t id, double &angle)
{
  int err;
  err = Mobot_getJointAngleAbs(_comms, id, &angle);
  angle = RAD2DEG(angle);
  return err;
}

int CMobot::getJointAngles(
    double &angle1,
    double &angle2,
    double &angle3,
    double &angle4)
{
  double time;
  int err;
  err = Mobot_getJointAnglesTime(
      _comms, 
      &time,
      &angle1,
      &angle2,
      &angle3,
      &angle4);
  if(err) return err;
  angle1 = RAD2DEG(angle1);
  angle2 = RAD2DEG(angle2);
  angle3 = RAD2DEG(angle3);
  angle4 = RAD2DEG(angle4);
  return 0;
}

int CMobot::getJointAnglesAbs(
    double &angle1,
    double &angle2,
    double &angle3,
    double &angle4)
{
  double time;
  int err;
  err = Mobot_getJointAnglesAbsTime(
      _comms, 
      &time,
      &angle1,
      &angle2,
      &angle3,
      &angle4);
  if(err) return err;
  angle1 = RAD2DEG(angle1);
  angle2 = RAD2DEG(angle2);
  angle3 = RAD2DEG(angle3);
  angle4 = RAD2DEG(angle4);
  return 0;
}

int CMobot::getJointDirection(robotJointId_t id, robotJointState_t &dir)
{
  return Mobot_getJointDirection(_comms, id, &dir);
}

int CMobot::getJointMaxSpeed(robotJointId_t id, double &maxSpeed)
{
  int err = Mobot_getJointMaxSpeed(_comms, id, &maxSpeed);
  maxSpeed = RAD2DEG(maxSpeed);
  return err;
}

int CMobot::getJointSafetyAngle(double &angle)
{
  int r;
  double a;
  r = Mobot_getJointSafetyAngle(_comms, &a);
  angle = RAD2DEG(a);
  return r;
}

int CMobot::getJointSafetyAngleTimeout(double &seconds)
{
  return Mobot_getJointSafetyAngleTimeout(_comms, &seconds);
}

int CMobot::getJointSpeed(robotJointId_t id, double &speed)
{
  int err;
  err = Mobot_getJointSpeed(_comms, id, &speed);
  speed = RAD2DEG(speed);
  return err;
}

int CMobot::getJointSpeedRatio(robotJointId_t id, double &ratio)
{
  return Mobot_getJointSpeedRatio(_comms, id, &ratio);
}

int CMobot::getJointSpeeds(double &speed1, double &speed2, double &speed3, double &speed4)
{
  int i;
  int err = Mobot_getJointSpeeds(_comms, &speed1, &speed2, &speed3, &speed4);
  speed1 = RAD2DEG(speed1);
  speed2 = RAD2DEG(speed2);
  speed3 = RAD2DEG(speed3);
  speed4 = RAD2DEG(speed4);
  return err;
}

int CMobot::getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3, double &ratio4)
{
  return Mobot_getJointSpeedRatios(_comms, &ratio1, &ratio2, &ratio3, &ratio4);
}

int CMobot::getJointState(robotJointId_t id, robotJointState_t &state)
{
  return Mobot_getJointState(_comms, id, &state);
}

mobot_t* CMobot::getMobotObject()
{
  return _comms;
}

int CMobot::move( double angle1,
                        double angle2,
                        double angle3,
                        double angle4)
{
  return Mobot_move(
      _comms, 
      DEG2RAD(angle1), 
      DEG2RAD(angle2), 
      DEG2RAD(angle3), 
      DEG2RAD(angle4));
}

int CMobot::moveNB( double angle1,
                        double angle2,
                        double angle3,
                        double angle4)
{
  return Mobot_moveNB(
      _comms, 
      DEG2RAD(angle1), 
      DEG2RAD(angle2), 
      DEG2RAD(angle3), 
      DEG2RAD(angle4));
}

int CMobot::moveContinuousNB( robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, robotJointState_t dir4)
{
  return Mobot_moveContinuousNB(_comms, dir1, dir2, dir3, dir4);
}

int CMobot::moveContinuousTime( robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, robotJointState_t dir4, double seconds)
{
  return Mobot_moveContinuousTime(_comms, dir1, dir2, dir3, dir4, seconds);
}

int CMobot::moveJointContinuousNB(robotJointId_t id, robotJointState_t dir)
{
  return Mobot_moveJointContinuousNB(_comms, id, dir);
}

int CMobot::moveJointContinuousTime(robotJointId_t id, robotJointState_t dir, double seconds)
{
  return Mobot_moveJointContinuousTime(_comms, id, dir, seconds);
}

int CMobot::moveJoint(robotJointId_t id, double angle)
{
  return Mobot_moveJoint(_comms, id, DEG2RAD(angle));
}

int CMobot::moveJointNB(robotJointId_t id, double angle)
{
  return Mobot_moveJointNB(_comms, id, DEG2RAD(angle));
}

int CMobot::moveJointTo(robotJointId_t id, double angle)
{
  return Mobot_moveJointTo(_comms, id, DEG2RAD(angle));
}

int CMobot::moveJointToAbs(robotJointId_t id, double angle)
{
  return Mobot_moveJointToAbs(_comms, id, DEG2RAD(angle));
}

int CMobot::moveJointToDirect(robotJointId_t id, double angle)
{
  return Mobot_moveJointToDirect(_comms, id, DEG2RAD(angle));
}

int CMobot::moveJointToNB(robotJointId_t id, double angle)
{
  return Mobot_moveJointToNB(_comms, id, DEG2RAD(angle));
}

int CMobot::moveJointToAbsNB(robotJointId_t id, double angle)
{
  return Mobot_moveJointToAbsNB(_comms, id, DEG2RAD(angle));
}

int CMobot::moveJointToDirectNB(robotJointId_t id, double angle)
{
  return Mobot_moveJointToDirectNB(_comms, id, DEG2RAD(angle));
}

int CMobot::moveJointToPIDNB(robotJointId_t id, double angle)
{
  return Mobot_moveJointToPIDNB(_comms, id, DEG2RAD(angle));
}

int CMobot::moveJointWait(robotJointId_t id)
{
  return Mobot_moveJointWait(_comms, id);
}

int CMobot::moveTo( double angle1,
                          double angle2,
                          double angle3,
                          double angle4)
{
  return Mobot_moveTo(
      _comms, 
      DEG2RAD(angle1), 
      DEG2RAD(angle2), 
      DEG2RAD(angle3), 
      DEG2RAD(angle4));
}

int CMobot::moveToAbs( double angle1,
                          double angle2,
                          double angle3,
                          double angle4)
{
  return Mobot_moveToAbs(
      _comms, 
      DEG2RAD(angle1), 
      DEG2RAD(angle2), 
      DEG2RAD(angle3), 
      DEG2RAD(angle4));
}

int CMobot::moveToDirect( double angle1,
                          double angle2,
                          double angle3,
                          double angle4)
{
  return Mobot_moveToDirect(
      _comms, 
      DEG2RAD(angle1), 
      DEG2RAD(angle2), 
      DEG2RAD(angle3), 
      DEG2RAD(angle4));
}

int CMobot::moveToPID( double angle1,
                          double angle2,
                          double angle3,
                          double angle4)
{
  return Mobot_moveToPID(
      _comms, 
      DEG2RAD(angle1), 
      DEG2RAD(angle2), 
      DEG2RAD(angle3), 
      DEG2RAD(angle4));
}

int CMobot::moveToNB( double angle1,
                          double angle2,
                          double angle3,
                          double angle4)
{
  return Mobot_moveToNB(
      _comms, 
      DEG2RAD(angle1), 
      DEG2RAD(angle2), 
      DEG2RAD(angle3), 
      DEG2RAD(angle4));
}

int CMobot::moveToAbsNB( double angle1,
                          double angle2,
                          double angle3,
                          double angle4)
{
  return Mobot_moveToAbsNB(
      _comms, 
      DEG2RAD(angle1), 
      DEG2RAD(angle2), 
      DEG2RAD(angle3), 
      DEG2RAD(angle4));
}

int CMobot::moveToDirectNB( double angle1,
                          double angle2,
                          double angle3,
                          double angle4)
{
  return Mobot_moveToDirectNB(
      _comms, 
      DEG2RAD(angle1), 
      DEG2RAD(angle2), 
      DEG2RAD(angle3), 
      DEG2RAD(angle4));
}

int CMobot::moveToPIDNB( double angle1,
                          double angle2,
                          double angle3,
                          double angle4)
{
  return Mobot_moveToPIDNB(
      _comms, 
      DEG2RAD(angle1), 
      DEG2RAD(angle2), 
      DEG2RAD(angle3), 
      DEG2RAD(angle4));
}

int CMobot::moveWait()
{
  return Mobot_moveWait(_comms);
}

int CMobot::moveToZero()
{
  return Mobot_moveToZero(_comms);
}

int CMobot::moveToZeroNB()
{
  return Mobot_moveToZeroNB(_comms);
}

int CMobot::recordAngle(robotJointId_t id, double* time, double* angle, int num, double seconds)
{
  return Mobot_recordAngle(_comms, id, time, angle, num, seconds);
}

int CMobot::recordAngles(double *time, 
    double *angle1, 
    double *angle2, 
    double *angle3, 
    double *angle4, 
    int num, 
    double seconds)
{
  return Mobot_recordAngles(_comms, time, angle1, angle2, angle3, angle4, num, seconds);
}

int CMobot::recordWait()
{
  return Mobot_recordWait(_comms);
}

int CMobot::setJointSafetyAngle(double angle)
{
  angle = DEG2RAD(angle);
  return Mobot_setJointSafetyAngle(_comms, angle);
}

int CMobot::setJointSafetyAngleTimeout(double seconds)
{
  return Mobot_setJointSafetyAngleTimeout(_comms, seconds);
}

int CMobot::setJointDirection(robotJointId_t id, robotJointState_t dir)
{
  return Mobot_setJointDirection(_comms, id, dir);
}

int CMobot::setJointSpeed(robotJointId_t id, double speed)
{
  return Mobot_setJointSpeed(_comms, id, DEG2RAD(speed));
}

int CMobot::setJointSpeeds(double speed1, double speed2, double speed3, double speed4)
{
  return Mobot_setJointSpeeds(
      _comms, 
      DEG2RAD(speed1), 
      DEG2RAD(speed2), 
      DEG2RAD(speed3), 
      DEG2RAD(speed4));
}

int CMobot::setJointSpeedRatio(robotJointId_t id, double ratio)
{
  return Mobot_setJointSpeedRatio(_comms, id, ratio);
}

int CMobot::setJointSpeedRatios(double ratio1, double ratio2, double ratio3, double ratio4)
{
  return Mobot_setJointSpeedRatios(_comms, ratio1, ratio2, ratio3, ratio4);
}

int CMobot::setMotorPower(robotJointId_t id, int power)
{
  return Mobot_setMotorPower(_comms, id, power);
}

int CMobot::setTwoWheelRobotSpeed(double speed, double radius)
{
  return Mobot_setTwoWheelRobotSpeed(_comms, speed, radius);
}

int CMobot::stop()
{
  return Mobot_stop(_comms);
}

int CMobot::motionArch(double angle)
{
  return Mobot_motionArch(_comms, DEG2RAD(angle));
}

int CMobot::motionArchNB(double angle)
{
  return Mobot_motionArchNB(_comms, DEG2RAD(angle));
}

int CMobot::motionInchwormLeft(int num)
{
  return Mobot_motionInchwormLeft(_comms, num);
}

int CMobot::motionInchwormLeftNB(int num)
{
  return Mobot_motionInchwormLeftNB(_comms, num);
}

int CMobot::motionInchwormRight(int num)
{
  return Mobot_motionInchwormRight(_comms, num);
}

int CMobot::motionInchwormRightNB(int num)
{
  return Mobot_motionInchwormRightNB(_comms, num);
}

int CMobot::motionRollBackward(double angle)
{
  return Mobot_motionRollBackward(_comms, DEG2RAD(angle));
}

int CMobot::motionRollBackwardNB(double angle)
{
  return Mobot_motionRollBackwardNB(_comms, DEG2RAD(angle));
}

int CMobot::motionRollForward(double angle)
{
  return Mobot_motionRollForward(_comms, DEG2RAD(angle));
}

int CMobot::motionRollForwardNB(double angle)
{
  return Mobot_motionRollForwardNB(_comms, DEG2RAD(angle));
}

int CMobot::motionSkinny(double angle)
{
  return Mobot_motionSkinny(_comms, DEG2RAD(angle));
}

int CMobot::motionSkinnyNB(double angle)
{
  return Mobot_motionSkinnyNB(_comms, DEG2RAD(angle));
}

int CMobot::motionStand()
{
  return Mobot_motionStand(_comms);
}

int CMobot::motionStandNB()
{
  return Mobot_motionStandNB(_comms);
}

int CMobot::motionTumbleRight(int num)
{
  return Mobot_motionTumbleRight(_comms, num);
}

int CMobot::motionTumbleRightNB(int num)
{
  return Mobot_motionTumbleRightNB(_comms, num);
}

int CMobot::motionTumbleLeft(int num)
{
  return Mobot_motionTumbleLeft(_comms, num);
}

int CMobot::motionTumbleLeftNB(int num)
{
  return Mobot_motionTumbleLeftNB(_comms, num);
}

int CMobot::motionTurnLeft(double angle)
{
  return Mobot_motionTurnLeft(_comms, DEG2RAD(angle));
}

int CMobot::motionTurnLeftNB(double angle)
{
  return Mobot_motionTurnLeftNB(_comms, DEG2RAD(angle));
}

int CMobot::motionTurnRight(double angle)
{
  return Mobot_motionTurnRight(_comms, DEG2RAD(angle));
}

int CMobot::motionTurnRightNB(double angle)
{
  return Mobot_motionTurnRightNB(_comms, DEG2RAD(angle));
}

int CMobot::motionUnstand()
{
  return Mobot_motionUnstand(_comms);
}

int CMobot::motionUnstandNB()
{
  return Mobot_motionUnstandNB(_comms);
}

int CMobot::motionWait()
{
  return Mobot_motionWait(_comms);
}

CMobotGroup::CMobotGroup()
{
  _numRobots = 0;
  _motionInProgress = 0;
  _thread = (THREAD_T*)malloc(sizeof(THREAD_T));
}

CMobotGroup::~CMobotGroup()
{
}

int CMobotGroup::addRobot(CMobot& robot)
{
  _robots[_numRobots] = &robot;
  _numRobots++;
  return 0;
}

int CMobotGroup::isMoving()
{
  for(int i = 0; i < _numRobots; i++) {
    if(_robots[i]->isMoving()) {
      return 1;
    }
  }
  return 0;
}

int CMobotGroup::move(double angle1, double angle2, double angle3, double angle4)
{
  moveNB(angle1, angle2, angle3, angle4);
  return moveWait();
}

int CMobotGroup::moveNB(double angle1, double angle2, double angle3, double angle4)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveNB(angle1, angle2, angle3, angle4);
  }
  return 0;
} 

int CMobotGroup::moveContinuousNB(robotJointState_t dir1, 
                       robotJointState_t dir2, 
                       robotJointState_t dir3, 
                       robotJointState_t dir4)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveContinuousNB(dir1, dir2, dir3, dir4);
  }
  return 0;
}

int CMobotGroup::moveContinuousTime(robotJointState_t dir1, 
                           robotJointState_t dir2, 
                           robotJointState_t dir3, 
                           robotJointState_t dir4, 
                           double seconds)
{
  int msecs = seconds * 1000.0;
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveContinuousNB(dir1, dir2, dir3, dir4);
  }
#ifdef _WIN32
  Sleep(msecs);
#else
  usleep(msecs*1000);
#endif
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->stop();
  }
  return 0;
}

int CMobotGroup::moveJointContinuousNB(robotJointId_t id, robotJointState_t dir)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveJointContinuousNB(id, dir);
  }
  return 0;
}

int CMobotGroup::moveJointContinuousTime(robotJointId_t id, robotJointState_t dir, double seconds)
{
  int msecs = seconds * 1000.0;
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveJointContinuousNB(id, dir);
  }
#ifdef _WIN32
  Sleep(msecs);
#else
  usleep(msecs * 1000);
#endif
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->stop();
  }
  return 0;
}

int CMobotGroup::moveJointTo(robotJointId_t id, double angle)
{
  moveJointToNB(id, angle);
  return moveWait();
}

int CMobotGroup::moveJointToNB(robotJointId_t id, double angle)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveJointToNB(id, angle);
  }
  return 0;
}

int CMobotGroup::moveJointWait(robotJointId_t id)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveJointWait(id);
  }
  return 0;
}

int CMobotGroup::moveTo(double angle1, double angle2, double angle3, double angle4)
{
  moveToNB(angle1, angle2, angle3, angle4);
  return moveWait();
}

int CMobotGroup::moveToNB(double angle1, double angle2, double angle3, double angle4)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveToNB(angle1, angle2, angle3, angle4);
  }
  return 0;
}

int CMobotGroup::moveWait()
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveWait();
  }
  return 0;
}

int CMobotGroup::moveToZeroNB()
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->moveToZeroNB();
  }
  return 0;
}

int CMobotGroup::moveToZero()
{
  moveToZeroNB();
  return moveWait();
}

int CMobotGroup::setJointSpeed(robotJointId_t id, double speed)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setJointSpeed(id, speed);
  }
  return 0;
}

int CMobotGroup::setJointSpeeds(double speed1, double speed2, double speed3, double speed4)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setJointSpeeds(speed1, speed2, speed3, speed4);
  }
  return 0;
}

int CMobotGroup::setJointSpeedRatio(robotJointId_t id, double ratio)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setJointSpeedRatio(id, ratio);
  }
  return 0;
}

int CMobotGroup::setJointSpeedRatios(double ratio1, double ratio2, double ratio3, double ratio4)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setJointSpeedRatios(ratio1, ratio2, ratio3, ratio4);
  }
  return 0;
}

int CMobotGroup::setTwoWheelRobotSpeed(double speed, double radius)
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->setTwoWheelRobotSpeed(speed, radius);
  }
  return 0;
}

int CMobotGroup::stop()
{
  for(int i = 0; i < _numRobots; i++) {
    _robots[i]->stop();
  }
  return 0;
}

int CMobotGroup::motionArch(double angle) {
  argDouble = angle;
  _motionInProgress++;
  motionArchThread(this);
  return 0;
}

int CMobotGroup::motionArchNB(double angle) {
  argDouble = angle;
  _motionInProgress++;
  THREAD_CREATE(_thread, motionArchThread, this);
  return 0;
}

void* CMobotGroup::motionArchThread(void* arg) 
{
  CMobotGroup *cmg = (CMobotGroup*)arg;
  cmg->moveJointToNB(ROBOT_JOINT2, -cmg->argDouble/2);
  cmg->moveJointToNB(ROBOT_JOINT3, cmg->argDouble/2);
  cmg->moveJointWait(ROBOT_JOINT2);
  cmg->moveJointWait(ROBOT_JOINT3);
  cmg->_motionInProgress--;
  return NULL;
}

int CMobotGroup::motionInchwormLeft(int num)
{
  argInt = num;
  _motionInProgress++;
  motionInchwormLeftThread(this);
  return 0;
}

int CMobotGroup::motionInchwormLeftNB(int num)
{
  argInt = num;
  _motionInProgress++;
  THREAD_CREATE(_thread, motionInchwormLeftThread, this);
  return 0;
}

void* CMobotGroup::motionInchwormLeftThread(void* arg)
{
  int i;
  CMobotGroup *cmg = (CMobotGroup*)arg;
  cmg->moveJointToNB(ROBOT_JOINT2, 0);
  cmg->moveJointToNB(ROBOT_JOINT3, 0);
  cmg->moveWait();
  for(i = 0; i < cmg->argInt ; i++) {
    cmg->moveJointTo(ROBOT_JOINT2, -50);
    cmg->moveJointTo(ROBOT_JOINT3, 50);
    cmg->moveJointTo(ROBOT_JOINT2, 0);
    cmg->moveJointTo(ROBOT_JOINT3, 0);
  }
  cmg->_motionInProgress--;
  return NULL;
}

int CMobotGroup::motionInchwormRight(int num)
{
  argInt = num;
  _motionInProgress++;
  motionInchwormRightThread(this);
  return 0;
}

int CMobotGroup::motionInchwormRightNB(int num)
{
  argInt = num;
  _motionInProgress++;
  THREAD_CREATE(_thread, motionInchwormRightThread, this);
  return 0;
}

void* CMobotGroup::motionInchwormRightThread(void* arg)
{
  int i;
  CMobotGroup *cmg = (CMobotGroup*)arg;

  cmg->moveJointToNB(ROBOT_JOINT2, 0);
  cmg->moveJointToNB(ROBOT_JOINT3, 0);
  cmg->moveWait();
  for(i = 0; i < cmg->argInt; i++) {
    cmg->moveJointTo(ROBOT_JOINT3, 50);
    cmg->moveJointTo(ROBOT_JOINT2, -50);
    cmg->moveJointTo(ROBOT_JOINT3, 0);
    cmg->moveJointTo(ROBOT_JOINT2, 0);
  }
  cmg->_motionInProgress--;
  return NULL;
}

int CMobotGroup::motionRollBackward(double angle)
{
  argDouble = angle;
  _motionInProgress++;
  motionRollBackwardThread(this);
  return 0;
}

int CMobotGroup::motionRollBackwardNB(double angle)
{
  argDouble = angle;
  _motionInProgress++;
  THREAD_CREATE(_thread, motionRollBackwardThread, this);
  return 0;
}

void* CMobotGroup::motionRollBackwardThread(void* arg)
{
  CMobotGroup *cmg = (CMobotGroup*)arg;
  cmg->move(-cmg->argDouble, 0, 0, -cmg->argDouble);
  cmg->_motionInProgress--;
  return NULL;
}

int CMobotGroup::motionRollForward(double angle)
{
  argDouble = angle;
  _motionInProgress++;
  motionRollForwardThread(this);
  return 0;
}

int CMobotGroup::motionRollForwardNB(double angle)
{
  argDouble = angle;
  _motionInProgress++;
  THREAD_CREATE(_thread, motionRollForwardThread, this);
  return 0;
}

void* CMobotGroup::motionRollForwardThread(void* arg)
{
  CMobotGroup *cmg = (CMobotGroup*)arg;
  cmg->move(cmg->argDouble, 0, 0, cmg->argDouble);
  cmg->_motionInProgress--;
  return NULL;
}

int CMobotGroup::motionSkinny(double angle)
{
  argDouble = angle;
  _motionInProgress++;
  motionSkinnyThread(this);
  return 0;
}

int CMobotGroup::motionSkinnyNB(double angle)
{
  argDouble = angle;
  _motionInProgress++;
  THREAD_CREATE(_thread, motionSkinnyThread, this);
  return 0;
}

void* CMobotGroup::motionSkinnyThread(void* arg)
{
  CMobotGroup *cmg = (CMobotGroup*)arg;
  cmg->moveJointToNB(ROBOT_JOINT2, cmg->argDouble);
  cmg->moveJointToNB(ROBOT_JOINT3, cmg->argDouble);
  cmg->_motionInProgress--;
  return NULL;
}

int CMobotGroup::motionStand()
{
  _motionInProgress++;
  motionStandThread(this);
  return 0;
}

int CMobotGroup::motionStandNB()
{
  _motionInProgress++;
  THREAD_CREATE(_thread, motionStandThread, NULL);
  return 0;
}

void* CMobotGroup::motionStandThread(void* arg)
{
  CMobotGroup* cmg = (CMobotGroup*)arg;
  cmg->moveToZero();
  cmg->moveJointTo(ROBOT_JOINT2, -85);
  cmg->moveJointTo(ROBOT_JOINT3, 70);
  cmg->moveWait();
  cmg->moveJointTo(ROBOT_JOINT1, 45);
  cmg->moveJointTo(ROBOT_JOINT2, 20);
  cmg->_motionInProgress--;
  return 0;
}

int CMobotGroup::motionTurnLeft(double angle)
{
  argDouble = angle;
  _motionInProgress++;
  motionTurnLeftThread(this);
  return 0;
}

int CMobotGroup::motionTurnLeftNB(double angle)
{
  argDouble = angle;
  _motionInProgress++;
  THREAD_CREATE(_thread, motionTurnLeftThread, this);
  return 0;
}

void* CMobotGroup::motionTurnLeftThread(void* arg)
{
  CMobotGroup* cmg = (CMobotGroup*)arg;
  cmg->move(-cmg->argDouble, 0, 0, cmg->argDouble);
  cmg->_motionInProgress--;
  return NULL;
}

int CMobotGroup::motionTurnRight(double angle)
{
  argDouble = angle;
  _motionInProgress++;
  motionTurnRightThread(this);
  return 0;
}

int CMobotGroup::motionTurnRightNB(double angle)
{
  argDouble = angle;
  _motionInProgress++;
  THREAD_CREATE(_thread, motionTurnRightThread, this);
  return 0;
}

void* CMobotGroup::motionTurnRightThread(void* arg)
{
  CMobotGroup* cmg = (CMobotGroup*)arg;
  cmg->move(cmg->argDouble, 0, 0, -cmg->argDouble);
  cmg->_motionInProgress--;
  return NULL;
}

int CMobotGroup::motionTumbleRight(int num)
{
  argInt = num;
  _motionInProgress++;
  motionTumbleRightThread(this);
  return 0;
}

int CMobotGroup::motionTumbleRightNB(int num)
{
  argInt = num;
  _motionInProgress++;
  THREAD_CREATE(_thread, motionTumbleRightThread, this);
  return 0;
}

void* CMobotGroup::motionTumbleRightThread(void* arg)
{
  int i;
  CMobotGroup* cmg = (CMobotGroup*)arg;
  int num = cmg->argInt;

  cmg->moveToZero();
#ifndef _WIN32
  sleep(1);
#else
  Sleep(1000);
#endif

  for(i = 0; i < num; i++) {
    cmg->moveJointTo(ROBOT_JOINT3, DEG2RAD(85));
    cmg->moveJointTo(ROBOT_JOINT2, DEG2RAD(-80));
    cmg->moveJointTo(ROBOT_JOINT3, DEG2RAD(0));
    cmg->moveJointTo(ROBOT_JOINT2, DEG2RAD(0));
    cmg->moveJointTo(ROBOT_JOINT3, DEG2RAD(-80));
    cmg->moveJointTo(ROBOT_JOINT3, DEG2RAD(-45));
    cmg->moveJointTo(ROBOT_JOINT2, DEG2RAD(85));
    cmg->moveJointTo(ROBOT_JOINT3, DEG2RAD(-80));
    cmg->moveJointTo(ROBOT_JOINT2, DEG2RAD(0));
    cmg->moveJointTo(ROBOT_JOINT3, DEG2RAD(0));
    cmg->moveJointTo(ROBOT_JOINT2, DEG2RAD(-80));
    if(i != (num-1)) {
      cmg->moveJointTo(ROBOT_JOINT2, DEG2RAD(-45));
    }
  }
  cmg->moveJointToNB(ROBOT_JOINT3, 0);
  cmg->moveJointToNB(ROBOT_JOINT2, 0);
  cmg->moveWait();

  cmg->_motionInProgress--;
  return NULL;
}

int CMobotGroup::motionTumbleLeft(int num)
{
  argInt = num;
  _motionInProgress++;
  motionTumbleLeftThread(this);
  return 0;
}

int CMobotGroup::motionTumbleLeftNB(int num)
{
  argInt = num;
  _motionInProgress++;
  THREAD_CREATE(_thread, motionTumbleLeftThread, this);
  return 0;
}

void* CMobotGroup::motionTumbleLeftThread(void* arg)
{
  int i;
  CMobotGroup* cmg = (CMobotGroup*)arg;
  int num = cmg->argInt;

  cmg->moveToZero();
#ifndef _WIN32
  sleep(1);
#else
  Sleep(1000);
#endif

  for(i = 0; i < num; i++) {
    cmg->moveJointTo(ROBOT_JOINT2, DEG2RAD(-85));
    cmg->moveJointTo(ROBOT_JOINT3, DEG2RAD(80));
    cmg->moveJointTo(ROBOT_JOINT2, DEG2RAD(0));
    cmg->moveJointTo(ROBOT_JOINT3, DEG2RAD(0));
    cmg->moveJointTo(ROBOT_JOINT2, DEG2RAD(80));
    cmg->moveJointTo(ROBOT_JOINT2, DEG2RAD(45));
    cmg->moveJointTo(ROBOT_JOINT3, DEG2RAD(-85));
    cmg->moveJointTo(ROBOT_JOINT2, DEG2RAD(80));
    cmg->moveJointTo(ROBOT_JOINT3, DEG2RAD(0));
    cmg->moveJointTo(ROBOT_JOINT2, DEG2RAD(0));
    cmg->moveJointTo(ROBOT_JOINT3, DEG2RAD(80));
    if(i != (num-1)) {
      cmg->moveJointTo(ROBOT_JOINT3, DEG2RAD(45));
    }
  }
  cmg->moveJointToNB(ROBOT_JOINT2, 0);
  cmg->moveJointToNB(ROBOT_JOINT3, 0);
  cmg->moveWait();
  cmg->_motionInProgress--;
  return NULL;
}

int CMobotGroup::motionUnstand()
{
  _motionInProgress++;
  motionUnstandThread(this);
  return 0;
}

int CMobotGroup::motionUnstandNB()
{
  _motionInProgress++;
  THREAD_CREATE(_thread, motionUnstandThread, NULL);
  return 0;
}

void* CMobotGroup::motionUnstandThread(void* arg)
{
  CMobotGroup* cmg = (CMobotGroup*)arg;
  cmg->moveToZero();
  cmg->moveJointTo(ROBOT_JOINT3, 45);
  cmg->moveJointTo(ROBOT_JOINT2, -85);
  cmg->moveWait();
  cmg->moveToZero();
  cmg->moveJointTo(ROBOT_JOINT2, 20);
  cmg->_motionInProgress--;
  return 0;
}

int CMobotGroup::motionWait()
{
  while(_motionInProgress > 0) {
#ifdef _WIN32
    Sleep(200);
#else
    usleep(200000);
#endif
  }
  return 0;
}
#endif

