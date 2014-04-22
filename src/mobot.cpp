/*
   Copyright 2013 Barobo, Inc.

   This file is part of libbarobo.

   BaroboLink is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   BaroboLink is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with BaroboLink.  If not, see <http://www.gnu.org/licenses/>.
*/

//#define COMMSDEBUG

#include "logging.h"
#ifdef _MSC_VER
#define __func__ __FUNCTION__
#endif

#include "dongle.h"
#include "serial_framing_protocol.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>
#ifndef _WIN32
#include <regex.h>
#endif
#include "mobot.h"
#include "mobot_internal.h"
#ifndef _WIN32
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <libgen.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <termios.h>
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

#ifndef _WIN32
#include <netdb.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <arpa/inet.h>
#else
#include <ws2tcpip.h>
#endif

#include "commands.h"
#include <BaroboConfigFile.h>

/* FIXME hlh: hacky, shouldn't be using statically-sized arrays for filenames
 * if we can help it */
#ifndef MAX_PATH
#define MAX_PATH 512
#endif

#define MOBOT_BAUD 230400

#define ABS(x) ((x)<0?-(x):(x))

#define DEPRECATED(from, to) \
  fprintf(stderr, "Warning: The function \"%s()\" is deprecated. Please use \"%s()\"\n" , from, to)

int g_numConnected = 0;
int g_disconnectSignal = 0;

volatile int g_mobotThreadInitializing = 0;

mobot_t* g_dongleMobot = NULL;

bcf_t* g_bcf;

void* eventThread(void* arg);

char* mc_strdup(const char* str)
{
  char* s;
  s = (char*)malloc(sizeof(char)*(strlen(str)+1));
  strcpy(s, str);
  return s;
}

double deg2rad(double deg)
{
  return deg * M_PI / 180.0;
}

double degree2radian(double deg)
{
  return deg * M_PI / 180.0;
}

double rad2deg(double rad)
{
  return rad * 180.0 / M_PI;
}

double radian2degree(double rad)
{
  return rad * 180.0 / M_PI;
}

void* nullThread(void* arg)
{ 
  return NULL;
}

// get sockaddr, IPv4 or IPv6:
void *get_in_addr(struct sockaddr *sa)
{
    if (sa->sa_family == AF_INET) {
        return &(((struct sockaddr_in*)sa)->sin_addr);
    }

    return &(((struct sockaddr_in6*)sa)->sin6_addr);
}

int Mobot_accelTimeNB(mobot_t* comms, double radius, double acceleration, double timeout)
{
  double alpha = acceleration/radius;
  int i;
  int rc;
  for(i = 1; i < 4; i++) {
    rc = Mobot_accelAngularTimeNB(comms, (robotJointId_t)i, alpha, timeout);
    if(rc) return rc;
  }
  return 0;
}

int Mobot_accelToVelocityNB(mobot_t* comms, double radius, double acceleration, double velocity)
{
  double timeout;
  timeout = velocity/acceleration;
  double alpha = acceleration/radius;
  int i;
  int rc = 0;
  for(i = 1; i < 4; i++) {
    rc = Mobot_accelAngularTimeNB(comms, (robotJointId_t)i, alpha, timeout);
    if(rc) return rc;
  }
  return 0;
}

int Mobot_accelToMaxSpeedNB(mobot_t* comms, double radius, double acceleration)
{
  double alpha = acceleration/radius;
  int i;
  int rc;
  for(i = 1; i < 4; i++) {
    rc = Mobot_accelAngularTimeNB(comms, (robotJointId_t)i, alpha, 0);
    if(rc) return rc;
  }
  return 0;
}

int Mobot_accelAngularTimeNB(mobot_t* comms, robotJointId_t id, double acceleration, double time)
{
  uint8_t buf[32];
  uint32_t millis;
  float a;
  int status;
  buf[0] = (uint8_t)id-1;
  a = acceleration;
  memcpy(&buf[1], &a, 4);
  millis = time*1000;
  millis = htonl(millis);
  memcpy(&buf[5], &millis, 4);
  status = MobotMsgTransaction(comms, BTCMD(CMD_SET_ACCEL), buf, 13);
  if(status < 0) return status;
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_accelAngularToVelocityNB(mobot_t* comms, robotJointId_t id, double acceleration, double speed)
{
  double timeout = speed/acceleration;
  return Mobot_accelAngularTimeNB(comms, id, acceleration, timeout);
}

int Mobot_accelAngularAngleNB(mobot_t* comms, robotJointId_t id, double acceleration, double angle)
{
  double timeout = sqrt(2*angle/acceleration);
  return Mobot_accelAngularTimeNB(comms, id, acceleration, timeout);
}

int Mobot_smoothMoveToNB(
    mobot_t* comms, 
    robotJointId_t id, 
    double accel0, 
    double accelf, 
    double vmax, 
    double angle)
{
  uint8_t buf[32];
  float f[4];
  int status;
  f[0] = accel0;
  f[1] = accelf;
  f[2] = vmax;
  f[3] = angle;
  buf[0] = (uint8_t)id-1;
  memcpy(&buf[1], &f, 16);
  status = MobotMsgTransaction(comms, BTCMD(CMD_SMOOTHMOVE), buf, 17);
  if(status < 0) return status;
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
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
  /* Try a TCP connection first */
  if(Mobot_connectWithTCP(comms) == 0) {
    return 0;
  }
  FILE *fp;
  int i;
  const char* path = comms->configFilePath;
  char buf[512];
  const char *str;
  if(g_bcf == NULL) {
    g_bcf = BCF_New();
    if(BCF_Read(g_bcf, path)) {
      fprintf(stderr, 
          "ERROR: Your Barobo configuration file does not exist.\n"
          "Please create one by opening the MoBot remote control, clicking on\n"
          "the 'Robot' menu entry, and selecting 'Configure Robot Bluetooth'.\n");
      BCF_Destroy(g_bcf);
      g_bcf = NULL;
      return -1;
    }
  }

  /* Read the correct line */
  str = BCF_GetIndex(g_bcf, g_numConnected);
  if(str) {
   strcpy(buf, str);
  } 
  /* Get rid of trailing newline and/or carriage return */
  while(
      (buf[strlen(buf)-1] == '\r' ) ||
      (buf[strlen(buf)-1] == '\n' ) 
      )
  {
    buf[strlen(buf)-1] = '\0';
  }
#ifndef __MACH__ /* If Unix or Windows */
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
  return 0;
}

#define PORT "5768"
#define MAXDATASIZE 128
int Mobot_connectWithTCP(mobot_t* comms)
{
  return Mobot_connectWithIPAddress(comms, "localhost", PORT);
}

int Mobot_connectWithIPAddress(mobot_t* comms, const char address[], const char port[])
{
  int sockfd, numbytes;  
  char buf[MAXDATASIZE];
  struct addrinfo hints, *servinfo, *p;
  int rv;
  int rc;
  char s[INET6_ADDRSTRLEN];
  memset(&hints, 0, sizeof hints);
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  if ((rv = getaddrinfo(address, port, &hints, &servinfo)) != 0) {
    //fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
    return -1;
  }

  // loop through all the results and connect to the first we can
  for(p = servinfo; p != NULL; p = p->ai_next) {
    if ((sockfd = socket(p->ai_family, p->ai_socktype,
            p->ai_protocol)) == -1) {
      //perror("client: socket");
      continue;
    }

    if (connect(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
#ifndef _WIN32
      close(sockfd);
#else
      closesocket(sockfd);
#endif
      //perror("client: connect");
      continue;
    }

    break;
  }
  if (p == NULL) {
    //fprintf(stderr, "client: failed to connect\n");
    return -1;
  }

  /*
  inet_ntop(p->ai_family, get_in_addr((struct sockaddr *)p->ai_addr),
      s, sizeof s);
   */
  //printf("client: connecting to %s\n", s);

  freeaddrinfo(servinfo); // all done with this structure
  comms->socket = sockfd;
  comms->connected = 1;
  comms->connectionMode = MOBOTCONNECT_TCP;
  rc = finishConnect(comms);
  if(rc) return rc;
  return 0;
}

void Mobot_initDongle()
{
  int rc;
  char buf[32];
  int i;
  if(g_dongleMobot == NULL) {
    g_dongleMobot = (mobot_t*)malloc(sizeof(mobot_t));
    Mobot_init(g_dongleMobot);
  }
  if( -1 == Mobot_dongleGetTTY(buf, sizeof(buf)) ) {
    fprintf(stderr, "(barobo) ERROR: in %s, no dongle found.\n", __func__);
    free(g_dongleMobot);
    g_dongleMobot = NULL;
    return;
  }
  Mobot_connectWithTTY(g_dongleMobot, buf);
}

int Mobot_connectWithAddress(mobot_t* comms, const char* address, int channel)
{
  /* Depending on the format of the address, we should either try bluetooth
   * connection or zigbee connection */
  int rc;
#ifndef _WIN32
  regex_t regex;
  /* Try to match a Bluetooth mac address similar to 00:06:66:45:AE:3F */
  rc = regcomp(&regex, "\\<\\([0-9a-fA-F]\\{2\\}:\\)\\{5\\}[0-9a-fA-F]\\{2\\}\\>", REG_ICASE);
  if(rc) {
    fprintf(stderr, "FATAL: Error compiling regex %s:%d\n", __FILE__, __LINE__);
    exit(0);
  }
  rc = regexec(&regex, address, 0, NULL, 0);
  regfree(&regex);
#else
  if(strlen(address) == 17) {
    rc = 0;
  } else {
    rc = -1;
  }
#endif
  if(rc == 0) {
#ifdef ENABLE_BLUETOOTH
    return Mobot_connectWithBluetoothAddress(comms, address, channel);
#else
    assert(0);  // FIXME :(
#endif
  } else {
    rc = Mobot_connectChildID(g_dongleMobot, comms, address);
    return rc;
  }
  return 0;
}

int Mobot_connectWithSerialID(mobot_t* comms, const char address[])
{
  return Mobot_connectChildID(g_dongleMobot, comms, address);
}

int Mobot_connectWithZigbeeAddress(mobot_t* comms, uint16_t addr)
{
  int rc;
  int form;
  if(g_dongleMobot == NULL) {
    Mobot_initDongle();
  }
  if(
      (g_dongleMobot == NULL) || 
      (g_dongleMobot->connected == 0)
    )
  {
    return -1;
  }
  comms->connected = 1;
  comms->connectionMode = MOBOTCONNECT_ZIGBEE;
  comms->parent = g_dongleMobot;
  comms->zigbeeAddr = addr;

  /* Need to add this mobot to the parent's list of children */
  mobotInfo_t* iter;
  int idFound = 0;
  for(iter = g_dongleMobot->children; iter!=NULL; iter = iter->next) {
    if(iter->zigbeeAddr == comms->zigbeeAddr) {
      idFound = 1;
      iter->mobot = comms;
      break;
    }
  }
  if(!idFound) {
    MUTEX_LOCK(g_dongleMobot->mobotTree_lock);
    iter = (mobotInfo_t*)malloc(sizeof(mobotInfo_t));
    iter->zigbeeAddr = comms->zigbeeAddr;
    iter->parent = g_dongleMobot;
    iter->mobot = comms;
    iter->next = g_dongleMobot->children;
    g_dongleMobot->children = iter;
    rc = Mobot_getID(comms);
    strcpy(iter->serialID, comms->serialID);
    if(Mobot_pair(comms)) {
      /* The child is already paired. Disconnect and return error... */
      comms->connected = 0;
      comms->connectionMode = MOBOTCONNECT_NONE;
      comms->parent = NULL;
      //iter->mobot = NULL;
      MUTEX_UNLOCK(g_dongleMobot->mobotTree_lock);
      return -1;
    }
    MUTEX_UNLOCK(g_dongleMobot->mobotTree_lock);
  }

  rc = getFormFactor(comms, &form);
  if(rc < 0) {
    comms->connected = 0;
    return rc;
  }
  comms->formFactor = (mobotFormFactor_t)form;

  if(rc){
    return rc;
  }
  return 0;
}

#ifdef ENABLE_BLUETOOTH
#if _WIN32
typedef struct bdaddr_s {
  UINT8 b[6];
} bdaddr_t;

static void baswap(bdaddr_t *dst, const bdaddr_t *src)
{
	register unsigned char *d = (unsigned char *) dst;
	register const unsigned char *s = (const unsigned char *) src;
	register int i;

	for (i = 0; i < 6; i++)
		d[i] = s[5-i];
}

static int str2ba(const char *str, bdaddr_t *ba)
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

int Mobot_connectWithBluetoothAddress(mobot_t* comms, const char* address, int channel)
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
  comms->connectionMode = MOBOTCONNECT_BLUETOOTH;
  comms->connected = 1;
  status = finishConnect(comms);
  if(status) {
    comms->connectionMode = MOBOTCONNECT_NONE;
    comms->connected = 0;
  } 
  return status;
#else
  err = Mobot_connectWithAddressTTY(comms, address);
  if(err) return err;
  comms->connectionMode = MOBOTCONNECT_BLUETOOTH;
  return 0;
#endif
}
#endif // ENABLE_BLUETOOTH

int Mobot_connectWithAddressTTY(mobot_t* comms, const char* address)
{
  char buf[80];
  char chunk1[3];
  char chunk2[3];
  sscanf(address, "%*c%*c:%*c%*c:%*c%*c:%*c%*c:%c%c:%c%c", 
      &chunk1[0], &chunk1[1],
      &chunk2[0], &chunk2[1]);
  chunk1[2] = '\0';
  chunk2[2] = '\0';
  sprintf(buf, "/dev/tty.MOBOT-%s%s-SPP", chunk1, chunk2);
  return Mobot_connectWithTTY(comms, buf);
}

int Mobot_connectWithTTY(mobot_t *comms, const char *ttyfilename) {
  return Mobot_connectWithTTYBaud(comms, ttyfilename, MOBOT_BAUD);
}

int Mobot_connectWithTTYBaud(mobot_t* comms, const char* ttyfilename, unsigned long baud)
{
  int status;
#ifndef _WIN32
  FILE *lockfile;
  char *filename = strdup(ttyfilename);
  char lockfileName[MAX_PATH];
  int pid;
  bInfo(stderr, "(barobo) INFO: Connecting to %s\n", ttyfilename);
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
  comms->lockfileName = strdup(lockfileName);
  if(lockfile != NULL) {
    fclose(lockfile);
  }
#endif

  comms->dongle = (MOBOTdongle*)malloc(sizeof(MOBOTdongle));
  assert(comms->dongle);

  bInfo(stderr, "(barobo) INFO: calling dongleOpen()\n");
  int err = dongleOpen(comms->dongle, ttyfilename, baud);
  if (err) {
    bInfo(stderr, "(barobo) INFO: dongleOpen() failed\n");
    return err;
  }
  bInfo(stderr, "(barobo) INFO: dongleOpen() succeeded!\n");

  comms->connected = 1;
  comms->connectionMode = MOBOTCONNECT_TTY;

  status = finishConnect(comms);
  if(status) {
    fprintf(stderr, "(barobo) ERROR: finishConnect() returned something not good.\n");
    Mobot_disconnect(comms);
    return status;
  }

#ifndef _WIN32
  /* Finished connecting. Create the lockfile. */
  lockfile = fopen(lockfileName, "w");
  if(lockfile == NULL) {
    fprintf(stderr, "Fatal error. %s:%d\n", __FILE__, __LINE__);
    return -1;
  }
  fprintf(lockfile, "%d", getpid());
  fclose(lockfile);
#endif

  return 0;
}

int getFormFactor(mobot_t* comms, int* form)
{
  uint8_t buf[32];
  int status;
  status = MobotMsgTransaction(comms, BTCMD(CMD_GETFORMFACTOR), buf, 0);
  if(status < 0) return status;
  /* Make sure the data size is correct */
  if(buf[1] != 4) {
    return -1;
  }
  /* Copy the data */
  *form = buf[2];
  return 0;
}

int Mobot_connectChild(mobot_t* parent, mobot_t* child)
{
  /* First check to see if the requested child is already in the list of knows
   * Serial ID's */  
  int idFound = 0;
  mobotInfo_t* iter;
  int i;
  int rc;
  Mobot_initDongle();
  for(i = 0; i < 2; i++) {
    MUTEX_LOCK(parent->mobotTree_lock);
    if(parent->children != NULL) {
      child->connected = 1;
      child->connectionMode = MOBOTCONNECT_ZIGBEE;
      strcpy(child->serialID, parent->children->serialID);
      child->zigbeeAddr = parent->children->zigbeeAddr;

      rc = finishConnect(child);
      if(rc) {
        child->connected = 0;
        child->connectionMode = MOBOTCONNECT_NONE;
      } else {
        parent->children->mobot = child;
      }
      MUTEX_UNLOCK(parent->mobotTree_lock);
      return rc;
    }
    MUTEX_UNLOCK(parent->mobotTree_lock);
    Mobot_queryAddresses(parent);
#ifdef _WIN32
    Sleep(5000);
#else
    sleep(5);
#endif
  }
  /* Did not find the child */
  MUTEX_UNLOCK(parent->mobotTree_lock);
  return -1;
}

int Mobot_findMobot(mobot_t* parent, const char* childSerialID)
{
  uint8_t buf[64];
  int rc;
  /* If a parent was specified, use it as a dongle */
  if(parent == NULL) {
    Mobot_initDongle();
    parent = g_dongleMobot;
  }
  memcpy(buf, childSerialID, 4);
  rc = MobotMsgTransaction(parent, BTCMD(CMD_FINDMOBOT), buf, 4);
  if(rc) {return rc;}
  if(buf[0] == 0xff) { return -1; } 
  return 0;
}

int Mobot_connectChildID(mobot_t* parent, mobot_t* child, const char* childSerialID)
{ 
  int form; int rc;
  int i;
  char* _childSerialID;
  _childSerialID = strdup(childSerialID);
  for(i = 0; i < strlen(_childSerialID); i++) {
    _childSerialID[i] = toupper(_childSerialID[i]);
  }
  /* If a parent was specified, use it as a dongle */
  if(parent == NULL) {
    Mobot_initDongle();
    parent = g_dongleMobot;
  } else {
  }
  /* If parent is still NULL, return error */
  if(parent == NULL) {
    return -2;
  }
  /* First, check to see if it is our ID */
  if(!strcmp(parent->serialID, _childSerialID)) {
    /* FIXME dammit, this and the code in the if(idFound) block below needs
     * to be factored out into a finishConnectZigbee() function. */
    free(_childSerialID);

    strcpy(child->serialID, parent->serialID);
    child->zigbeeAddr = parent->zigbeeAddr;
    child->connected = 1;
    child->connectionMode = MOBOTCONNECT_ZIGBEE;
    child->parent = parent;
    parent->child = child;

    rc = Mobot_pair(child);
    if (rc) {
        return rc;
    }

    return finishConnectWithoutCommsThread(child);
  }
  /* Now check to see if the requested child is already in the list of known
   * Serial ID's */  
  int idFound = 0;
  mobotInfo_t* iter;
  for(i = 0; i < 2; i++) {
    MUTEX_LOCK(parent->mobotTree_lock);
    for(iter = parent->children; iter != NULL; iter = iter->next)
    {
      if(!strncmp(iter->serialID, _childSerialID, 4)) {
        idFound = 1;
        break;
      }
    }
    if(idFound) {
      free(_childSerialID);

      strcpy(child->serialID, iter->serialID);
      child->zigbeeAddr = iter->zigbeeAddr;
      child->connected = 1;
      child->connectionMode = MOBOTCONNECT_ZIGBEE;
      child->parent = iter->parent;
      iter->mobot = child;

      /* This block used to be protected by an if (form == Mobot-L | Mobot-I)
       * condition. Removed it to match the code above that executes when the
       * child is the parent. */
      {
        /* Tell the Mobot it is now paired */
        if(Mobot_pair(child)) {
          /* The child is already paired. Disconnect and return error... */
          child->connected = 0;
          child->connectionMode = MOBOTCONNECT_NONE;
          child->parent = NULL;
          //iter->mobot = NULL;
          MUTEX_UNLOCK(parent->mobotTree_lock);
          return -1;
        }
      }
      MUTEX_UNLOCK(parent->mobotTree_lock);
      return finishConnectWithoutCommsThread(child);
    } else if (i == 0){
      MUTEX_UNLOCK(parent->mobotTree_lock);
      //Mobot_queryAddresses(parent);
      Mobot_findMobot(parent, _childSerialID);
      Mobot_waitForReportedSerialID(parent, _childSerialID);
    }
  }
  /* Did not find the child */
  MUTEX_UNLOCK(parent->mobotTree_lock);
  free(_childSerialID);
  return -1;
}

int finishConnectWithoutCommsThread (mobot_t* comms);

int finishConnect (mobot_t* comms) {
  /* Start the comms engine */
  g_mobotThreadInitializing = 1;
  THREAD_CREATE(comms->commsThread, commsEngine, comms);
  while(g_mobotThreadInitializing);

#if 0
  if(comms->connectionMode == MOBOTCONNECT_TTY) {
  /* deprecated by libsfp */

    g_mobotThreadInitializing = 1;

    THREAD_CREATE(comms->commsOutThread, commsOutEngine, comms);
    while(g_mobotThreadInitializing);

    /* hlh: I wish this didn't have to go here, but we need the commsEngine
     * thread to be running before sfpConnect() can really do anything. */
    sfpConnect(comms->sfp_ctx);
    /* FIXME abort after a timeout */
    while (!sfpIsConnected(comms->sfp_ctx));
  }
#endif

  return finishConnectWithoutCommsThread(comms);
}

/* finishConnectWithoutCommsThread():
 * Perform final connecting tasks common to all connection methods */
int finishConnectWithoutCommsThread(mobot_t* comms)
{
  int i = 0, rc;
  int numJoints = 0;
  mobotFormFactor_t form;
  uint8_t buf[256];

  /* Make sure we are connected to a Mobot */
  if(Mobot_getStatus(comms)) {
    fprintf(stderr, "(barobo) ERROR: Mobot_getStatus() returned something not good.\n");
    Mobot_disconnect(comms);
    return -1;
  }

  /* Get form factor */
  rc = getFormFactor(comms, (int*)&form);
  if(rc == -1) {
    form = MOBOTFORM_ORIGINAL;
  } else if (rc == -2) {
    fprintf(stderr, "(barobo) ERROR: getFormFactor() returned something not good.\n");
    Mobot_disconnect(comms);
    return rc;
  }
  comms->formFactor = form;
  switch(form) {
    case MOBOTFORM_ORIGINAL:
      numJoints = 4;
      break;
    case MOBOTFORM_I:
    case MOBOTFORM_L:
    case MOBOTFORM_T:
      numJoints = 3;
      break;
    default:
      numJoints = 0;
  }
  /* Get the protocol version; make sure it matches ours */
  int version;
  version = Mobot_getVersion(comms); 
  if(version < 0) {
    fprintf(stderr, "(barobo) ERROR: Mobot_getVersion() returned something not good.\n");
    Mobot_disconnect(comms);
    return version;
  }
  if(version < CMD_NUMCOMMANDS) {
    fprintf(stderr, "Warning. Communications protocol version mismatch.\n");
    fprintf(stderr, "Robot Firmware Protocol Version: %d\n", version);
    fprintf(stderr, "CMobot Library Protocol Version: %d\n", CMD_NUMCOMMANDS);
  }
  /* Get the joint max speeds */
  /* DEBUG */
  /*
  for(i = numJoints; i >= 1; i--) {
    if(Mobot_getJointMaxSpeed(comms, (robotJointId_t)i, &(comms->maxSpeed[i-1])) < 0) {
      i++;
    }
  }
  */
  /* FIXME double check, should this be a 4 or a 3? Or should it be numJoints? */
  for(i = 0; i < 4; i++) {
    if(comms->formFactor == MOBOTFORM_ORIGINAL) {
      /* FIXME should this be MOBOT_MAX_SPEED? */
      comms->maxSpeed[i] = DEG2RAD(120);
    } else {
      comms->maxSpeed[i] = LINKBOT_MAX_SPEED;
    }
  }
  Mobot_setJointSpeeds( comms, 
      DEG2RAD(45), 
      DEG2RAD(45), 
      DEG2RAD(45), 
      DEG2RAD(45) );

  if(
      (comms->formFactor == MOBOTFORM_I) ||
      (comms->formFactor == MOBOTFORM_L) 
    )
  {
    comms->zigbeeAddr = Mobot_getAddress(comms);
    /* See if we can get the serial id */
    rc = Mobot_getID(comms);
    if (-1 == rc) {
      fprintf(stderr, "(barobo) WARNING: Unable to get robot serial ID.\n");
    }
    else {
      bInfo(stderr, "(barobo) INFO: %s finished connecting\n", comms->serialID);
    }
  }

  /* Start the eventqueue thread */
  THREAD_CREATE(comms->eventthread, eventThread, comms);

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
  status = MobotMsgTransaction(comms, BTCMD(CMD_BLINKLED), buf, 5);
  if(status < 0) return status;
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

mobotMelodyNote_t* Mobot_createMelody(int tempo)
{
  mobotMelodyNote_t* tmp;
  tmp = (mobotMelodyNote_t*)malloc(sizeof(mobotMelodyNote_t));
  tmp->tempo = tempo;
  tmp->next = NULL;
  return tmp;
}

int Mobot_melodyAddNote(mobotMelodyNote_t* melody, const char* note, int divider)
{
  int i;
  int index;
  uint8_t byte;
  uint8_t octave;
  char mynote;
  mobotMelodyNote_t* iter;
  iter = melody;
  for(i = 0; iter->next != NULL; iter = iter->next, i++);
  if(i > 255) {
    return -1;
  }
  /* First, lowercase the first character */
  mynote = tolower(note[0]);
  if(mynote < 'a' || mynote > 'g') {
    return -1;
  }
  /* Allocate new note */ 
  iter->next = (mobotMelodyNote_t*)malloc(sizeof(mobotMelodyNote_t));
  iter->next->next = NULL;
  /* Parse the note */
  switch(mynote) {
    case 'c':
      index = 0;
      break;
    case 'd':
      index = 2;
      break;
    case 'e':
      index = 4;
      break;
    case 'f':
      index = 5;
      break;
    case 'g':
      index = 7;
      break;
    case 'a':
      index = 9;
      break;
    case 'b':
      index = 11;
      break;
    default:
      return -1;
  }
  i = 1;
  if(note[i] == '#') {
    index++;
    i++;
  } else if (note[i] == 'b') {
    index--;
    i++;
  }
  index = (index + 12)%12;

  if(note[i] > '0' && note[i] < '9') {
    octave = note[i] - '0';
  } else {
    octave = 4;
  }

  iter->next->notedata[0] = (uint8_t)divider;
  byte = octave;
  byte = byte << 4;
  iter->next->notedata[1] = byte;
  byte = index;
  iter->next->notedata[1] |= byte;
  return 0;
}

int Mobot_loadMelody(mobot_t* comms, int id, mobotMelodyNote_t* melody)
{
  uint8_t data[256];
  uint8_t length;
  int status = 1;
  int retries;
  int recvBuf[16];
  mobotMelodyNote_t* iter;
  iter = melody->next;
  for(length = 0; iter != NULL; length++, iter = iter->next);
  data[0] = (uint8_t)id;
  data[1] = melody->tempo;
  data[2] = length;
  iter = melody->next;
  for(length = 1; iter != NULL; length++, iter = iter->next) {
    memcpy(&data[length*2+1], &iter->notedata[0], 2);
  }
  for(retries = 0; retries <= MAX_RETRIES && status != 0; retries++) {
    SendToIMobot(comms, BTCMD(CMD_LOADMELODY), data, length*2+1);
#ifndef _WIN32
    usleep(500000);
#else
    Sleep(500);
#endif
    status = RecvFromIMobot(comms, (uint8_t*)recvBuf, sizeof(recvBuf));
  }
  /* Make sure the data size is correct */
  if(recvBuf[1] != 3) {
    return -1;
  }
  return status;
}

int Mobot_playMelody(mobot_t* comms, int id)
{
  int status;
  uint8_t buf[8];
  buf[0] = (uint8_t)id;
  status = MobotMsgTransaction(comms, BTCMD(CMD_PLAYMELODY), buf, 1);
  if(status < 0) return status;
  /* Make sure the buf size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_getAddress(mobot_t* comms)
{
  int status;
  uint8_t buf[8];
  int addr;
  status = MobotMsgTransaction(comms, BTCMD(CMD_GETADDRESS), buf, 0);
  if(status < 0) return status;
  /* Make sure the buf size is correct */
  if(buf[1] != 5) {
    return -1;
  }
  addr = (buf[2]<<8) | buf[3];
  return addr;
}

int Mobot_queryAddresses(mobot_t* comms)
{
  int status;
  uint8_t buf[8];
  status = MobotMsgTransaction(comms, BTCMD(CMD_QUERYADDRESSES), buf, 0);
  if(status < 0) return status;
  /* Make sure the buf size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_registerScanCallback(mobot_t* comms, void (*cb) (const char* id))
{
  MUTEX_LOCK(comms->scan_callback_lock);
  comms->scan_callback = cb;
  MUTEX_UNLOCK(comms->scan_callback_lock);
  return 0;
}

int Mobot_clearQueriedAddresses(mobot_t* comms)
{
  int status;
  uint8_t buf[8];
  status = MobotMsgTransaction(comms, BTCMD(CMD_CLEARQUERIEDADDRESSES), buf, 0);
  if(status < 0) return status;
  /* Make sure the buf size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

mobot_t* Mobot_getDongle()
{
  return g_dongleMobot;
}

int Mobot_setDongleMobot(mobot_t* comms)
{
  g_dongleMobot = comms;
  return 0;
}

int Mobot_setRFChannel(mobot_t* comms, uint8_t channel)
{
  int status;
  uint8_t buf[8];
  int addr;
  buf[0] = channel;
  status = MobotMsgTransaction(comms, BTCMD(CMD_SETRFCHANNEL), buf, 1);
  if(status < 0) return status;
  /* Make sure the buf size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_setID(mobot_t* comms, const char* id)
{
  int status;
  uint8_t buf[8];
  int addr;
  int i = 0;
  while(id[i] != '\0') {
    buf[i] = id[i];
    i++;
  }
  status = MobotMsgTransaction(comms, BTCMD(CMD_SETSERIALID), buf, i);
  if(status < 0) return status;
  /* Make sure the buf size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_reboot(mobot_t* comms)
{
  uint16_t addr;
  int i;
  uint8_t buf[8];
  //status = MobotMsgTransaction(comms, BTCMD(CMD_REBOOT), buf, 3);
  SendToIMobot(comms, BTCMD(CMD_REBOOT), NULL, 0);
  MUTEX_UNLOCK(comms->commsLock);
#if 0
  /* deprecated by libsfp */

  /* Wait until message is sent before returning */
  MUTEX_LOCK(comms->sendBuf_lock);
  while(comms->sendBuf_N > 0) {
    COND_WAIT(comms->sendBuf_cond, comms->sendBuf_lock);
  }
  MUTEX_UNLOCK(comms->sendBuf_lock);
#endif
  return 0;
}

int Mobot_disconnect(mobot_t* comms)
{
  int rc = 0;
  /* Only globally allow one mobot to disconnect at a time */
  static int initialized = 0;
  static MUTEX_T lock;
  if(!initialized) {
    MUTEX_INIT(&lock);
  }
  MUTEX_LOCK(&lock);
  mobotInfo_t* iter;
  if(comms->connected == 0) {
    MUTEX_UNLOCK(&lock);
    return 0;
  }
  bInfo(stderr, "(barobo) INFO: disconnecting %s\n", comms->serialID);
#ifndef _WIN32
  switch(comms->connectionMode) {
    case MOBOTCONNECT_BLUETOOTH:
    case MOBOTCONNECT_TCP:
      comms->connected = 0;
      shutdown(comms->socket, SHUT_RDWR);
      if(close(comms->socket)) {
        /* Error closing file descriptor */
        rc = -1;
      } 
      break;
    case MOBOTCONNECT_TTY:
      comms->connected = 0;
      /* Unpair all children */
      for(iter = comms->children; iter != NULL; iter = iter->next) {
        if(iter->mobot) {
          bInfo(stderr, "(barobo) INFO: Disconnecting child %s.\n", iter->mobot->serialID);
          Mobot_disconnect(iter->mobot);
        }
      }
      if(comms->lockfileName != NULL) {
        unlink(comms->lockfileName);
        free(comms->lockfileName);
        comms->lockfileName = NULL;
      }
      g_disconnectSignal = 1;
      pthread_kill(*comms->commsThread, SIGINT);
      //while(g_disconnectSignal);
      dongleClose(comms->dongle);
      free(comms->dongle);
      break;
    case MOBOTCONNECT_ZIGBEE:
      Mobot_unpair(comms);
      /* If we are the ghost-child of a TTY connected robot, we need to set
       * child back to NULL */
      if(comms == comms->parent->child) {
        comms->parent->child = NULL;
      }
      break;
    default:
      rc = 0;
  }
#else
#if 0
  closesocket(comms->socket);
  if(comms->connectionMode == MOBOTCONNECT_TTY) {
    CancelIo(comms->commHandle);
  }
  sendBufAppend(comms, (uint8_t*)&rc, 1);
  //THREAD_JOIN(*comms->commsThread);
  //CloseHandle(*comms->commsThread);
  //CloseHandle((LPVOID)comms->socket);
#endif
  switch(comms->connectionMode) {
    case MOBOTCONNECT_BLUETOOTH:
    case MOBOTCONNECT_TCP:
      comms->connected = 0;
      if(closesocket(comms->socket)) {
        /* Error closing file descriptor */
        rc = -1;
      } 
      THREAD_JOIN(*comms->commsThread);
      Sleep(200);
      break;
    case MOBOTCONNECT_TTY:
      /* Cancel IO, stop threads */
      comms->connected = 0;
      dongleWrite(comms->dongle, (uint8_t *)&rc, 1);

      /* Unpair all children */
      for(iter = comms->children; iter != NULL; iter = iter->next) {
        if(iter->mobot) {
          bInfo(stderr, "(barobo) INFO: Disconnecting child %s.\n", iter->mobot->serialID);
          Mobot_disconnect(iter->mobot);
        }
      }
      dongleClose(comms->dongle);
      // hlh: not used? Gonna have to figure out a better way of cancelling
      // threads, anyway.
      //SetEvent(comms->cancelEvent);
      /* FIXME Once full link abstraction is accomplished, we can just do a
       * linkTimedRead() or something like that in the commsEngine thread,
       * which will enable us to periodically check a kill semaphore. Kind of
       * a clunky way to do it, but nicely portable. */
      THREAD_JOIN(*comms->commsThread);
      Sleep(200);

      break;
    case MOBOTCONNECT_ZIGBEE:
      /* If we are the ghost-child of a TTY connected robot, we need to set
       * child back to NULL */
      if(comms == comms->parent->child) {
        comms->parent->child = NULL;
      }
      Mobot_unpair(comms);
      break;
    default:
      rc = 0;
  }
#endif
  if(g_numConnected > 0) {
    g_numConnected--;
  }
  MUTEX_UNLOCK(&lock);
  return rc;
}

int Mobot_enableButtonCallback(mobot_t* comms, void* data, 
    void (*buttonCallback)(void* data, int button, int buttonDown))
{
  uint8_t buf[16];
  int status;
  MUTEX_LOCK(comms->callback_lock);
  /* Send a message to the Mobot */
  buf[0] = 1;
  status = MobotMsgTransaction(comms, BTCMD(CMD_ENABLEBUTTONHANDLER), buf, 1);
  if(status < 0) {
    MUTEX_UNLOCK(comms->callback_lock);
    return status;
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

int Mobot_enableJointEventCallback(mobot_t* comms, void* userdata,
    void (*jointCallback)(int millis, double j1, double j2, double j3, double j4, int mask, void* data)
    )
{
  int status;
  uint8_t buf[24];
  buf[0] = 7;
  MUTEX_LOCK(comms->callback_lock);
  status = MobotMsgTransaction(comms, BTCMD(CMD_SET_ENABLE_JOINT_EVENT), buf, 1);
  if(status < 0) {
    MUTEX_UNLOCK(comms->callback_lock);
    return status;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 0x03) {
    MUTEX_UNLOCK(comms->callback_lock);
    return -1;
  }
  comms->jointCallback = jointCallback;
  comms->jointCallbackData = userdata;
  MUTEX_UNLOCK(comms->callback_lock);

  return 0;
}

int Mobot_disableJointEventCallback(mobot_t* comms)
{
  int status;
  uint8_t buf[24];
  buf[0] = 0;
  MUTEX_LOCK(comms->callback_lock);
  status = MobotMsgTransaction(comms, BTCMD(CMD_SET_ENABLE_JOINT_EVENT), buf, 1);
  if(status < 0) {
    MUTEX_UNLOCK(comms->callback_lock);
    return status;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 0x03) {
    MUTEX_UNLOCK(comms->callback_lock);
    return -1;
  }
  comms->jointCallback = NULL;
  comms->jointCallbackData = NULL;
  MUTEX_UNLOCK(comms->callback_lock);

  return 0;
}

int Mobot_enableAccelEventCallback(mobot_t* comms, void* data,
    void (*accelCallback)(int millis, double x, double y, double z, void* data))
{
  int status;
  uint8_t buf[24];
  buf[0] = 7;
  MUTEX_LOCK(comms->callback_lock);
  status = MobotMsgTransaction(comms, BTCMD(CMD_SET_ENABLE_ACCEL_EVENT), buf, 1);
  if(status < 0) {
    MUTEX_UNLOCK(comms->callback_lock);
    return status;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 0x03) {
    MUTEX_UNLOCK(comms->callback_lock);
    return -1;
  }
  comms->accelCallback = accelCallback;
  comms->accelCallbackData = data;
  MUTEX_UNLOCK(comms->callback_lock);

  return 0;
}

int Mobot_disableAccelEventCallback(mobot_t* comms)
{ 
  int status;
  uint8_t buf[24];
  buf[0] = 7;
  MUTEX_LOCK(comms->callback_lock);
  status = MobotMsgTransaction(comms, BTCMD(CMD_SET_ENABLE_ACCEL_EVENT), buf, 1);
  if(status < 0) {
    MUTEX_UNLOCK(comms->callback_lock);
    return status;
  }
  /* Make sure the data size is correct */
  if(buf[1] != 0x03) {
    MUTEX_UNLOCK(comms->callback_lock);
    return -1;
  }
  comms->accelCallback = NULL;
  comms->accelCallbackData = NULL;
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
  status = MobotMsgTransaction(comms, BTCMD(CMD_ENABLEBUTTONHANDLER), buf, 1);
  if(status < 0) {
    MUTEX_UNLOCK(comms->callback_lock);
    return status;
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

int Mobot_enableEventCallback(mobot_t* comms, 
    void (*eventCallback)(const uint8_t* buf, int size, void* userdata), void* data)
{
  comms->eventCallback = eventCallback;
  comms->eventCallbackData = data;
  return 0;
}

int Mobot_disableEventCallback(mobot_t* comms)
{
  comms->eventCallback = NULL;
  comms->eventCallbackData = NULL;
  return 0;
}

int Mobot_init(mobot_t* comms)
{
  int i;
  memset(comms, 0, sizeof(mobot_t));
#ifndef __MACH__
  comms->addr = (sockaddr_t*)malloc(sizeof(sockaddr_t));
  memset(comms->addr, 0, sizeof(sockaddr_t));
#endif
  comms->connected = 0;
  comms->connectionMode = MOBOTCONNECT_NONE;
#ifdef _WIN32
  WSADATA wsd;
  if(WSAStartup (MAKEWORD(2,2), &wsd) != 0) {
    printf("WSAStartup failed with error %d\n", WSAGetLastError());
  }
#endif
  for(i = 0; i < 4; i++) {
    comms->jointSpeeds[i] = DEF_MOTOR_SPEED;
    comms->recordingEnabled[i] = 0;
    comms->recordingActive[i] = 0;
    /* Set the default maximum speed to something reasonable */
    comms->maxSpeed[i] = DEF_MOTOR_MAXSPEED;
  }
  comms->shiftDataGlobalEnable = 0;
  comms->shiftDataGlobal = 0;
  comms->exitState = ROBOT_NEUTRAL;
  comms->wheelRadius = 0;
  comms->distanceOffset = 0;
  comms->thread = (THREAD_T*)malloc(sizeof(THREAD_T));
  comms->commsThread = (THREAD_T*)malloc(sizeof(THREAD_T));
  THREAD_CREATE(comms->thread, nullThread, NULL);
  comms->commsLock = (MUTEX_T*)malloc(sizeof(MUTEX_T));
  MUTEX_INIT(comms->commsLock);
  comms->recordingLock = (MUTEX_T*)malloc(sizeof(MUTEX_T));
  MUTEX_INIT(comms->recordingLock);
  comms->recordingActive_lock = (MUTEX_T*)malloc(sizeof(MUTEX_T));
  MUTEX_INIT(comms->recordingActive_lock);
  comms->recordingActive_cond = (COND_T*)malloc(sizeof(COND_T));
  COND_INIT(comms->recordingActive_cond);
  comms->motionInProgress = 0;
  MUTEX_NEW(comms->recvBuf_lock);
  MUTEX_INIT(comms->recvBuf_lock);
  COND_NEW(comms->recvBuf_cond);
  COND_INIT(comms->recvBuf_cond);
  comms->recvBuf_ready = 0;
  comms->commsEngine_bytes = 0;

  comms->commsWaitingForMessage = 0;
  MUTEX_NEW(comms->commsWaitingForMessage_lock);
  MUTEX_INIT(comms->commsWaitingForMessage_lock);
#if 0
  /* deprecated by libsfp */

  MUTEX_NEW(comms->socket_lock);
  MUTEX_INIT(comms->socket_lock);
#endif
  MUTEX_NEW(comms->callback_lock);
  MUTEX_INIT(comms->callback_lock);
  comms->callbackEnabled = 0;
  comms->eventthread = (THREAD_T*)malloc(sizeof(THREAD_T));

#if 0
  /* deprecated by libsfp */

  comms->commsOutThread = (THREAD_T*)malloc(sizeof(THREAD_T));
  THREAD_CREATE(comms->commsOutThread, nullThread, NULL);
  comms->sendBuf = (uint8_t*)malloc(SENDBUF_SIZE);
  comms->sendBuf_index = 0;
  comms->sendBuf_N = 0;
  MUTEX_NEW(comms->sendBuf_lock);
  MUTEX_INIT(comms->sendBuf_lock);
  COND_NEW(comms->sendBuf_cond);
  COND_INIT(comms->sendBuf_cond);
#endif

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
  char path[MAX_PATH];
  strcpy(path, getenv("HOME"));
  strcat(path, "/.Barobo.config");
  comms->configFilePath = strdup(path);
#endif
  comms->numItemsToFreeOnExit = 0;
  memset(comms->serialID, 5, sizeof(char));
  MUTEX_NEW(comms->mobotTree_lock);
  MUTEX_INIT(comms->mobotTree_lock);
  COND_NEW(comms->mobotTree_cond);
  COND_INIT(comms->mobotTree_cond);
  comms->parent = NULL;
  comms->children = NULL;

  MUTEX_NEW(comms->scan_callback_lock);
  MUTEX_INIT(comms->scan_callback_lock);
  comms->jointCallback = NULL;
  comms->accelCallback = NULL;
  comms->eventCallback = NULL;
  comms->eventCallbackData = NULL;

  /* FIXME properly abstract links */
  comms->dongle = NULL;

  comms->eventqueue = new RingBuf<event_t*>();

  return 0;
}

int Mobot_isConnected(mobot_t* comms)
{
  int status;
  int rc;
  if(comms->connected == 0) {
    return 0;
  }
  rc = Mobot_getStatus(comms);
  if(rc) {return 0;}
  return 1;
}

int Mobot_protocolVersion()
{
  return CMD_NUMCOMMANDS;
}

int Mobot_pair(mobot_t* mobot)
{
  int status;
  uint8_t buf[8];
  if(mobot->parent == NULL) {
    return -1;
  }
  buf[0] = mobot->parent->zigbeeAddr>>8;
  buf[1] = mobot->parent->zigbeeAddr & 0x00ff;
  status = MobotMsgTransaction(mobot , BTCMD(CMD_PAIRPARENT), buf, 2);
  if(status < 0) return status;
  /* Make sure the buf size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_unpair(mobot_t* mobot)
{
  int status;
  uint8_t buf[8];
  if(mobot->parent == NULL) {
    return -1;
  }
  status = MobotMsgTransaction(mobot, BTCMD(CMD_UNPAIRPARENT), buf, 0);
  if(status < 0) return status;
  /* Make sure the buf size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_reset(mobot_t* comms)
{
  uint8_t buf[64];
  int status;
  status = MobotMsgTransaction(comms, BTCMD(CMD_RESETABSCOUNTER), buf, 0);
  if(status < 0) return status;
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_resetToZero(mobot_t* comms) {
  Mobot_reset(comms);
  return Mobot_moveToZero(comms);
}

int Mobot_resetToZeroNB(mobot_t* comms) {
  Mobot_reset(comms);
  return Mobot_moveToZeroNB(comms);
}

int Mobot_setAccelEventThreshold(mobot_t* comms, double threshold)
{
  uint8_t buf[32];
  uint16_t thresh = threshold * 16384.0;
  int status;
  buf[0] = thresh >> 8;
  buf[1] = thresh & 0x00ff;
  status = MobotMsgTransaction(comms, BTCMD(CMD_SET_ACCEL_EVENT_THRESHOLD), buf, 2);
  if(status < 0) return status;
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_setJointEventThreshold(mobot_t* comms, int joint, double threshold)
{
  uint8_t buf[32];
  float thresh = threshold;
  int status;
  if( (joint < 1) || (joint > 4) ) {
    return -1;
  }
  buf[0] = joint-1;
  memcpy(&buf[1], &thresh, 4);
  status = MobotMsgTransaction(comms, BTCMD(CMD_SET_JOINT_EVENT_THRESHOLD), buf, 5);
  if(status < 0) return status;
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_setFourierCoefficients(mobot_t* comms, robotJointId_t id, double* a, double* b)
{
  uint8_t buf[32];
  int8_t coefs[10];
  int i, status;
  buf[0] = (uint8_t)id-1;
  for(i = 0; i < 5; i++) {
    coefs[i] = (a[i]/5.0) * 128;
    coefs[i+5] = (b[i]/5.0) * 128;
  }
  memcpy(&buf[1], &coefs[0], 10);
  status = MobotMsgTransaction(comms, BTCMD(CMD_SETFOURIERCOEFS), buf, 10);
  if(status < 0) return status;
  /* Make sure the data size is correct */
  if(buf[1] != 3) {
    return -1;
  }
  return 0;
}

int Mobot_twiSend(mobot_t* comms, uint8_t addr, uint8_t* buf, int size)
{
  uint8_t* sendbuf = (uint8_t*)malloc(size+10);
  sendbuf[0] = addr;
  sendbuf[1] = size;
  memcpy(&sendbuf[2], buf, size);
  sendbuf[2+size] = 0x00;
  int rc = MobotMsgTransaction(comms, BTCMD(CMD_TWI_SEND), sendbuf, 3+size);
  free(sendbuf);
  return rc;
}

int Mobot_twiRecv(mobot_t* comms, uint8_t addr, void* buf, int size)
{
  uint8_t* sendbuf = (uint8_t*)malloc(size+10);
  sendbuf[0] = addr;
  sendbuf[1] = size;
  sendbuf[2] = 0x0;
  int rc = MobotMsgTransaction(comms, BTCMD(CMD_TWI_RECV), sendbuf, 3);
  if(rc == 0) memcpy(buf, &sendbuf[2], sendbuf[1]-3);
  free(sendbuf);
  return rc;
}

int Mobot_twiSendRecv(mobot_t* comms, uint8_t addr, 
    uint8_t* sendbuf, int sendsize,
    void* recvbuf, int recvsize)
{
  uint8_t *buf = (uint8_t*)malloc(sendsize+recvsize+10);
  buf[0] = addr;
  buf[1] = sendsize;
  memcpy(&buf[2], sendbuf, sendsize);
  buf[2+sendsize] = recvsize;
  buf[3+sendsize] = 0x00;
  int rc = MobotMsgTransaction(comms, BTCMD(CMD_TWI_SENDRECV), buf, 4+sendsize);
  if(rc == 0) {
    memcpy(recvbuf, &buf[2], buf[1]-3);
  }
  return rc;
}

int Mobot_waitForReportedSerialID(mobot_t* comms, char* id) 
{
  /* Wait on the mobot tree condition variable... Return if our mobot shows up
   * or we time out */
  int rc;
  mobotInfo_t *iter;
#ifndef _WIN32
  struct timespec ts;
#else
#endif
  /* Wait until transaction is ready */
  MUTEX_LOCK(comms->mobotTree_lock);
  while(1) {
#ifndef _WIN32
    /* Set up a wait with timeout */
    /* Get the current time */
#ifndef __MACH__
    clock_gettime(CLOCK_REALTIME, &ts);
#else
    clock_serv_t cclock;
    mach_timespec_t mts;
    host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
    clock_get_time(cclock, &mts);
    mach_port_deallocate(mach_task_self(), cclock);
    ts.tv_sec = mts.tv_sec;
    ts.tv_nsec = mts.tv_nsec;
#endif
    /* Add a timeout */
    ts.tv_sec += 1;
    rc = pthread_cond_timedwait(
      comms->mobotTree_cond, 
      comms->mobotTree_lock,
      &ts);
    if(rc) {
      /* Timed out */
      /* return error */
      MUTEX_UNLOCK(comms->mobotTree_lock);
      return -1;
    } else {
      /* We got an entry. Cycle through the tree and see if the one we are
       * looking for arrived */
      for(iter = comms->children; iter != NULL; iter = iter->next) {
        if(!strcmp(id, iter->serialID)) {
          /* We found it. We can return now */
          MUTEX_UNLOCK(comms->mobotTree_lock);
          return 0;
        }
      }
    }
#else
    ResetEvent(*comms->mobotTree_cond);
    ReleaseMutex(*comms->mobotTree_lock);
    rc = WaitForSingleObject(*comms->recvBuf_cond, 1000);
    if(rc == WAIT_TIMEOUT) {
      /* Timed out */
      /* return error */
      MUTEX_UNLOCK(comms->mobotTree_lock);
      return -1;
    } else {
      /* We got an entry. Cycle through the tree and see if the one we are
       * looking for arrived */
      for(iter = comms->children; iter != NULL; iter = iter->next) {
        if(!strcmp(id, iter->serialID)) {
          /* We found it. We can return now */
          MUTEX_UNLOCK(comms->mobotTree_lock);
          return 0;
        }
      }
    }
#endif
  }
}

/* This function does a complete message transaction with a Mobot, including
 * sending the message, waiting for a response, and resending the message in
 * case a response times out. The buffer "buf" is used as both the send buffer
 * and receive buffer, so care must be taken to ensure that it is large enough
 * to hold any response from the Mobot. */
int MobotMsgTransaction(mobot_t* comms, uint8_t cmd, /*IN&OUT*/ void* buf, int size)
{
  int retries = 0;
  int rc = 1;
  void* sendbuf;
  if(size > 0) {
    sendbuf = malloc(size); 
    memcpy(sendbuf, buf, size);
  }
  while(
      (retries <= MAX_RETRIES) &&
      (rc != 0)
      ) 
  {
    MUTEX_LOCK(comms->commsWaitingForMessage_lock);
    comms->commsWaitingForMessage = 1;
    MUTEX_UNLOCK(comms->commsWaitingForMessage_lock);
    SendToIMobot(comms, cmd, sendbuf, size);
    rc = RecvFromIMobot(comms, (uint8_t*)buf, size);
    MUTEX_LOCK(comms->commsWaitingForMessage_lock);
    comms->commsWaitingForMessage = 0;
    MUTEX_UNLOCK(comms->commsWaitingForMessage_lock);
    retries++;
  }
  if(size > 0) free(sendbuf);
  if(rc) {return rc;}
  if(((uint8_t*)buf)[0] == 0xff) {
    return -1;
  }
  return 0;
}

int SendToIMobot(mobot_t* comms, uint8_t cmd, const void* data, int datasize)
{
  int err = 0;
  int i;
  int len;
  uint8_t str[1024];
  if(comms->connected == 0) {
    return -1;
  }
  MUTEX_LOCK(comms->commsLock);
  comms->recvBuf_ready = 0;

  if(
      (comms->connectionMode == MOBOTCONNECT_BLUETOOTH) ||
      (comms->connectionMode == MOBOTCONNECT_TCP) 
    ) 
  {
    str[0] = cmd;
    str[1] = datasize + 3;
    if(datasize > 0) {
      memcpy(&str[2], data, datasize);
    }
    str[datasize+2] = MSG_SENDEND;
    len = datasize + 3;
  } else {
    str[0] = cmd;
    str[1] = datasize + 8;
    if(comms->connectionMode == MOBOTCONNECT_ZIGBEE) {
      str[2] = comms->zigbeeAddr >> 8;
      str[3] = comms->zigbeeAddr & 0x00ff;
    } else {
      str[2] = 0x00;
      str[3] = 0x00;
    }
    str[4] = 1;
    str[5] = cmd;
    str[6] = datasize + 3;

    if(datasize > 0) memcpy(&str[7], data, datasize);
    str[datasize+7] = MSG_SENDEND;
    len = datasize + 8;
  }
#if 0
  char* str;
  str = (char*)malloc(strlen(buf)+2);
  strcpy(str, buf);
  strcat(str, "$");
  len++;
#endif
  //printf("SEND %d: <<%s>>\n", comms->socket, str);
#ifdef COMMSDEBUG
  printf("SEND: ");
  for(i = 0; i < len; i++) {
    printf("0x%x ", str[i]);
  }
  printf("\n");
#endif
  //Sleep(1);

  if(comms->connectionMode == MOBOTCONNECT_ZIGBEE) {
    assert(MOBOTCONNECT_TTY == comms->parent->connectionMode);

    /* Send the message out on the parent's dongle. */
    if (-1 == dongleWrite(comms->parent->dongle, str, len)) {
      return -1;
    }
  } else if (comms->connectionMode == MOBOTCONNECT_TTY) {
    /* Send the message out on our dongle. */
    if (-1 == dongleWrite(comms->dongle, str, len)) {
      return -1;
    }
  } else {
    //MUTEX_LOCK(comms->socket_lock);
#ifndef _WIN32
    err = write(comms->socket, str, len);
#else
    err = send(comms->socket, (const char*)str, len, 0);
#endif
    //MUTEX_UNLOCK(comms->socket_lock);
  }
  return 0;
}

#if 0
/* hlh: unused? */

int SendToMobotDirect(mobot_t* comms, const void* data, int datasize)
{
  int err;
  if(comms->connected == 0) {
    return -1;
  }
  MUTEX_LOCK(comms->commsLock);
  comms->recvBuf_ready = 0;
  if(comms->connected == 1) {
#ifdef _WIN32
    err = send(comms->socket, (const char*)data, datasize, 0);
#else
    err = write(comms->socket, data, datasize);
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
#endif

int RecvFromIMobot(mobot_t* comms, uint8_t* buf, int size)
{
  int rc;
#ifndef _WIN32
  struct timespec ts;
#else
#endif
  /* Wait until transaction is ready */
  MUTEX_LOCK(comms->recvBuf_lock);
  while(!comms->recvBuf_ready) {
#ifndef _WIN32
    /* Set up a wait with timeout */
    /* Get the current time */
#ifndef __MACH__
    clock_gettime(CLOCK_REALTIME, &ts);
#else
    clock_serv_t cclock;
    mach_timespec_t mts;
    host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
    clock_get_time(cclock, &mts);
    mach_port_deallocate(mach_task_self(), cclock);
    ts.tv_sec = mts.tv_sec;
    ts.tv_nsec = mts.tv_nsec;
#endif
    /* Add a timeout */
    ts.tv_nsec += 700000000; // 100 ms
    if(ts.tv_nsec > 1000000000) {
      ts.tv_nsec -= 1000000000;
      ts.tv_sec += 1;
    }
    rc = pthread_cond_timedwait(
      comms->recvBuf_cond, 
      comms->recvBuf_lock,
      &ts);
    /*
    rc = pthread_cond_wait(
      comms->recvBuf_cond, 
      comms->recvBuf_lock);
      */
    if(rc) {
      char errbuf[256];
      /* pthread_cond_timedwait returns the error code rather than setting
       * errno. */
      strerror_r(rc, errbuf, sizeof(errbuf));
      fprintf(stderr, "(barobo) ERROR: in RecvFromIMobot, "
          "pthread_cond_timedwait: %s\n", errbuf);

      /* Reset the incoming message queue */
      comms->commsEngine_bytes = 0;
      /* Disconnect and return error */
      MUTEX_UNLOCK(comms->recvBuf_lock);
      MUTEX_UNLOCK(comms->commsLock);
      //Mobot_disconnect(comms);
      return -2;
    }
#else
    ResetEvent(*comms->recvBuf_cond);
    ReleaseMutex(*comms->recvBuf_lock);
    rc = WaitForSingleObject(*comms->recvBuf_cond, 700);
    if(rc == WAIT_TIMEOUT) {
      MUTEX_UNLOCK(comms->recvBuf_lock);
      MUTEX_UNLOCK(comms->commsLock);
      //Mobot_disconnect(comms);
      return -2;
    }
#endif
  }
  memcpy(buf, comms->recvBuf, comms->recvBuf_bytes);

  /* Print out results */
  int i;
  /*
  printf("RECV: ");
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

void sigint_handler(int sig)
{
  if(!g_disconnectSignal) {
    exit(0);
  }
  g_disconnectSignal = 0;
}

static void Mobot_processMessage (mobot_t *comms, uint8_t *buf, size_t len);

/* The comms engine will watch the incoming comm channel for any message. If a
 * message is expected, it will get the data to RecvFromIMobot(). If it was
 * triggered by an event, then the appropriate callback will be called. */
void* commsEngine(void* arg)
{
  mobot_t* comms = (mobot_t*)arg;
  uint8_t byte;
  int err;
#ifndef _WIN32
  struct sigaction int_handler;
  int_handler.sa_handler = sigint_handler;
  sigaction(SIGINT, &int_handler, 0);
#endif
  g_mobotThreadInitializing = 0;
  while(1) {
    if (MOBOTCONNECT_TTY == comms->connectionMode) {
      uint8_t buf[256];
      long len = dongleRead(comms->dongle, buf, sizeof(buf));
      /* Either error or EOF. */
      if (len <= 0) {
        break;
      }
#ifdef COMMSDEBUG
      int i;
      printf("Recv: ");
      for(i = 0; i < len; i++) {
        printf("0x%X ", buf[i]);
      }
      printf("\n");
#endif
      Mobot_processMessage(comms, buf, len);
    }
    else {
      /* Try and receive a byte */
#ifndef _WIN32
      err = read(comms->socket, &byte, 1);
      /* Check to see if we were interrupted */
      if(-1 == err) {
        if(errno == EINTR) {
          if(comms->connected == 0) {
            break;
          }
        }
        else {
          char barf[256];
          strerror_r(errno, barf, 256);
          fprintf(stderr, "(barobo) ERROR: read(): %s\n", barf);
          break;
        }
      }
#else
      err = recvfrom(comms->socket, (char*)&byte, 1, 0, (struct sockaddr*)0, 0);
#endif
    }
    /* If we are no longer connected, just return */
    if(comms->connected == 0) {
      return NULL;
    }
    if(err <= 0) {
      continue;
    }

    /* DEBUG */
#ifdef COMMSDEBUG
    /* hlh: note that comms->commsEngine_bytes will probably always be zero
     * now for TTY-connected mobots. */
    printf("%d RECV: 0x%0x\n", comms->commsEngine_bytes, byte);
#endif

    if (MOBOTCONNECT_TTY != comms->connectionMode) {
      MUTEX_LOCK(comms->recvBuf_lock);
      comms->recvBuf[comms->commsEngine_bytes] = byte;
      comms->commsEngine_bytes++;
      MUTEX_UNLOCK(comms->recvBuf_lock);
      if (comms->recvBuf[1] == comms->commsEngine_bytes) {
        Mobot_processMessage(comms, comms->recvBuf, comms->commsEngine_bytes);
      }
    }
  }
  Mobot_disconnect(comms);
  return NULL;
}

void* eventThread(void* arg)
{
  mobot_t* comms = (mobot_t*)arg;
  int addressFound;
  while(comms->connected) {
    comms->eventqueue->lock();
    while(comms->eventqueue->num() <= 0) {
      comms->eventqueue->wait();
    }
    /* Got an event. Process dat bitch */
    event_t* event = comms->eventqueue->pop();
    comms->eventqueue->unlock();
    switch(event->event) {
      case EVENT_BUTTON:
        MUTEX_LOCK(comms->callback_lock);
        if(comms->callbackEnabled && comms->buttonCallback) {
          int bit;
          for(bit = 0; bit < 3; bit++) {
            if(event->data.button_data.event_mask & (1<<bit)) {
              comms->buttonCallback(comms->mobot, bit, (event->data.button_data.down_mask & (1<<bit)) ? 1 : 0);
            }
          }
        }
        MUTEX_UNLOCK(comms->callback_lock);
        break;
      case EVENT_REPORTADDRESS:
        /* Check the list to see if the reported address aready exists */
        MUTEX_LOCK(comms->mobotTree_lock);
        addressFound = 0;
        mobotInfo_t* iter;
        for(iter = comms->children; iter != NULL; iter = iter->next) {
          if(!strncmp(iter->serialID, event->data.reportAddress.serialID, 4)) {
            MUTEX_LOCK(comms->scan_callback_lock);
            if(comms->scan_callback) {
              comms->scan_callback(iter->serialID);
            }
            MUTEX_UNLOCK(comms->scan_callback_lock);
            addressFound = 1;
            break;
          }
        }
        /* If the address is not found, add a new Mobot with that address to
         * the head of the list */
        if(!addressFound) {
          mobotInfo_t* tmp = (mobotInfo_t*)malloc(sizeof(mobotInfo_t));
          memset(tmp, 0, sizeof(mobotInfo_t));
          /* Copy the zigbee address */
          tmp->zigbeeAddr = event->data.reportAddress.address;
          /* Copy the serial number */
          memcpy(tmp->serialID, event->data.reportAddress.serialID, 4);
          /* Set the parent */
          tmp->parent = comms;
          /* Stick it on the head of the list */
          tmp->next = comms->children;
          comms->children = tmp;
          MUTEX_LOCK(comms->scan_callback_lock);
          if(comms->scan_callback) {
            comms->scan_callback(tmp->serialID);
          }
          MUTEX_UNLOCK(comms->scan_callback_lock);
        }
        COND_SIGNAL(comms->mobotTree_cond);
        MUTEX_UNLOCK(comms->mobotTree_lock);
        break;
      case EVENT_DEBUG_MSG:
        printf("Debug Message received from %s: %s\n",
            comms->serialID, event->data.debug_data);
        break;
      case EVENT_JOINT_MOVED:
        MUTEX_LOCK(comms->callback_lock);
        if(comms->jointCallback) {
          comms->jointCallback(
              event->millis,
              event->data.joint_data.angles[0],
              event->data.joint_data.angles[1],
              event->data.joint_data.angles[2],
              event->data.joint_data.angles[3],
              event->data.joint_data.mask,
              comms->jointCallbackData);
        }
        MUTEX_UNLOCK(comms->callback_lock);
        break;
      case EVENT_ACCEL_CHANGED:
        MUTEX_LOCK(comms->callback_lock);
        if(comms->accelCallback) {
          comms->accelCallback(
              event->millis,
              event->data.accel_data[0]/16384.0,
              event->data.accel_data[1]/16384.0,
              event->data.accel_data[2]/16384.0,
              comms->accelCallbackData);
        }
        MUTEX_UNLOCK(comms->callback_lock);
        break;
    }
    delete event;
  }
  return NULL;
}

/* Formerly part of commsEngine */
static void Mobot_processMessage (mobot_t *comms, uint8_t *buf, size_t len) {
  uint16_t uint16;
  mobotInfo_t* iter;

  /* This function should be running under commsEngine's thread, so it's okay
   * to do this right off the bat, I think. */
  comms->commsEngine_bytes = 0;

  assert(comms && buf);

  if( len < 2 ||
      (buf[1] > 255) ) {
    /* We cannot accept a message of this size, or it's an incomplete message. */
    return;
  }

  if (RESP_OK == buf[0]
      || RESP_ERR == buf[0]
      || RESP_ALREADY_PAIRED == buf[0]) {
    /* We have received the entire response */
    /* We need to copy it to the correct receive buffer */
    /* First, check to see if we are using the old Bluetooth protocol or
     * the new Zigbee protocol... */
    int delivered_message = 0;  // boolean to see if we succeed in delivering
    if(
        (comms->connectionMode == MOBOTCONNECT_BLUETOOTH) ||
        (comms->connectionMode == MOBOTCONNECT_TCP)
      ) 
    {
      MUTEX_LOCK(comms->recvBuf_lock);
      memmove(comms->recvBuf, buf, len);
      comms->recvBuf_ready = 1;
      comms->recvBuf_bytes = comms->recvBuf[1];
      delivered_message = 1;
      COND_BROADCAST(comms->recvBuf_cond);
      MUTEX_UNLOCK(comms->recvBuf_lock);
    } else {
      /* Check to see if it matches our address */
      uint16 = 0;
      uint16 = buf[2]<<8;
      uint16 |= buf[3] & 0x00ff;
      if(uint16 == 0) {
        /* Make sure the parent is waiting for a message first */
        MUTEX_LOCK(comms->commsWaitingForMessage_lock);
        if(comms->commsWaitingForMessage) {
          /* Address of 0 means the connected TTY mobot */
          MUTEX_LOCK(comms->recvBuf_lock);
          memmove(comms->recvBuf, &buf[5], buf[6]);
          comms->recvBuf_ready = 1;
          comms->recvBuf_bytes = comms->recvBuf[1];
          delivered_message = 1;
          COND_BROADCAST(comms->recvBuf_cond);
          MUTEX_UNLOCK(comms->recvBuf_lock);
        } else if (comms->child != NULL) {
          MUTEX_LOCK(comms->child->recvBuf_lock);
          MUTEX_LOCK(comms->recvBuf_lock);
          memmove(comms->child->recvBuf, &buf[5], buf[6]);
          comms->child->recvBuf_ready = 1;
          comms->child->recvBuf_bytes = comms->child->recvBuf[1];
          delivered_message = 1;
          COND_BROADCAST(comms->child->recvBuf_cond);
          MUTEX_UNLOCK(comms->child->recvBuf_lock);
          MUTEX_UNLOCK(comms->recvBuf_lock);
        }
        MUTEX_UNLOCK(comms->commsWaitingForMessage_lock);
      } else if ((comms->child != NULL) && (comms->child->zigbeeAddr == uint16)) {
        MUTEX_LOCK(comms->child->recvBuf_lock);
        MUTEX_LOCK(comms->recvBuf_lock);
        memmove(comms->child->recvBuf, &buf[5], buf[6]);
        comms->child->recvBuf_ready = 1;
        comms->child->recvBuf_bytes = comms->child->recvBuf[1];
        delivered_message = 1;
        COND_BROADCAST(comms->child->recvBuf_cond);
        MUTEX_UNLOCK(comms->child->recvBuf_lock);
        MUTEX_UNLOCK(comms->recvBuf_lock);
      } else { 
        /* See if it matches any of our children */
        for(iter = comms->children; iter != NULL; iter = iter->next) {
          if(uint16 == iter->zigbeeAddr) {
            MUTEX_LOCK(iter->mobot->recvBuf_lock);
            memcpy(((mobot_t*)iter->mobot)->recvBuf, &buf[5], buf[6]);
            iter->mobot->recvBuf_ready = 1;
            iter->mobot->recvBuf_bytes = iter->mobot->recvBuf[1];
            delivered_message = 1;
            COND_BROADCAST(iter->mobot->recvBuf_cond);
            MUTEX_UNLOCK(iter->mobot->recvBuf_lock);
            break;
          }
        }
      }
    }
    if (!delivered_message) {
      //fprintf(stderr, "(barobo) ERROR: message received from disconnected robot, ignoring\n");
    }
  } else {
    /* It was a user triggered event */
    /* First, we need to see which mobot initiated the button press */
#ifdef _WIN32
    event_t *event = static_cast<event_t*>(malloc(sizeof(event_t)));
#else
    event_t *event = new event_t;
#endif
    event->address = buf[2] << 8;
    event->address |= buf[3] & 0x00ff;
    event->event = buf[0];
#ifdef COMMSDEBUG
    printf("Received event from 0x%x: %d\n", event->address, event->event);
#endif
    switch(buf[0]) {
      case EVENT_BUTTON:
        memcpy(&event->millis, &buf[7], 4);
        event->data.button_data.event_mask = buf[11];
        event->data.button_data.down_mask = buf[12];
        event->data.button_data.up_mask = buf[13];
        break;
      case EVENT_JOINT_MOVED:
        memcpy(&event->millis, &buf[7], 4);
        memcpy(&event->data.joint_data.angles[0], &buf[11], 16);
        for(int i = 0; i < 4; i++) {
          event->data.joint_data.angles[i] = RAD2DEG(event->data.joint_data.angles[i]);
        }
        event->data.joint_data.mask = buf[27];
        break;
      case EVENT_ACCEL_CHANGED:
        memcpy(&event->millis, &buf[7], 4);
        for(int i = 0; i < 3; i++) {
          event->data.accel_data[i] = buf[11+2*i] << 8;
          event->data.accel_data[i] |= buf[12+2*i];
        }
        break;
      case EVENT_REPORTADDRESS:
        event->data.reportAddress.address = buf[7] << 8;
        event->data.reportAddress.address |= buf[8];
        memcpy(&event->data.reportAddress.serialID[0], &buf[9], 4);
        event->data.reportAddress.serialID[4] = '\0';
        break;
      case EVENT_DEBUG_MSG:
        event->data.debug_data = new char[buf[1]];
        memcpy(event->data.debug_data, &buf[7], buf[6]-3);
        event->data.debug_data[buf[6]-2] = '\0';
        break;
    }
    /* Now put the event in the correct event queue */
    /* DEBUG */
    if( 
        (event->address == 0 ) ||
        (event->event == EVENT_REPORTADDRESS)
      ) 
    {
      if(event->event == EVENT_REPORTADDRESS) {
        /* Check the list to see if the reported address aready exists */
        MUTEX_LOCK(comms->mobotTree_lock);
        int addressFound = 0;
        mobotInfo_t* iter;
        for(iter = comms->children; iter != NULL; iter = iter->next) {
          if(!strncmp(iter->serialID, (const char*)&buf[4], 4)) {
            MUTEX_LOCK(comms->scan_callback_lock);
            if(comms->scan_callback) {
              comms->scan_callback(iter->serialID);
            }
            MUTEX_UNLOCK(comms->scan_callback_lock);
            addressFound = 1;
            break;
          }
        }
        /* If the address is not found, add a new Mobot with that address to
         * the head of the list */
        if(!addressFound) {
          mobotInfo_t* tmp = (mobotInfo_t*)malloc(sizeof(mobotInfo_t));
          memset(tmp, 0, sizeof(mobotInfo_t));
          /* Copy the zigbee address */
          tmp->zigbeeAddr = buf[2] << 8;
          tmp->zigbeeAddr |= buf[3] & 0x00ff;
          /* Copy the serial number */
          memcpy(tmp->serialID, &buf[4], 4);
          /* Set the parent */
          tmp->parent = comms;
          /* Stick it on the head of the list */
          tmp->next = comms->children;
          comms->children = tmp;
          MUTEX_LOCK(comms->scan_callback_lock);
          if(comms->scan_callback) {
            comms->scan_callback(tmp->serialID);
          }
          MUTEX_UNLOCK(comms->scan_callback_lock);
        }
        COND_SIGNAL(comms->mobotTree_cond);
        MUTEX_UNLOCK(comms->mobotTree_lock);
      } else {
        comms->eventqueue->lock();
        comms->eventqueue->push(event);
        comms->eventqueue->signal();
        comms->eventqueue->unlock();
        if(comms->eventCallback) {
          comms->eventCallback(&buf[5], buf[6], comms->eventCallbackData);
        }
      }
    } else if ((comms->child != NULL) && (comms->child->zigbeeAddr == event->address)) {
      comms->child->eventqueue->lock();
      comms->child->eventqueue->push(event);
      comms->child->eventqueue->signal();
      comms->child->eventqueue->unlock();
      if(comms->child->eventCallback) {
        comms->child->eventCallback(&buf[5], buf[6], comms->child->eventCallbackData);
      }
    } else {
      /* See if it is one of the connected children */
      bool success = false;
      for(iter = comms->children; iter != NULL; iter = iter->next) {
        if(event->address == iter->zigbeeAddr) {
          iter->mobot->eventqueue->lock();
          iter->mobot->eventqueue->push(event);
          iter->mobot->eventqueue->signal();
          iter->mobot->eventqueue->unlock();
          if(iter->mobot->eventCallback) {
            iter->mobot->eventCallback(&buf[5], buf[6], iter->mobot->eventCallbackData);
          }
          success = true;
          break;
        }
      }
      if(!success) {
        delete event;
      }
    }
  }
#if 0
  else if (EVENT_BUTTON == buf[0]
      || EVENT_REPORTADDRESS == buf[0]
      || EVENT_DEBUG_MSG == buf[0]) {

    /* It was a user triggered event */
    uint16_t address;
    uint8_t events;
    uint8_t buttonDown;
    if (EVENT_BUTTON == buf[0]) {
      /* We got the entire message */
      MUTEX_LOCK(comms->callback_lock);
      /* First, we need to see which mobot initiated the button press */
      address = buf[2] << 8;
      address |= buf[3] & 0x00ff;
      if( address == 0 ) {
        if(comms->callbackEnabled) {
          /* Call the callback multiple times depending on the events */
          int bit;
          if(
              (comms->connectionMode == MOBOTCONNECT_BLUETOOTH) ||
              (comms->connectionMode == MOBOTCONNECT_TCP) 
            )
          {
            events = buf[6];
            buttonDown = buf[7];
          } else {
            events = buf[11];
            buttonDown = buf[12];
          }
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
      } else if (
          (comms->child) &&
          (comms->child->zigbeeAddr == address)
          ) 
      {
        if(comms->child->callbackEnabled) {
          /* Call the callback multiple times depending on the events */
          int bit;
          events = buf[11];
          buttonDown = buf[12];
          THREAD_T callbackThreadHandle;
          callbackArg_t* callbackArg;
          for(bit = 0; bit < 2; bit++) {
            if(events & (1<<bit)) {
              callbackArg = (callbackArg_t*)malloc(sizeof(callbackArg_t));
              callbackArg->comms = comms->child;
              callbackArg->button = bit;
              callbackArg->buttonDown = (buttonDown & (1<<bit)) ? 1 : 0;
              //comms->child->buttonCallback(bit, (buttonDown & (1<<bit)) ? 1 : 0 );
              THREAD_CREATE(&callbackThreadHandle, callbackThread, callbackArg);
            }
          }
        }
      } else {
        /* Perhaps a child triggered the event? Cycle through all children
         * to see if we can match the address. */
        mobotInfo_t* target;
        for(target = comms->children; target != NULL; target = target->next) {
          if(target->zigbeeAddr == address) {
            if(target->mobot == NULL) {
              break;
            }
            if(target->mobot->callbackEnabled) {
              /* Call the callback multiple times depending on the events */
              int bit;
              THREAD_T callbackThreadHandle;
              callbackArg_t* callbackArg;
              events = buf[11];
              buttonDown = buf[12];
              for(bit = 0; bit < 2; bit++) {
                if(events & (1<<bit)) {
                  callbackArg = (callbackArg_t*)malloc(sizeof(callbackArg_t));
                  callbackArg->comms = target->mobot;
                  callbackArg->button = bit;
                  callbackArg->buttonDown = (buttonDown & (1<<bit)) ? 1 : 0;
                  //target->mobot->buttonCallback(bit, (buttonDown & (1<<bit)) ? 1 : 0 );
                  THREAD_CREATE(&callbackThreadHandle, callbackThread, callbackArg);
                }
              }
            }
            break;
          }
        }
      }
      MUTEX_UNLOCK(comms->callback_lock);
    } else if (EVENT_REPORTADDRESS == buf[0]) {
      /* Check the list to see if the reported address aready exists */
      MUTEX_LOCK(comms->mobotTree_lock);
      int addressFound = 0;
      mobotInfo_t* iter;
      for(iter = comms->children; iter != NULL; iter = iter->next) {
        if(!strncmp(iter->serialID, (const char*)&buf[4], 4)) {
          MUTEX_LOCK(comms->scan_callback_lock);
          if(comms->scan_callback) {
            comms->scan_callback(iter->serialID);
          }
          MUTEX_UNLOCK(comms->scan_callback_lock);
          addressFound = 1;
          break;
        }
      }
      /* If the address is not found, add a new Mobot with that address to
       * the head of the list */
      if(!addressFound) {
        mobotInfo_t* tmp = (mobotInfo_t*)malloc(sizeof(mobotInfo_t));
        memset(tmp, 0, sizeof(mobotInfo_t));
        /* Copy the zigbee address */
        tmp->zigbeeAddr = buf[2] << 8;
        tmp->zigbeeAddr |= buf[3] & 0x00ff;
        /* Copy the serial number */
        memcpy(tmp->serialID, &buf[4], 4);
        /* Set the parent */
        tmp->parent = comms;
        /* Stick it on the head of the list */
        tmp->next = comms->children;
        comms->children = tmp;
        MUTEX_LOCK(comms->scan_callback_lock);
        if(comms->scan_callback) {
          comms->scan_callback(tmp->serialID);
        }
        MUTEX_UNLOCK(comms->scan_callback_lock);
      }
      COND_SIGNAL(comms->mobotTree_cond);
      MUTEX_UNLOCK(comms->mobotTree_lock);
    }
    else if (EVENT_DEBUG_MSG == buf[0]) {
      printf("Received Message from %s: %s\n", comms->serialID, (const char*)&buf[2]);
    }
  }
#endif
}

#if 0
#ifdef _WIN32
void* commsOutEngine(void* arg)
{
  mobot_t* comms = (mobot_t*)arg;
  int index;
  uint8_t* byte;
  uint8_t b;
  int err;
  DWORD bytesWritten;
  uint8_t msgbuf[256];
  int i = 0;
  //comms->ovOutgoing.hEvent = CreateEvent(0, true, 0, 0);

  MUTEX_LOCK(comms->sendBuf_lock);
  g_mobotThreadInitializing = 0;
  while(1) {
    while(
        (comms->sendBuf_N == 0) &&
        (comms->connected) ) 
    {
      COND_WAIT(comms->sendBuf_cond, comms->sendBuf_lock);
    }
    if(comms->connected == 0) {
      MUTEX_UNLOCK(comms->sendBuf_lock);
      break;
    }
    i = 0;
    /* Write one whole message at a time */
    while(comms->sendBuf_N > 0) {
      index = comms->sendBuf_index % SENDBUF_SIZE;
      byte = &comms->sendBuf[index];
      msgbuf[i] = *byte;
      comms->sendBuf_N--;
      comms->sendBuf_index++;
      i++;
    }
    //printf(" ** OUT: 0x%02x %d\n", *byte, comms->sendBuf_N);
    MUTEX_LOCK(comms->socket_lock);
    if(!WriteFile(
          comms->commHandle, 
          msgbuf, 
          i,
          NULL,
          comms->ovOutgoing)) {
      //printf("Error writing. %d \n", GetLastError());
    }
    MUTEX_UNLOCK(comms->socket_lock);
    GetOverlappedResult(comms->commHandle, comms->ovOutgoing, &bytesWritten, TRUE);
    COND_SIGNAL(comms->sendBuf_cond);
    ResetEvent(comms->ovOutgoing->hEvent);
  }
  return NULL;
}

#else
void* commsOutEngine(void* arg)
{
  mobot_t* comms = (mobot_t*)arg;
  int index;
  uint8_t* bytes = NULL;
  int n;
  int bufsize = 0;
  uint8_t b;
  int err;
  int i;

  MUTEX_LOCK(comms->sendBuf_lock);
  g_mobotThreadInitializing = 0;
  while(1) {
    while(
        (comms->sendBuf_N == 0) &&
        (comms->connected) ) 
    {
      COND_WAIT(comms->sendBuf_cond, comms->sendBuf_lock);
    }
    if(comms->connected == 0) {
      MUTEX_UNLOCK(comms->sendBuf_lock);
      break;
    }
    /* Copy the whole buffer and send it */
    if(comms->sendBuf_N > bufsize) {
      if(bytes != NULL) {
        free(bytes);
      }
      bufsize = comms->sendBuf_N;
      bytes = (uint8_t*)malloc(bufsize);
    }
    i = 0;
    while(comms->sendBuf_N > 0) {
      index = comms->sendBuf_index % SENDBUF_SIZE;
      bytes[i] = comms->sendBuf[index];
      comms->sendBuf_N--;
      comms->sendBuf_index++;
      i++;
    }
    MUTEX_UNLOCK(comms->sendBuf_lock);
    MUTEX_LOCK(comms->socket_lock);
    err = write(comms->socket, bytes, i);
    //printf("*** OUT: 0x%2x\n", *byte);
    MUTEX_UNLOCK(comms->socket_lock);
    MUTEX_LOCK(comms->sendBuf_lock);
    COND_SIGNAL(comms->sendBuf_cond);
  }
  return NULL;
}

#endif
#endif /* 0 */

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

int shiftDataIsEnabled(mobot_t* comms)
{
  if(comms->shiftData && !comms->shiftDataGlobalEnable) {
    return 1;
  } else if (comms->shiftDataGlobalEnable && comms->shiftDataGlobal) {
    return 1;
  } else {
    return 0;
  }
}

double systemTime()
{
#ifdef _WIN32
  return (GetTickCount()/1000.0);
#elif defined __MACH__
  double t;
  clock_serv_t cclock;
  mach_timespec_t mts;
  mach_timespec_t cur_time;
	host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
  clock_get_time(cclock, &mts);
  mach_port_deallocate(mach_task_self(), cclock);
  cur_time.tv_nsec = mts.tv_nsec;
  t = mts.tv_sec;
  t += (mts.tv_nsec / 1000000000.0);
  return t;
#else
  double t;
  struct timespec cur_time;
  clock_gettime(CLOCK_REALTIME, &cur_time);
  t = cur_time.tv_sec;
  t += (cur_time.tv_nsec/1000000000.0);
  return t;
#endif
}
