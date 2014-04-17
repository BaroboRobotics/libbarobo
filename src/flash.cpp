#include "libstkcomms.hpp"

#include "mobot.h"
#include "dongle.h"

#include <unistd.h>

#include <string.h>

/* Since robots can be connected multiple times if they are acting as a dongle,
 * this is a way to find a given robot's TTY alter ego. If the passed robot is
 * connected via TTY, return it, otherwise find the closest ancestor which is
 * connected via TTY and whose serial ID is the same as the passed robot's.
 * Return NULL if no such robot can be found (i.e., the passed robot has never
 * been connected via TTY). */
static mobot_t* getFlashableMobotStruct (mobot_t* comms) {
  assert(comms);

  mobot_t* bot;
  for (bot = comms; bot; bot = bot->parent) {
    if (MOBOTCONNECT_TTY == bot->connectionMode &&
        !strcmp(bot->serialID, comms->serialID)) {
      return bot;
    }
  }
  return NULL;
}

//////////////////////////////////////////////////////////////////////////////

int Mobot_canFlashFirmware (mobot_t* comms) {
  return !!getFlashableMobotStruct(comms);
}

int Mobot_flashFirmwareAsync (mobot_t* comms, const char* hexfile,
    void (*progressCallback)(double progress),
    void (*completionCallback)(int status)) {
  mobot_t* flashable = getFlashableMobotStruct(comms);

  if (!flashable) {
    return -1;
  }

  assert(flashable->dongle);

  char tty[MOBOT_DONGLE_TTYFILENAME_MAX_PATH];
  dongleGetTTYFilename(flashable->dongle, tty, sizeof(tty));

  Mobot_reboot(flashable);
  Mobot_disconnect(flashable);

  /* I have tested with zero milliseconds (i.e., a sleep for the amount of time
   * it takes to call a library function), and it worked, but I'll leave it at
   * one just to be safe. If there is no delay here, the robot will not reboot. */
#if _WIN32
  //Sleep(1);
#else
  usleep(1000);
#endif

  CStkComms* stk = new CStkComms;
  int rc = stk->connectWithTTY(tty);
  if (-1 == rc) {
    fprintf(stderr, "(barobo) ERROR: in Mobot_flashFirmware, unable to "
        "connect to %s with the STK protocol.\n", tty);
    return -1;
  }

  /* The CStkComms::DISCONNECT_AND_DELETE tells libstkcomms to clean up after
   * itself--we no longer own the object. */
  stk->programAllAsync(hexfile, 0, progressCallback, completionCallback,
      CStkComms::DISCONNECT_AND_DELETE);

  return 0;
}
