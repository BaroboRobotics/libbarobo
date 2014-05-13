#include "libstkcomms.hpp"

#include "mobot.h"
#include "dongle.h"

#include <string.h>

#if _WIN32

#include <windows.h>
static void sleep_for_ms (int delay) {
  Sleep(delay);
}

#else

#include <unistd.h>
static void sleep_for_ms (int delay) {
  usleep(delay * 1000);
}

#endif

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

struct FlashFirmwareThreadArgs {
  mobot_t* flashable;
  std::string hexfile;
  stkComms_progressCallbackFunc progressCallback;
  stkComms_completionCallbackFunc completionCallback;
  void* userData;
};

void* flashFirmwareThread (void* a) {
  FlashFirmwareThreadArgs* args = static_cast<FlashFirmwareThreadArgs*>(a);

  assert(args);

  mobot_t* flashable = args->flashable;
  std::string hexfile = args->hexfile;
  stkComms_progressCallbackFunc progressCallback = args->progressCallback;
  stkComms_completionCallbackFunc completionCallback = args->completionCallback;
  void* userData = args->userData;

  delete args;

  assert(flashable);
  assert(flashable->dongle);

  char tty[MOBOT_DONGLE_TTYFILENAME_MAX_PATH];
  dongleGetTTYFilename(flashable->dongle, tty, sizeof(tty));

  Mobot_reboot(flashable);
  Mobot_disconnect(flashable);

  sleep_for_ms(2000);

  CStkComms* stk = new CStkComms;
  const int maxTries = 30;
  int rc = -1;
  for (int i = 0; 0 != rc && i < maxTries; ++i) {
    sleep_for_ms(100);
    rc = stk->connectWithTTY(tty);
  }

  if (-1 == rc) {
    fprintf(stderr, "(barobo) ERROR: in Mobot_flashFirmware, unable to "
        "connect to %s with the STK protocol.\n", tty);
    if (completionCallback) {
      completionCallback(0, userData);
    }
    return 0;
  }

  /* The CStkComms::DISCONNECT_AND_DELETE tells libstkcomms to clean up after
   * itself--we no longer own the object. */
  stk->programAllAsync(hexfile, 0, progressCallback, completionCallback,
      userData, CStkComms::DISCONNECT_AND_DELETE);

  return 0;
}

int Mobot_flashFirmwareAsync (mobot_t* comms, const char* hexfile,
    Mobot_progressCallbackFunc progressCallback,
    Mobot_completionCallbackFunc completionCallback,
    void* userData) {
  mobot_t* flashable = getFlashableMobotStruct(comms);

  if (!flashable) {
    return -1;
  }

  THREAD_T thread;
  /* The thread will delete this FlashFirmwareThreadArgs object for us. */
  FlashFirmwareThreadArgs* args = new FlashFirmwareThreadArgs;
  args->flashable = flashable;
  args->hexfile = hexfile;
  args->progressCallback = progressCallback;
  args->completionCallback = completionCallback;
  args->userData = userData;
  THREAD_CREATE(&thread, flashFirmwareThread, args);

  return 0;
}

