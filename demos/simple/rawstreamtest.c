#include <mobot.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define NUMBOTS 1

void rawStreamCB(const uint8_t* data, size_t size, void* userdata)
{
  int i;
  for(i = 0; i < size; i++) {
    printf("0x%x\n", data[i]);
  }
}

void buttonCB(void* data, int button, int buttonDown) {
}

int main()
{
  int i = 0;
  uint8_t buf[10];
  mobot_t mobot;
  Mobot_init(&mobot);
  Mobot_connectWithAddress(&mobot, "D6ZD", 1);
  //Mobot_connectWithTTY(&mobot, "/dev/ttyACM0");
  Mobot_enableButtonCallback(&mobot, NULL, buttonCB);
  Mobot_enableRawStream(&mobot, rawStreamCB, NULL);

  buf[0] = 0x30;
  buf[1] = 0x08;
  buf[2] = 0x00;
  buf[3] = 0x00;
  buf[4] = 0x01;
  buf[5] = 0x30;
  buf[6] = 0x03;
  buf[7] = 0x00;
  while(1) {
    sleep(2);
    Mobot_sendRawStream(&mobot, (void*)buf, 9);
  }

  return 0;
}
