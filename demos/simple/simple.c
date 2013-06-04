#include <mobot.h>
#include <stdio.h>
#include <stdlib.h>

#define NUMBOTS 1

int main()
{
  int i;
  mobot_t mobot;
  Mobot_init(&mobot);

  for(i = 0; i < 10; i++) {
	  printf("Connect...\n");
	  if(!Mobot_connectWithTTY(&mobot, "COM13")) 
	  {
		  Mobot_setBuzzerFrequencyOn(&mobot, 440);
#ifdef _WIN32
		  Sleep(1000);
#else
      sleep(1);
#endif
		  Mobot_setBuzzerFrequencyOff(&mobot);
	  }
	  printf("Disconnect...\n");
	  Mobot_disconnect(&mobot);
	  printf("Connect...\n");
	  Mobot_connectWithTTY(&mobot, "COM8");
	  printf("Disconnect...\n");
	  Mobot_disconnect(&mobot);
  }
  return 0;
}
