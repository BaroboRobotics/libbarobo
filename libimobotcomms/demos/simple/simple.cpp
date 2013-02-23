/* Filename: simple.cpp
 * Rotate the faceplates by 90 degrees */
#include <mobot.h>
#include <stdlib.h>

int main()
{
  CMobot mobot;
  mobot.connect();
  mobot.move(360, 0, 360, 0);

  return 0;
}
