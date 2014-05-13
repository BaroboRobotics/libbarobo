/* Filename: simple.cpp
 * Rotate the faceplates by 90 degrees */
#include <mobot.h>
#include <linkbot.h>
#include <stdlib.h>
#include <iostream>

using namespace std;
int main()
{
  char color[20],color2[20]="red";
  int r, g, b;
  int pin=2;
  CLinkbotL robot;
  
  robot.connect();
  robot.getLEDColorName(color);
  cout<<color<<endl;
  robot.setLEDColor(color2);
  robot.getLEDColorName(color);
  cout<<color<<endl;
  robot.setLEDColorRGB(100,20,200);
  robot.getLEDColorRGB(r,g,b);
  cout<<r<<" "<<g<<" "<<b<<endl;
  
  robot.LinkPodPinMode(pin, 1);
  robot.LinkPodDigitalWrite(pin, 0x1);
  //robot.LinkPodDigitalWrite(pin, 0x0);

  return 0;
}
