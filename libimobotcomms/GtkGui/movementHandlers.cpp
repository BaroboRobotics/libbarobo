#include "GtkiMobotController.h"

void on_button_forward_clicked(GtkWidget* widget, gpointer data)
{
  BRComms_setMotorDirection(imobotComms, 2, 2);
  BRComms_setMotorDirection(imobotComms, 3, 1);
  int speeds[2];
  speeds[0] = gtk_range_get_value(GTK_RANGE(scale_motorSpeeds[2]));
  speeds[1] = gtk_range_get_value(GTK_RANGE(scale_motorSpeeds[3]));
  BRComms_setMotorSpeed(imobotComms, 2, speeds[0]);
  BRComms_setMotorSpeed(imobotComms, 3, speeds[1]);
}

void on_button_backward_clicked(GtkWidget* widget, gpointer data)
{
  BRComms_setMotorDirection(imobotComms, 2, 1);
  BRComms_setMotorDirection(imobotComms, 3, 2);
  int speeds[2];
  speeds[0] = gtk_range_get_value(GTK_RANGE(scale_motorSpeeds[2]));
  speeds[1] = gtk_range_get_value(GTK_RANGE(scale_motorSpeeds[3]));
  BRComms_setMotorSpeed(imobotComms, 2, speeds[0]);
  BRComms_setMotorSpeed(imobotComms, 3, speeds[1]);
}

void on_button_rotateLeft_clicked(GtkWidget* widget, gpointer data)
{
  BRComms_setMotorDirection(imobotComms, 2, 2);
  BRComms_setMotorDirection(imobotComms, 3, 2);
  int speeds[2];
  speeds[0] = gtk_range_get_value(GTK_RANGE(scale_motorSpeeds[2]));
  speeds[1] = gtk_range_get_value(GTK_RANGE(scale_motorSpeeds[3]));
  BRComms_setMotorSpeed(imobotComms, 2, speeds[0]);
  BRComms_setMotorSpeed(imobotComms, 3, speeds[1]);
}

void on_button_rotateRight_clicked(GtkWidget* widget, gpointer data)
{
  BRComms_setMotorDirection(imobotComms, 2, 1);
  BRComms_setMotorDirection(imobotComms, 3, 1);
  int speeds[2];
  speeds[0] = gtk_range_get_value(GTK_RANGE(scale_motorSpeeds[2]));
  speeds[1] = gtk_range_get_value(GTK_RANGE(scale_motorSpeeds[3]));
  BRComms_setMotorSpeed(imobotComms, 2, speeds[0]);
  BRComms_setMotorSpeed(imobotComms, 3, speeds[1]);
}

void on_button_lfaceForward_clicked(GtkWidget* widget, gpointer data)
{
  BRComms_setMotorDirection(imobotComms, 3, 1);
  int speed;
  speed = gtk_range_get_value(GTK_RANGE(scale_motorSpeeds[3]));
  BRComms_setMotorSpeed(imobotComms, 3, speed);
}

void on_button_lfaceBackward_clicked(GtkWidget* widget, gpointer data)
{
  BRComms_setMotorDirection(imobotComms, 3, 2);
  int speed;
  speed = gtk_range_get_value(GTK_RANGE(scale_motorSpeeds[3]));
  BRComms_setMotorSpeed(imobotComms, 3, speed);
}

void on_button_rfaceForward_clicked(GtkWidget* widget, gpointer data)
{
  BRComms_setMotorDirection(imobotComms, 2, 2);
  int speed;
  speed = gtk_range_get_value(GTK_RANGE(scale_motorSpeeds[2]));
  BRComms_setMotorSpeed(imobotComms, 2, speed);
}

void on_button_rfaceBackward_clicked(GtkWidget* widget, gpointer data)
{
  BRComms_setMotorDirection(imobotComms, 2, 1);
  int speed;
  speed = gtk_range_get_value(GTK_RANGE(scale_motorSpeeds[2]));
  BRComms_setMotorSpeed(imobotComms, 2, speed);
}

void on_button_inchLeft_clicked(GtkWidget* widget, gpointer data)
{
  executeGait(findGait("Inch Left"));
}

void on_button_inchRight_clicked(GtkWidget* widget, gpointer data)
{
  executeGait(findGait("Inch Right"));
}

void on_button_stop_clicked(GtkWidget* widget, gpointer data)
{
  BRComms_stop(imobotComms);
  int i;

  for(i = 0; i < 4; i++) {
    /* Make all motor direction modes back to auto */
    BRComms_setMotorDirection(imobotComms, i, 0);
  }
}

void on_button_home_clicked(GtkWidget* widget, gpointer data) 
{
  int i;
  GtkRange* range;
  int speed;
  for(i = 0; i < 4; i++) {
    range = GTK_RANGE(scale_motorSpeeds[i]);
    speed = gtk_range_get_value(range);
    if(speed < 30) {
      speed = 30;
    }
    BRComms_setMotorSpeed(imobotComms, i, speed);
    BRComms_setMotorDirection(imobotComms, i, 0);
    BRComms_setMotorPosition(imobotComms, i, 0);
  }
}

gboolean on_vscale_motorspeed0_button_release_event(GtkRange* range, GdkEvent* event, gpointer data)
{
  int speed = (int)gtk_range_get_value(range);
  BRComms_setMotorSpeed(imobotComms, 0, speed);
  return FALSE;
}

gboolean on_vscale_motorspeed1_button_release_event(GtkRange* range, GdkEvent* event, gpointer data)
{
  int speed = (int)gtk_range_get_value(range);
  BRComms_setMotorSpeed(imobotComms, 1, speed);
  return FALSE;
}

gboolean on_vscale_motorspeed2_button_release_event(GtkRange* range, GdkEvent* event, gpointer data)
{
  int speed = (int)gtk_range_get_value(range);
  BRComms_setMotorSpeed(imobotComms, 2, speed);
  return FALSE;
}

gboolean on_vscale_motorspeed3_button_release_event(GtkRange* range, GdkEvent* event, gpointer data)
{
  int speed = (int)gtk_range_get_value(range);
  BRComms_setMotorSpeed(imobotComms, 3, speed);
  return FALSE;
}

