#include <stdlib.h>
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

void on_button_playGait_clicked(GtkWidget* widget, gpointer data) 
{
  /* Need to get the selected gait */
  GtkTreeView *treeView;
  GtkTreeSelection *treeSelection;
  GtkTreeModel *model;
  GtkTreeIter iter;
  char* name;

  treeView = GTK_TREE_VIEW(
      gtk_builder_get_object(builder, "treeview_gaits"));
  model = GTK_TREE_MODEL(
      gtk_builder_get_object(builder, "liststore_gaits"));

  treeSelection = gtk_tree_view_get_selection( treeView );
  gtk_tree_selection_set_mode(treeSelection, GTK_SELECTION_BROWSE);
  bool success = 
    gtk_tree_selection_get_selected( treeSelection, &model, &iter );
  if(!success) {
    return;
  }
  gtk_tree_model_get(model, &iter,
      0, &name,
      -1);
  executeGait(findGait(name));
  free(name);
}

void motor_forward(int id)
{
  BRComms_setMotorDirection(imobotComms, id, 1);
  int speed;
  speed = gtk_range_get_value(GTK_RANGE(scale_motorSpeeds[id]));
  BRComms_setMotorSpeed(imobotComms, id, speed);
}

void motor_stop(int id)
{
  BRComms_setMotorSpeed(imobotComms, id, 0);
  BRComms_setMotorDirection(imobotComms, id, 0);
}

void motor_back(int id)
{
  BRComms_setMotorDirection(imobotComms, id, 2);
  int speed;
  speed = gtk_range_get_value(GTK_RANGE(scale_motorSpeeds[id]));
  BRComms_setMotorSpeed(imobotComms, id, speed);
}

#define ON_MOTOR_BUTTON_CLICKED(id, motion) \
void on_button_motor##id##motion##_clicked(GtkWidget* widget, gpointer data) \
{ \
  motor_##motion (id); \
}
ON_MOTOR_BUTTON_CLICKED(0, forward)
ON_MOTOR_BUTTON_CLICKED(1, forward)
ON_MOTOR_BUTTON_CLICKED(2, forward)
ON_MOTOR_BUTTON_CLICKED(3, forward)
ON_MOTOR_BUTTON_CLICKED(0, stop)
ON_MOTOR_BUTTON_CLICKED(1, stop)
ON_MOTOR_BUTTON_CLICKED(2, stop)
ON_MOTOR_BUTTON_CLICKED(3, stop)
ON_MOTOR_BUTTON_CLICKED(0, back)
ON_MOTOR_BUTTON_CLICKED(1, back)
ON_MOTOR_BUTTON_CLICKED(2, back)
ON_MOTOR_BUTTON_CLICKED(3, back)
#undef ON_MOTOR_BUTTON_CLICKED


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

