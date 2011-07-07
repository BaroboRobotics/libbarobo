#include <stdlib.h>
#include "GtkiMobotController.h"
void on_menuitem_connect_activate(GtkWidget* widget, gpointer data)
{
  gtk_widget_show(GTK_WIDGET(dialog_connect));
}

void on_menuitem_localInit_activate(GtkWidget* widget, gpointer data)
{
  int i;
  g_localInit = 1;
  iMobot = (iMobot_t*)malloc(sizeof(iMobot_t));
  BR_init(iMobot);
  for(i = 0; i < 4; i++) {
    gtk_range_set_value(GTK_RANGE(scale_motorSpeeds[i]), 30);
  }
  /* Set the status bar */
  GtkStatusbar* status = GTK_STATUSBAR(gtk_builder_get_object(builder, "statusbar1"));
  int contextid;
  contextid = gtk_statusbar_get_context_id(status, "connection");
  gtk_statusbar_push(status, contextid, "Connected.");
}
