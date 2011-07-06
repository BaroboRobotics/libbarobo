#include <stdlib.h>
#include "GtkiMobotController.h"
void on_menuitem_connect_activate(GtkWidget* widget, gpointer data)
{
  gtk_widget_show(GTK_WIDGET(dialog_connect));
}

void on_menuitem_localInit_activate(GtkWidget* widget, gpointer data)
{
  g_localInit = 1;
  iMobot = (iMobot_t*)malloc(sizeof(iMobot_t));
  BR_init(iMobot);
}
