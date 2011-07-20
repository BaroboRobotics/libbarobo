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

void on_button_connectRemote_clicked(GtkWidget* widget, gpointer data)
{
  gtk_widget_hide(dialog_intro);
  gtk_widget_show(window);
  gtk_widget_show(dialog_connect);
}

void on_button_connectLocal_clicked(GtkWidget* widget, gpointer data)
{
  on_menuitem_localInit_activate(widget, data);
  gtk_widget_hide(dialog_intro);
  gtk_widget_show(window);
}

void on_checkmenuitem_motorPos_toggled(GtkWidget* widget, gpointer data)
{
  GtkWidget* motorPosSliderWindow = GTK_WIDGET(gtk_builder_get_object(builder, "window_motorPos"));
  if(gtk_check_menu_item_get_active(GTK_CHECK_MENU_ITEM(widget)))
  {
    gtk_widget_show(motorPosSliderWindow);
  } else {
    gtk_widget_hide(motorPosSliderWindow);
  }
}
