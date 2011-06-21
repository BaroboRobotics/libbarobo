#include "GtkiMobotController.h"

void on_button_connect_clicked(GtkWidget* widget, gpointer data)
{
  /* get the text entry texts */
  GtkEntry* address_entry;
  GtkEntry* channel_entry;
  const char* address;
  const char* channel;
  int c, err, i;

  address_entry = GTK_ENTRY( gtk_builder_get_object(builder, "entry_connect_address"));
  channel_entry = GTK_ENTRY( gtk_builder_get_object(builder, "entry_connect_channel"));
  
  address = gtk_entry_get_text(address_entry); 
  channel = gtk_entry_get_text(channel_entry); 
  sscanf(channel, "%d", &c);
  err = BRComms_connect(imobotComms, address, c);
  if(err == 0) {
    /* Change the status bar */
    /* Set the status bar */
    g_isConnected = 1;
    GtkStatusbar* status = GTK_STATUSBAR(gtk_builder_get_object(builder, "statusbar1"));
    int contextid;
    contextid = gtk_statusbar_get_context_id(status, "connection");
    gtk_statusbar_push(status, contextid, "Connected.");
    gtk_widget_hide(dialog_connect);

    /* Get the motor speeds and set the sliders */
    int speed;
    for(i = 0; i < 4; i++) {
      BRComms_getMotorSpeed(imobotComms, i, &speed);
      gtk_range_set_value(GTK_RANGE(scale_motorSpeeds[i]), speed);
    }
  }
}

void on_button_connect_cancel_clicked(GtkWidget* widget, gpointer data)
{
  gtk_widget_hide(dialog_connect);
}
