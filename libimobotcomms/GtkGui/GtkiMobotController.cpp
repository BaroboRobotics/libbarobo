#include <stdlib.h>
#include <gtk/gtk.h>
#include "../imobotcomms.h"
#include "gait.h"
#include "GtkiMobotController.h"

/* Keep the builder global so we can find any widget from anywhere */
GtkBuilder *builder;
br_comms_t *imobotComms;
GtkWidget  *window;
GtkWidget  *dialog_connect;
GtkVScale* scale_motorSpeeds[4];

Gait* g_gaits[100];
int g_numGaits;

int main(int argc, char* argv[])
{
  GError     *error = NULL;
  g_numGaits = 0;

  gtk_init(&argc, &argv);

  /* Create the GtkBuilder */
  builder = gtk_builder_new();

  /* Load the UI from the file */
  if( ! gtk_builder_add_from_file(builder, "interface/interface.glade", &error) )
  {
    g_warning("%s", error->message);
    g_free(error);
    return -1;
  }

  /* Initialize everything */
  initialize();

  /* Get the main window */
  window = GTK_WIDGET( gtk_builder_get_object( builder, "window1"));
  dialog_connect = GTK_WIDGET( gtk_builder_get_object(builder, "dialog_connect"));

  /* Connect signals */
  gtk_builder_connect_signals(builder, NULL);

  gtk_widget_show( window );
  
  gtk_main();

  return 0;
}

void
quick_message (GtkWidget* parent, gchar *message)
{
  GtkWidget *dialog, *label, *content_area;

  /* Create the widgets */
  dialog = gtk_dialog_new_with_buttons ("Message",
      GTK_WINDOW(parent),
      GTK_DIALOG_DESTROY_WITH_PARENT,
      GTK_STOCK_OK,
      GTK_RESPONSE_NONE,
      NULL);
  content_area = gtk_dialog_get_content_area (GTK_DIALOG (dialog));
  label = gtk_label_new (message);

  /* Ensure that the dialog box is destroyed when the user responds */
  g_signal_connect_swapped (dialog,
      "response",
      G_CALLBACK (gtk_widget_destroy),
      dialog);

  /* Add the label, and show everything we've added to the dialog */

  gtk_container_add (GTK_CONTAINER (content_area), label);
  gtk_widget_show_all (dialog);
}

int getIterModelFromTreeSelection(GtkTreeView *treeView, GtkTreeModel **model, GtkTreeIter *iter)
{
  GtkTreeSelection *treeSelection;

  treeSelection = gtk_tree_view_get_selection( treeView );
  gtk_tree_selection_set_mode(treeSelection, GTK_SELECTION_BROWSE);

  bool success = 
    gtk_tree_selection_get_selected( treeSelection, model, iter );
  if(!success) {
    return -1;
  }
  return 0;
}

int initialize()
{
  int i;
  int speed; 

  imobotComms = (br_comms_t*)malloc(sizeof(br_comms_t));
  BRComms_init(imobotComms);

  /* Set up the motor speed VScales */
  scale_motorSpeeds[0] = GTK_VSCALE(gtk_builder_get_object( builder, "vscale_motorspeed0"));
  scale_motorSpeeds[1] = GTK_VSCALE(gtk_builder_get_object( builder, "vscale_motorspeed1"));
  scale_motorSpeeds[2] = GTK_VSCALE(gtk_builder_get_object( builder, "vscale_motorspeed2"));
  scale_motorSpeeds[3] = GTK_VSCALE(gtk_builder_get_object( builder, "vscale_motorspeed3"));
  for(i = 0; i < 4; i++) {
    gtk_range_set_range(GTK_RANGE(scale_motorSpeeds[i]), 0, 100);
  }

  /* Set the status bar */
  GtkStatusbar* status = GTK_STATUSBAR(gtk_builder_get_object(builder, "statusbar1"));
  int contextid;
  contextid = gtk_statusbar_get_context_id(status, "connection");
  gtk_statusbar_push(status, contextid, "Not connected.");
}
