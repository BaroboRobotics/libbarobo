#ifndef _GTKIMOBOTCONTROLLER_H_
#define _GTKIMOBOTCONTROLLER_H_

#include <gtk/gtk.h>
#include "gait.h"
#define MOTION_POSE 0
#define MOTION_MOVE 1

/* These function prototypes need to be in "C" so that gmodule can find them */
extern "C" {

int initialize();

/* Main Dialog */
void on_menuitem_connect_activate(GtkWidget* widget, gpointer data);

/* Connect Dialog */
void on_button_connect_clicked(GtkWidget* widget, gpointer data);
void on_button_connect_cancel_clicked(GtkWidget* widget, gpointer data);

/* Misc. */
void quick_message (GtkWidget* parent, gchar *message);
int getIterModelFromTreeSelection(GtkTreeView *treeView, GtkTreeModel **model, GtkTreeIter *iter);
}

extern GtkBuilder *builder;
extern br_comms_t *imobotComms;
extern GtkWidget  *window;
extern GtkWidget  *dialog_connect;
extern Gait* g_gaits[100];
extern int g_numGaits;

#endif
