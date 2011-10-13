#include <stdlib.h>
#include <gtk/gtk.h>
#include <imobotcomms.h>
#include <imobot.h>
#include "gait.h"
#include "GtkiMobotController.h"

/* These store the embedded glade xml file */                                                             
extern const char _binary_interface_interface_glade_start[];
extern size_t _binary_interface_interface_glade_size;
extern const char _binary_interface_interface_glade_end[];

/* Keep the builder global so we can find any widget from anywhere */
GtkBuilder *builder;
br_comms_t *imobotComms;
iMobot_t *iMobot;
GtkWidget *window;
GtkWidget *dialog_connect;
GtkWidget *dialog_intro;
GtkVScale* scale_motorSpeeds[4];
GtkVScale* scale_motorPositions[4];
int motor_position_scale_pressed[4];

GtkEntry* motorAngleEntries[4];

Gait* g_gaits[100];
int g_numGaits;
int g_isConnected;
int g_localInit;

int main(int argc, char* argv[])
{
  GError     *error = NULL;

  gtk_init(&argc, &argv);

  /* Create the GtkBuilder */
  builder = gtk_builder_new();

  /* Load the UI from the file */
  if( ! gtk_builder_add_from_string(builder, _binary_interface_interface_glade_start, strlen(_binary_interface_interface_glade_start), &error) )
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
  dialog_intro = GTK_WIDGET( gtk_builder_get_object(builder, "dialog_intro"));

  /* Connect signals */
  gtk_builder_connect_signals(builder, NULL);

  //gtk_widget_show( window );
  gtk_widget_show( dialog_intro );
  
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

  /* Initialize the gaits */
  init_gaits();
  imobotComms = (br_comms_t*)malloc(sizeof(br_comms_t));
  iMobotComms_init(imobotComms);
  g_isConnected = 0;
  g_localInit = 0;

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

  /* Get the motor angle entries */
  motorAngleEntries[0] = GTK_ENTRY(gtk_builder_get_object(builder, "entry_motor0angle"));
  motorAngleEntries[1] = GTK_ENTRY(gtk_builder_get_object(builder, "entry_motor1angle"));
  motorAngleEntries[2] = GTK_ENTRY(gtk_builder_get_object(builder, "entry_motor2angle"));
  motorAngleEntries[3] = GTK_ENTRY(gtk_builder_get_object(builder, "entry_motor3angle"));

  /* Set the ranges on the motor position adjustment vscales */
  scale_motorPositions[0] = GTK_VSCALE(gtk_builder_get_object( builder, "vscale_motorPos0"));
  scale_motorPositions[1] = GTK_VSCALE(gtk_builder_get_object( builder, "vscale_motorPos1"));
  scale_motorPositions[2] = GTK_VSCALE(gtk_builder_get_object( builder, "vscale_motorPos2"));
  scale_motorPositions[3] = GTK_VSCALE(gtk_builder_get_object( builder, "vscale_motorPos3"));
  gtk_range_set_range(GTK_RANGE(scale_motorPositions[0]), -180, 180);
  gtk_range_set_range(GTK_RANGE(scale_motorPositions[1]), -90, 90);
  gtk_range_set_range(GTK_RANGE(scale_motorPositions[2]), -90, 90);
  gtk_range_set_range(GTK_RANGE(scale_motorPositions[3]), -180, 180);

  /* Set up the motor angle entries handler */
  g_timeout_add(250, updateMotorAngles, NULL);
}

#define SET_ANGLES(angles, a, b, c, d) \
	angles[0] = a; \
	angles[1] = b; \
	angles[2] = c; \
	angles[3] = d

int init_gaits()
{
  g_numGaits = 0;
  Gait* gait;
  double angles[4];
  unsigned char motorMask;
	/* Arch */
	gait = new Gait("Arch");
	SET_ANGLES(angles, 0, -30, 30, 0);
	motorMask = 0;
	motorMask |= (1<<0); motorMask |= (1<<1);
	gait->addMotion( new Motion(
		(motion_type_t)MOTION_POSE, angles, motorMask));
	addGait(gait);

	/* Inch Right */
	gait = new Gait("Inch Right");
	SET_ANGLES(angles, 0, -50, 0, 0);
	motorMask = 0;
	motorMask |= (1<<0); motorMask |= (1<<1);
	gait->addMotion(new Motion(
		(motion_type_t)MOTION_POSE, angles, motorMask));
	SET_ANGLES(angles, 0, -50, 50, 0);
	gait->addMotion(new Motion(
		(motion_type_t)MOTION_POSE, angles, motorMask));
	SET_ANGLES(angles, 0, 0, 50, 0);
	gait->addMotion(new Motion(
		(motion_type_t)MOTION_POSE, angles, motorMask));
	SET_ANGLES(angles, 0, 0, 0, 0);
	gait->addMotion(new Motion(
		(motion_type_t)MOTION_POSE, angles, motorMask));
	addGait(gait);

	/* Inch Left */
	gait = new Gait("Inch Left");
	SET_ANGLES(angles, 0, 0, 50, 0);
	motorMask = 0;
	motorMask |= (1<<0); motorMask |= (1<<1);
	gait->addMotion(new Motion(
		(motion_type_t)MOTION_POSE, angles, motorMask));
	SET_ANGLES(angles, 0, -50, 50, 0);
	gait->addMotion(new Motion(
		(motion_type_t)MOTION_POSE, angles, motorMask));
	SET_ANGLES(angles, 0, -50, 0, 0);
	gait->addMotion(new Motion(
		(motion_type_t)MOTION_POSE, angles, motorMask));
	SET_ANGLES(angles, 0, 0, 0, 0);
	gait->addMotion(new Motion(
		(motion_type_t)MOTION_POSE, angles, motorMask));
	addGait(gait);

	/* Stand */
	gait = new Gait("Stand");
	SET_ANGLES(angles, 0, 0, 0, 0);
	motorMask = 0;
	motorMask |= (1<<0); motorMask |= (1<<1);
	gait->addMotion(new Motion(
		(motion_type_t)MOTION_POSE, angles, motorMask));
	SET_ANGLES(angles, 0, -85, 80, 0);
	gait->addMotion(new Motion(
		(motion_type_t)MOTION_POSE, angles, motorMask));
	SET_ANGLES(angles, 45, 0, 0, 0);
	motorMask = (1<<2);
	gait->addMotion(new Motion(
		(motion_type_t)MOTION_POSE, angles, motorMask));
	SET_ANGLES(angles, 0, 20, 0, 0);
	motorMask = (1<<0);
	gait->addMotion(new Motion(
		(motion_type_t)MOTION_POSE, angles, motorMask));
	addGait(gait);

	/* Stand turn Right */
	gait = new Gait("Stand Turn Right");
	SET_ANGLES(angles, -30, 0, 0, 0);
	motorMask = 1<<2;
	gait->addMotion(new Motion(
		(motion_type_t)MOTION_MOVE, angles, motorMask));
	addGait(gait);

	/* Stand turn left */
	gait = new Gait("Stand Turn Left");
	SET_ANGLES(angles, 30, 0, 0, 0);
	gait->addMotion(new Motion(
		(motion_type_t)MOTION_MOVE, angles, motorMask));
	addGait(gait);

	/* Right Faceplate Forward */
	gait = new Gait("Right Face Forward");
	SET_ANGLES(angles, -10, 0, 0, 0);
	motorMask = 1<<2;
	gait->addMotion(new Motion(
		(motion_type_t)MOTION_MOVE, angles, motorMask));
	addGait(gait);

	/* Right Faceplate Backward */
	gait = new Gait("Right Face Backward");
	SET_ANGLES(angles, 10, 0, 0, 0);
	motorMask = 1<<2;
	gait->addMotion(new Motion(
		(motion_type_t)MOTION_MOVE, angles, motorMask));
	addGait(gait);

	/* Left Faceplate Forward */
	gait = new Gait("Left Face Forward");
	SET_ANGLES(angles, 0, 0, 0, 10);
	motorMask = 1<<3;
	gait->addMotion(new Motion(
		(motion_type_t)MOTION_MOVE, angles, motorMask));
	addGait(gait);

	/* Left Faceplate Backward */
	gait = new Gait("Left Face Backward");
	SET_ANGLES(angles, 0, 0, 0, -10);
	motorMask = 1<<3;
	gait->addMotion(new Motion(
		(motion_type_t)MOTION_MOVE, angles, motorMask));
	addGait(gait);

	/* Unstand */
	gait = new Gait("Unstand");
	SET_ANGLES(angles, 0, -85, 85, 0);
	motorMask = 1<<0; motorMask |= 1<<1;
	gait->addMotion(new Motion(
		(motion_type_t)MOTION_POSE, angles, motorMask));
	SET_ANGLES(angles, 0, 0, 0, 0);
	gait->addMotion(new Motion(
		(motion_type_t)MOTION_POSE, angles, motorMask));
	addGait(gait);

	/* Skinny */
	gait = new Gait("Skinny");
	SET_ANGLES(angles, 0, 85, 85, 0);
	motorMask = 1<<0; motorMask |= 1<<1;
	gait->addMotion(new Motion(
		(motion_type_t)MOTION_POSE, angles, motorMask));
	addGait(gait);

  /* Add them to the liststore */
  int i;
  GtkTreeIter iter;
  GtkListStore* gaits_liststore = 
      GTK_LIST_STORE(gtk_builder_get_object(builder, "liststore_gaits"));
  for(i = 0; i < g_numGaits; i++) {
    gtk_list_store_append(gaits_liststore, &iter);
    gtk_list_store_set(gaits_liststore, &iter, 
      0, g_gaits[i]->getName(),
      -1 );
  }

  return 0;
}
#undef SET_ANGLES

int addGait(Gait* gait)
{
	g_gaits[g_numGaits] = gait;
	g_numGaits++;
	return 0;
}

Gait* findGait(const char* name)
{
  int i;
  for(i = 0; i < g_numGaits; i++) {
    if(!strcmp(name, g_gaits[i]->getName())) {
      return g_gaits[i];
    }
  }
  return NULL;
}

int executeGait(Gait* gait)
{
  int i,j;
  const Motion* motion;
  unsigned char motorMask;
  const double* angles;
  double pos;
  for(i = 0; i < gait->getNumMotions(); i++) {
    motion = gait->getMotion(i);
    angles = motion->getAngles();
    motorMask = motion->getMotorMask();

    switch(motion->getType()) {
      case MOTION_MOVE:
        for(j = 0; j < 4; j++) {
          if((1<<j) & motorMask) {
            getMotorPosition(j, &pos);
            pos += angles[j];
            setMotorDirection(j, 0);
            setMotorSpeed(j, 
                (int)gtk_range_get_value(GTK_RANGE(scale_motorSpeeds[j])));
            setMotorPosition(j, pos);
          }
        }
        break;
      case MOTION_POSE:
        for(j = 0; j<4; j++) {
          if((1<<j) & motorMask) {
            setMotorDirection(j, 0);
            setMotorSpeed(j, 
                (int)gtk_range_get_value(GTK_RANGE(scale_motorSpeeds[j])));
            setMotorPosition(j, angles[j]);
          }
        }
        break;
    }
    for(j = 0; j<4; j++) {
      if((1<<j) & motorMask) {
        waitMotor(j);
      }
    }
  }
  return 0;
}

gboolean updateMotorAngles(gpointer data)
{
  double position;
  int i;
  char buf[40];
  if(!iMobotComms_isConnected(imobotComms) && !g_localInit) {
    for(i = 0; i < 4; i++) {
      gtk_entry_set_text(motorAngleEntries[i], "N/A");
    }
    return TRUE;
  }
  for(i = 0; i < 4; i++) {
    getMotorPosition(i, &position);
    while(position > 180) position -= 360;
    while(position < -180) position += 360;
    sprintf(buf, "%.1f", position);
    gtk_entry_set_text(motorAngleEntries[i], buf);

    /* Update the motor position sliders */
    if(motor_position_scale_pressed[i]) {
      /* Send a set motor position to the appropriate motor */
      setMotorPosition(i, (double)gtk_range_get_value(GTK_RANGE(scale_motorPositions[i])));
    } else {
      while(position > 180) position -= 360;
      while(position < -180) position += 360;
      gtk_range_set_value(GTK_RANGE(scale_motorPositions[i]), position);
    }
  }
  return TRUE;
}
