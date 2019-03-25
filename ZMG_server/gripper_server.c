

//  Pubsub envelope publisher
//  Note that the zhelpers.h file also provides s_sendmore

#include "zhelpers.h"
#include <unistd.h>
#include <gtk/gtk.h>


typedef struct input_bundle {
   void* publisher;
   GtkWidget* toggle;
   int link1_angle;
   int link2_angle;
   char controller_select;
}input_bundle;


gint delete_event_handler(GtkWidget* widget, GdkEvent* event, gpointer data){
 g_print("delete event occured\n");
 return FALSE;
}

void destroy(GtkWidget* widget, gpointer data){
 gtk_main_quit();
}

void zmq_update(gpointer input_ptr){
  input_bundle* casted_iptr = (input_bundle*)input_ptr;

  //From guide: http://zguide.zeromq.org/c:wuserver
  char message [16];
  double first;
  double second;
  sprintf (message, "%c %d %d",(char) 0b00000010, casted_iptr->link1_angle, casted_iptr->link2_angle);
  //g_printf("%s",message);
  s_sendmore (casted_iptr->publisher, "B");
  s_send (casted_iptr->publisher, message);
  //g_printf("Link 1: %d | Link 2: %d \n" ,casted_iptr->link1_angle, casted_iptr->link2_angle);
}

void value_changed1(GtkRange *range, gpointer* input_ptr) {
  input_bundle* casted_iptr = (input_bundle*)input_ptr;
  casted_iptr->link1_angle = (int)gtk_range_get_value(range);

  if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(casted_iptr->toggle))){
    g_printf("Hello world 1\n");
    zmq_update(input_ptr);
  }
}

void value_changed2(GtkRange *range, gpointer* input_ptr){
    input_bundle* casted_iptr = (input_bundle*)input_ptr;
    casted_iptr->link2_angle = (int)gtk_range_get_value(range);

  if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(casted_iptr->toggle))){
    g_printf("Hello world 2\n");
    zmq_update(input_ptr);
  }
}

void transmission_toggle(GtkWidget *widget, gpointer* input_ptr) {
  if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget))) {
    g_printf("toggle on\n");
    zmq_update(input_ptr);
  } else {
    g_printf("toggle off\n");
  }
}

void tab1(GtkWidget *widget){
  g_printf("Tab1\n");
}
void tab2(GtkWidget *widget){
  g_printf("Tab2\n");
}
void tab3(GtkWidget *widget){
  g_printf("Tab3\n");
}
void tab4(GtkWidget *widget){
  g_printf("Tab4\n");
}


int main (int  argc, char *argv[])
{
    input_bundle *input = g_new0(input_bundle, 1);
    //input->publisher = g_new0(void*, 1)
    //input->random_nunmber =  g_new0(void*, 1)
    //
    //ZMQ: Prepare our context and publisher
    printf("init\n");
    void *context = zmq_ctx_new ();
    printf("init\n");
    input->publisher = zmq_socket (context, ZMQ_PUB);
    printf("init\n");
    zmq_bind (input->publisher, "tcp://*:5563");
    //zmq_bind (input->publisher, "tcp://enp1s0f1:5563");
    printf("init\n");
    //GTK: Widget pointers
    GtkWidget *window;
    GtkWidget *halign;
    GtkWidget *grid_tab1;
    GtkWidget *grid_tab2;
    GtkWidget *grid_tab3;
    GtkWidget *grid_tab4;
    GtkWidget *hscale1;
    GtkWidget *hscale2;
    GtkWidget *label1;
    GtkWidget *label2;
    GtkWidget *tablabel1;
    GtkWidget *tablabel2;
    GtkWidget *tablabel3;
    GtkWidget *tablabel4;
    GtkWidget *e_box1;
    GtkWidget *e_box2;
    GtkWidget *e_box3;
    GtkWidget *e_box4;
    GtkWidget *notebook;

    //input.toggle = check;

    gtk_init(&argc, &argv);
    int testlol = 1;
    //Creating widgets
    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_CENTER);
    gtk_window_set_default_size(GTK_WINDOW(window), 300, 250);
    gtk_container_set_border_width(GTK_CONTAINER(window), 10);
    gtk_window_set_title(GTK_WINDOW(window), "Gripper");

    notebook = gtk_notebook_new();
    //gtk_notebook_set_tab_pos(notebook, GTK_POS_TOP);

    hscale1 = gtk_hscale_new_with_range(0, 100, 1);
    gtk_scale_set_draw_value(GTK_SCALE(hscale1), TRUE);
    gtk_widget_set_size_request(hscale1, 150, -1);

    hscale2 = gtk_hscale_new_with_range(0, 100, 1);
    gtk_scale_set_draw_value(GTK_SCALE(hscale2), TRUE);
    gtk_widget_set_size_request(hscale2, 150, -1);

    input->toggle = gtk_check_button_new_with_label("Send to gripper");
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(input->toggle), FALSE);
    GTK_WIDGET_UNSET_FLAGS(input->toggle, GTK_CAN_FOCUS);

    label1 = gtk_label_new("Link 1 angle: ");
    label2 = gtk_label_new("Link 2 angle: ");
    tablabel1 = gtk_label_new("Joint space PID");
    tablabel2 = gtk_label_new("Cartesian PID");
    tablabel3 = gtk_label_new("Hybrid position/force control [Cartesian]");
    tablabel4 = gtk_label_new("Hybrid position/force control [Polar]");

    grid_tab1 = gtk_table_new(5,4,FALSE);
    gtk_table_set_row_spacings(GTK_TABLE(grid_tab1), 2);
    gtk_table_set_col_spacings(GTK_TABLE(grid_tab1), 2);

    grid_tab2 = gtk_table_new(5,4,FALSE);
    gtk_table_set_row_spacings(GTK_TABLE(grid_tab2), 2);
    gtk_table_set_col_spacings(GTK_TABLE(grid_tab2), 2);

    grid_tab3 = gtk_table_new(5,4,FALSE);
    gtk_table_set_row_spacings(GTK_TABLE(grid_tab3), 2);
    gtk_table_set_col_spacings(GTK_TABLE(grid_tab3), 2);

    grid_tab4 = gtk_table_new(5,4,FALSE);
    gtk_table_set_row_spacings(GTK_TABLE(grid_tab4), 2);
    gtk_table_set_col_spacings(GTK_TABLE(grid_tab4), 2);

    e_box1 = gtk_event_box_new ();
    e_box2 = gtk_event_box_new ();
    e_box3 = gtk_event_box_new ();
    e_box4 = gtk_event_box_new ();

    //Insert widgets to 4x4 table
    gtk_table_attach_defaults(GTK_TABLE(grid_tab1), label1, 0, 1, 0, 1);
    gtk_table_attach_defaults(GTK_TABLE(grid_tab1), hscale1, 1, 2, 0, 1);
    gtk_table_attach_defaults(GTK_TABLE(grid_tab1), label2, 0, 1, 1, 2);
    gtk_table_attach_defaults(GTK_TABLE(grid_tab1), hscale2, 1, 2, 1, 2);
    gtk_table_attach_defaults(GTK_TABLE(grid_tab1), input->toggle, 0, 1, 2, 3);

    //INSERT TABS TO NOTEBOOK (With functionality)
    gtk_notebook_append_page( notebook,  grid_tab1, e_box1);
    gtk_container_add (GTK_CONTAINER (e_box1), tablabel1);
    gtk_widget_show(e_box1);
    gtk_widget_show (tablabel1);
    gtk_widget_set_events (e_box1, GDK_BUTTON_PRESS_MASK);
    gtk_signal_connect (GTK_OBJECT(e_box1), "button_press_event", GTK_SIGNAL_FUNC (tab1), NULL);

    gtk_notebook_append_page( notebook,  grid_tab2, e_box2);
    gtk_container_add (GTK_CONTAINER (e_box2), tablabel2);
    gtk_widget_show(e_box2);
    gtk_widget_show (tablabel2);
    gtk_widget_set_events (e_box2, GDK_BUTTON_PRESS_MASK);
    gtk_signal_connect (GTK_OBJECT(e_box2), "button_press_event", GTK_SIGNAL_FUNC (tab2), NULL);


    gtk_notebook_append_page( notebook,  grid_tab3, e_box3);
    gtk_container_add (GTK_CONTAINER (e_box4), tablabel3);
    gtk_widget_show(e_box3);
    gtk_widget_show (tablabel3);
    gtk_widget_set_events (e_box3, GDK_BUTTON_PRESS_MASK);
    gtk_signal_connect (GTK_OBJECT(e_box3), "button_press_event", GTK_SIGNAL_FUNC (tab4), NULL);


    gtk_notebook_append_page( notebook,  grid_tab4, e_box4);
    gtk_container_add (GTK_CONTAINER (e_box4), tablabel4);
    gtk_widget_show(e_box4);
    gtk_widget_show (tablabel4);
    gtk_widget_set_events (e_box4, GDK_BUTTON_PRESS_MASK);
    gtk_signal_connect (GTK_OBJECT(e_box4), "button_press_event", GTK_SIGNAL_FUNC (tab4), NULL);

    //INSERT NOTEBOOK INTO WINDOW
    gtk_container_add(GTK_CONTAINER(window), notebook);

    //Connect widgets to functionalities
    g_signal_connect(window, "destroy",
        G_CALLBACK(gtk_main_quit), NULL);

    g_signal_connect(hscale1, "value-changed",
        G_CALLBACK(value_changed1), (gpointer)input);

    g_signal_connect(hscale2, "value-changed",
        G_CALLBACK(value_changed2), (gpointer)input);

    g_signal_connect(input->toggle, "clicked",
        G_CALLBACK(transmission_toggle), (gpointer)input);

  //  g_signal_connect(tablabel4, "clicked", G_CALLBACK(tab4), NULL);

    gtk_widget_show_all(window);

    gtk_main();




    //while (1) {
    //    //  Write two messages, each with an envelope and content
    //    printf("while1\n");
    //    s_sendmore (publisher, "A");
    //    printf("while2\n");
    //    s_send (publisher, "We don't want to see this");
    //    printf("while3\n");
    //    s_sendmore (publisher, "B");
    //    printf("while4\n");
    //    s_send (publisher, "We would like to see this");
    //    printf("while5\n");
    //    sleep (1);
    //    printf("while6\n");
    //}
    //  We never get here, but clean up anyhow
    //mq_close (input->publisher);
    //zmq_ctx_destroy (context);
    //return 0;
}
