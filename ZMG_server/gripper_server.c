

//  Pubsub envelope publisher
//  Note that the zhelpers.h file also provides s_sendmore

#include "zhelpers.h"
#include <unistd.h>
#include <gtk/gtk.h>


gint delete_event_handler(GtkWidget* widget, GdkEvent* event, gpointer data){
 g_print("delete event occured\n");
 return FALSE;
}

void destroy(GtkWidget* widget, gpointer data){
 gtk_main_quit();
}

void transmission_toggle(GtkWidget *widget, int *toggle) {
  if (toggle) {
    *toggle = 0;
  } else {
    *toggle = 1;
  }
}

void value_changed1(GtkRange *range, int *toggle) {
  if (*toggle){
    g_printf("Hello world 1\n");
  }
   //gdouble val = gtk_range_get_value(range);
  // gchar *str = g_strdup_printf("%.f", val);
   //gtk_label_set_text(GTK_LABEL(win), str);
  // g_free(str);
}

void value_changed2(GtkRange *range, int *toggle) {
  if (*toggle){
    g_printf("Hello world 2\n");
  }
   //gdouble val = gtk_range_get_value(range);
  // gchar *str = g_strdup_printf("%.f", val);
   //gtk_label_set_text(GTK_LABEL(win), str);
  // g_free(str);
}

int main (int  argc, char *argv[])
{
    //
    int send_toggle = 0;

    //Widget pointers
    GtkWidget *window;
    GtkWidget *halign;
    GtkWidget *grid;
    GtkWidget *hscale1;
    GtkWidget *hscale2;
    GtkWidget *label1;
    GtkWidget *label2;
    GtkWidget *check;

    gtk_init(&argc, &argv);

    //Creating widgets
    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_CENTER);
    gtk_window_set_default_size(GTK_WINDOW(window), 300, 250);
    gtk_container_set_border_width(GTK_CONTAINER(window), 10);
    gtk_window_set_title(GTK_WINDOW(window), "Gripper");

    hscale1 = gtk_hscale_new_with_range(0, 100, 1);
    gtk_scale_set_draw_value(GTK_SCALE(hscale1), TRUE);
    gtk_widget_set_size_request(hscale1, 150, -1);

    hscale2 = gtk_hscale_new_with_range(0, 100, 1);
    gtk_scale_set_draw_value(GTK_SCALE(hscale2), TRUE);
    gtk_widget_set_size_request(hscale2, 150, -1);

    check = gtk_check_button_new_with_label("Send to gripper");
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(check), FALSE);
    GTK_WIDGET_UNSET_FLAGS(check, GTK_CAN_FOCUS);

    label1 = gtk_label_new("Link 1 angle: ");
    label2 = gtk_label_new("Link 2 angle: ");

    grid = gtk_table_new(5,4,FALSE);
    gtk_table_set_row_spacings(GTK_TABLE(grid), 2);
    gtk_table_set_col_spacings(GTK_TABLE(grid), 2);

    //Insert widgets to 4x4 table
    gtk_table_attach_defaults(GTK_TABLE(grid), label1, 0, 1, 0, 1);
    gtk_table_attach_defaults(GTK_TABLE(grid), hscale1, 1, 2, 0, 1);
    gtk_table_attach_defaults(GTK_TABLE(grid), label2, 0, 1, 1, 2);
    gtk_table_attach_defaults(GTK_TABLE(grid), hscale2, 1, 2, 1, 2);
    gtk_table_attach_defaults(GTK_TABLE(grid), check, 0, 1, 2, 3);

    //Insert table into window
    gtk_container_add(GTK_CONTAINER(window), grid);

    //Connect widgets to functionalities
    g_signal_connect(window, "destroy",
        G_CALLBACK(gtk_main_quit), NULL);

    g_signal_connect(hscale1, "value-changed",
        G_CALLBACK(value_changed1), &send_toggle);

    g_signal_connect(hscale2, "value-changed",
        G_CALLBACK(value_changed2), &send_toggle);

    g_signal_connect(check, "clicked",
        G_CALLBACK(transmission_toggle), &send_toggle);

    gtk_widget_show_all(window);

    gtk_main();



    //  Prepare our context and publisher
    printf("init\n");
    void *context = zmq_ctx_new ();
    printf("init\n");
    void *publisher = zmq_socket (context, ZMQ_PUB);
    printf("init\n");
    zmq_bind (publisher, "tcp://enp1s0f1:5563");
    printf("init\n");

    while (1) {
        //  Write two messages, each with an envelope and content
        printf("while1\n");
        s_sendmore (publisher, "A");
        printf("while2\n");
        s_send (publisher, "We don't want to see this");
        printf("while3\n");
        s_sendmore (publisher, "B");
        printf("while4\n");
        s_send (publisher, "We would like to see this");
        printf("while5\n");
        sleep (1);
        printf("while6\n");
    }
    //  We never get here, but clean up anyhow
    zmq_close (publisher);
    zmq_ctx_destroy (context);
    return 0;
}
