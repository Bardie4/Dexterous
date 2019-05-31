

//  Pubsub envelope publisher
//  Note that the zhelpers.h file also provides s_sendmore

//#include "zhelpers.h"
#include <unistd.h>
#include <gtk/gtk.h>
#include <string>
#include <sstream>
#include <iostream>
#include <math.h>
#include "simple_instructions_generated.h"
#include <zmq.h>
#include "zmq.hpp"
// This was generated by `flatc`.
using namespace quad_double_mes; // Specified in the schema.

typedef struct zmq_bundle {
   void* context;
   void* publisher;
   short identity;
   short controller_select;
   double data1;
   double data2;
   double data3;
   double data4;
}zmq_bundle;

flatbuffers::FlatBufferBuilder builder(1024);

class finger{
  private:
    //FlatBuffers
  public:
    //FlatBuffers
    //ZMQ
    zmq_bundle zmq;
    //GTK
    int angle1_setpoint;
    int angle2_setpoint;
    int radius_setpoint;
    int angle_setpoint;
    bool init;
    char title[50];
    GtkWidget* upalign;
      GtkWidget* frame;
        GtkWidget* vbox_main;
          GtkWidget* finger_label;
          GtkWidget* hbox;
            GtkWidget* vbox_1;
              GtkWidget* init_button;
              GtkWidget* toggle_broadcast;
            GtkWidget* vbox_2;
              GtkWidget* scale_label_1;
              GtkWidget* scale_label_2;
              GtkWidget* scale_label_3;
              GtkWidget* scale_label_4;
            GtkWidget* vbox_3;
              GtkWidget *angle_link1_slider;
              GtkWidget *angle_link2_slider;
              GtkWidget *radius_slider;
              GtkWidget *angle_slider;
            GtkWidget* vbox_4;
              GtkWidget* radio_button1;
              GtkWidget* radio_button2;
              GtkWidget* entry;

  static void button_toggled_js (GtkToggleButton *button, gpointer data_input){
    finger* fing = (finger*) data_input;
            if (gtk_toggle_button_get_active(button) ){
                gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON(fing->toggle_broadcast), FALSE);
                fing->zmq.controller_select = 2;
                fing->zmq.data1 = fing->angle1_setpoint;
                fing->zmq.data2 = fing->angle2_setpoint;
                fing->zmq.data3 = 0;
                fing->zmq.data4 = 0;

                gtk_widget_show(fing->scale_label_1);
                gtk_widget_show(fing->scale_label_2);
                gtk_widget_show(fing->angle_link1_slider);
                gtk_widget_show(fing->angle_link2_slider);

                gtk_widget_hide(fing->scale_label_3);
                gtk_widget_hide(fing->scale_label_4);
                gtk_widget_hide(fing->radius_slider);
                gtk_widget_hide(fing->angle_slider);
            }
  }

  static void button_toggled_cs (GtkToggleButton *button, gpointer data_input){
    finger* fing = (finger*) data_input;
            if (gtk_toggle_button_get_active(button) ) {
                gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON(fing->toggle_broadcast), FALSE);
                fing->zmq.controller_select = 3;
                fing->zmq.data1 = cos(fing->angle_setpoint*(3.14/180))*fing->radius_setpoint;
                fing->zmq.data2 = sin(fing->angle_setpoint*(3.14/180))*fing->radius_setpoint;
                fing->zmq.data3 = 0;
                fing->zmq.data4 = 0;

                gtk_widget_show(fing->scale_label_3);
                gtk_widget_show(fing->scale_label_4);
                gtk_widget_show(fing->radius_slider);
                gtk_widget_show(fing->angle_slider);

                gtk_widget_hide(fing->scale_label_1);
                gtk_widget_hide(fing->scale_label_2);
                gtk_widget_hide(fing->angle_link1_slider);
                gtk_widget_hide(fing->angle_link2_slider);
              }
  }
  static void zmq_update(gpointer input_ptr){
    zmq_bundle* zmq = (zmq_bundle*)input_ptr;
    //from guide: https://stackoverflow.com/questions/40141120/simple-flatbuffers-over-zeromq-c-example-copy-struct-to-flatbuffer-over-zmq-an
    //From guide: http://zguide.zeromq.org/c:wuserver
    //my_message = Createsimple_instructions(builder, zmq->identity, zmq->controller_select, zmq->data1, zmq->data2, zmq->data3, zmq->data4)

    auto message_obj = CreateSimpleInstructionMsg(builder, zmq->identity, zmq->controller_select, zmq->data1, zmq->data2, zmq->data3, zmq->data4);
    FinishSimpleInstructionMsgBuffer(builder, message_obj);
    uint8_t *buf = builder.GetBufferPointer();
    int size = builder.GetSize();
    zmq_send (zmq->publisher, "B", 1, ZMQ_SNDMORE);
    zmq_send (zmq->publisher, buf, size, 0);
    std::cout << (int) zmq->identity <<" "<< (int) zmq->controller_select <<" "<< (int) zmq->data1<< " " << (int) zmq->data2 << " "<< (int) zmq->data3<< " "<< (int) zmq->data4 << std::endl;
    builder.Clear();
    std::cout <<"size: " << size << std::endl;
  }

  static void update_angle1(GtkRange *range, gpointer* input_ptr) {
    finger* fing = (finger*)input_ptr;
    fing->angle1_setpoint = (int)gtk_range_get_value(range);  //Update value
    fing->zmq.controller_select = 2;                 //Update and clear buffer
    fing->zmq.data1 = fing->angle1_setpoint * 3.14/180.0 ;
    fing->zmq.data2 = fing->angle2_setpoint * 3.14/180.0 ;
    fing->zmq.data3 = 0;
    fing->zmq.data4 = 0;

    //Send if broadcasting is enabled
    if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(fing->toggle_broadcast))){
      zmq_update(input_ptr);
    }
  }
  static void update_angle2(GtkRange *range, gpointer* input_ptr) {
    finger* fing = (finger*)input_ptr;

    fing->angle2_setpoint = (int) gtk_range_get_value(range);  //Update value
    fing->zmq.controller_select = 2;                  //Update and clear buffer
    fing->zmq.data1 = fing->angle1_setpoint * 3.14/180.0 ;
    fing->zmq.data2 = fing->angle2_setpoint * 3.14/180.0 ;
    fing->zmq.data3 = 0;
    fing->zmq.data4 = 0;

    //Send if broadcasting is enabled
    if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(fing->toggle_broadcast))){
      zmq_update(input_ptr);
    }
  }

  static void update_radius(GtkRange *range, gpointer* input_ptr) {
    finger* fing = (finger*)input_ptr;

    fing->radius_setpoint = (int)gtk_range_get_value(range);  //Update value
    fing->zmq.controller_select = 3;                 //Update and clear buffer
    fing->zmq.data1 = cos(fing->angle_setpoint*(3.14/180))*fing->radius_setpoint * 0.001;
    fing->zmq.data2 = sin(fing->angle_setpoint*(3.14/180))*fing->radius_setpoint * 0.001;
    fing->zmq.data3 = 0;
    fing->zmq.data4 = 0;

    //Send if broadcasting is enabled
    if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(fing->toggle_broadcast))){
      zmq_update(input_ptr);
    }
  }
  static void update_angle(GtkRange *range, gpointer* input_ptr) {
    finger* fing = (finger*)input_ptr;

    fing->angle_setpoint = (int)gtk_range_get_value(range);  //Update value
    fing->zmq.controller_select = 3;                //Update and clear buffer
    fing->zmq.data1 = cos(fing->angle_setpoint*(3.14/180))*fing->radius_setpoint * 0.001;
    fing->zmq.data2 = sin(fing->angle_setpoint*(3.14/180))*fing->radius_setpoint * 0.001;
    fing->zmq.data3 = 0;
    fing->zmq.data4 = 0;

    //Send if broadcasting is enabled
    if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(fing->toggle_broadcast))){
      zmq_update(input_ptr);
    }
  }

  static void broadcast(GtkWidget *widget, gpointer* input_ptr) {
    if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget))) {
      ////g_printf("toggle on\n");
      zmq_update(input_ptr);
    }
  }

  static void initiate_or_stop(GtkWidget *widget, gpointer* input_ptr){
    finger* fing = (finger*) input_ptr;
    if (fing->init){
      fing->init=0;
      gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON(fing->toggle_broadcast), FALSE);
      gtk_button_set_label(GTK_BUTTON(widget), "Initialize");
      gtk_widget_set_sensitive ((GtkWidget*) fing->toggle_broadcast, fing->init);
      gtk_widget_set_sensitive ((GtkWidget*) fing->vbox_2, fing->init);
      gtk_widget_set_sensitive ((GtkWidget*) fing->vbox_3, fing->init);
      gtk_widget_set_sensitive ((GtkWidget*) fing->vbox_4, fing->init);
      fing->zmq.controller_select = 0;                //Update and clear buffer
      fing->zmq.data1 = 0;
      fing->zmq.data2 = 0;
      fing->zmq.data3 = 0;
      fing->zmq.data4 = 0;

    }
    else {
      fing->init=1;
      gtk_button_set_label(GTK_BUTTON(widget), "Stop");
      gtk_widget_set_sensitive ((GtkWidget*) fing->toggle_broadcast, fing->init);
      gtk_widget_set_sensitive ((GtkWidget*) fing->vbox_2, fing->init);
      gtk_widget_set_sensitive ((GtkWidget*) fing->vbox_3, fing->init);
      gtk_widget_set_sensitive ((GtkWidget*) fing->vbox_4, fing->init);
      fing->zmq.controller_select = 1;                //Update and clear buffer
      fing->zmq.data1 = 0;
      fing->zmq.data2 = 0;
      fing->zmq.data3 = 0;
      fing->zmq.data4 = 0;
    }

    char message [100];
    sprintf ( message, "%d %d %d %d %d %d", fing->zmq.identity, fing->zmq.controller_select, fing->zmq.data1, fing->zmq.data2, fing->zmq.data3, fing->zmq.data4);


    auto message_obj = CreateSimpleInstructionMsg(builder, fing->zmq.identity, fing->zmq.controller_select, fing->zmq.data1, fing->zmq.data2, fing->zmq.data3, fing->zmq.data4);
    FinishSimpleInstructionMsgBuffer(builder, message_obj);
    uint8_t *buf = builder.GetBufferPointer();
    int size = builder.GetSize();
    zmq_send(fing->zmq.publisher, "B", 1, ZMQ_SNDMORE);
    zmq_send (fing->zmq.publisher, buf, size, 0);
    std::cout << (int) fing->zmq.identity <<" "<< (int) fing->zmq.controller_select <<" "<< (int) fing->zmq.data1<< " " << (int) fing->zmq.data2 << " "<< (int) fing->zmq.data3<< " "<< (int) fing->zmq.data4 << std::endl;
    builder.Clear();
        std::cout <<"size: " << size << std::endl;
  }

  finger(int id, void* context,void* publisher)
    {

    zmq.context = context;
    zmq.publisher = publisher;

    zmq.identity = id;
    zmq.controller_select = 2;
    zmq.data1=0;
    zmq.data2=0;
    zmq.data3=0;
    zmq.data4=0;

    angle1_setpoint = 0;
    angle2_setpoint = 0;
    radius_setpoint = 50;
    angle_setpoint = 0;
    init=0;


    upalign = gtk_alignment_new(0, 0, 0, 0);

    //Add a finger to the vertical box
    frame = gtk_frame_new(NULL);
    gtk_container_add(GTK_CONTAINER(upalign), frame);
    gtk_widget_set_size_request (frame, 500, 100);


    vbox_main = gtk_vbox_new(FALSE, 1);
    gtk_container_add(GTK_CONTAINER(frame), vbox_main);

    sprintf(title,"Finger %d",zmq.identity +1);
    //oss << "Finger " << identity;
    finger_label = gtk_label_new(title);
    gtk_box_pack_start(GTK_BOX(vbox_main), finger_label, TRUE, TRUE, 0);

    //Add a horisontal box to the finger
    hbox = gtk_hbox_new(TRUE, 1);
    gtk_box_pack_start(GTK_BOX(vbox_main), hbox, TRUE, TRUE, 0);

    //Add 4 vertical boxes inside the horisontal box
    vbox_1 = gtk_vbox_new(FALSE, 1);
    vbox_2 = gtk_vbox_new(TRUE, 1);
    vbox_3 = gtk_vbox_new(TRUE, 1);
    vbox_4 = gtk_vbox_new(TRUE, 1);
    gtk_box_pack_start(GTK_BOX(hbox), vbox_1, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(hbox), vbox_2, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(hbox), vbox_3, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(hbox), vbox_4, TRUE, TRUE, 0);

    //Add content to vertical box 1
    init_button = gtk_button_new_with_label("Initialize");
    toggle_broadcast =  gtk_check_button_new_with_label("Broadcast");
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(toggle_broadcast), FALSE);
    gtk_box_pack_start(GTK_BOX(vbox_1), init_button, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(vbox_1), toggle_broadcast, TRUE, TRUE, 0);

    //Add content to vertical box 2
    scale_label_1 = gtk_label_new("link 1 angle: [deg]");
    scale_label_2 = gtk_label_new("link 2 angle: [deg]");
    scale_label_3 = gtk_label_new("Radius: [mm]");
    scale_label_4 = gtk_label_new("Angle: [deg]");
    gtk_box_pack_start(GTK_BOX(vbox_2), scale_label_1, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(vbox_2), scale_label_2, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(vbox_2), scale_label_3, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(vbox_2), scale_label_4, TRUE, TRUE, 0);

    //Add content to vertical box 3
    angle_link1_slider = gtk_hscale_new_with_range(-45, 90, 1);
    angle_link2_slider = gtk_hscale_new_with_range(0, 135, 1);
    radius_slider = gtk_hscale_new_with_range(50, 122, 1);
    angle_slider = gtk_hscale_new_with_range(0, 90, 1);
    gtk_box_pack_start(GTK_BOX(vbox_3), angle_link1_slider, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(vbox_3), angle_link2_slider, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(vbox_3), radius_slider, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(vbox_3), angle_slider, TRUE, TRUE, 0);

    //Add content to vertical box 4
    radio_button1 = gtk_radio_button_new_with_label (NULL, "Angle");
    //entry = gtk_entry_new ();
    //gtk_container_add (GTK_CONTAINER (radio_button1), entry);
    //gtk_box_pack_start (GTK_BOX (vbox_4_f1), radio_button1_f1, TRUE, TRUE, 0);
    //gtk_widget_show (radio_button1_f1);

    //Create a second radio button, and add it to the same group as Button 1*/

    radio_button2 = gtk_radio_button_new_with_label_from_widget (GTK_RADIO_BUTTON (radio_button1), "Polar coordinates");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (radio_button1), TRUE);
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (radio_button2), FALSE);
    gtk_box_pack_start (GTK_BOX (vbox_4), radio_button1, TRUE, TRUE, 0);
    gtk_box_pack_start (GTK_BOX (vbox_4), radio_button2, TRUE, TRUE, 0);

    g_signal_connect (GTK_TOGGLE_BUTTON (radio_button1), "toggled", G_CALLBACK (&finger::button_toggled_js), this);
    g_signal_connect (GTK_TOGGLE_BUTTON (radio_button2), "toggled", G_CALLBACK (&finger::button_toggled_cs), this);

    g_signal_connect(angle_link1_slider, "value-changed",  G_CALLBACK(&finger::update_angle1), this);
    g_signal_connect(angle_link2_slider, "value-changed",  G_CALLBACK(&finger::update_angle2), this);
    g_signal_connect(radius_slider, "value-changed",  G_CALLBACK(&finger::update_radius), this);
    g_signal_connect(angle_slider, "value-changed",  G_CALLBACK(&finger::update_angle), this);
    g_signal_connect(toggle_broadcast, "clicked",G_CALLBACK(&finger::broadcast), this);
    g_signal_connect (init_button, "clicked", G_CALLBACK (&finger::initiate_or_stop), this);
    //radio_group_f1 = gtk_radio_button_get_group (GTK_RADIO_BUTTON (radio_button1_f1));
  //    radio_button1_f1 = gtk_radio_button_new_with_label (radio_group_f1, "button2");
  //    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (radio_button1_f1), TRUE);



    gtk_widget_show_all(upalign);
    gtk_widget_hide(radius_slider);
    gtk_widget_hide(angle_slider);
    gtk_widget_hide(scale_label_3);
    gtk_widget_hide(scale_label_4);

    gtk_widget_set_sensitive ((GtkWidget*) toggle_broadcast, init);
    gtk_widget_set_sensitive ((GtkWidget*) vbox_2, init);
    gtk_widget_set_sensitive ((GtkWidget*) vbox_3, init);
    gtk_widget_set_sensitive ((GtkWidget*) vbox_4, init);


  }
};

void destroy(GtkWidget* widget, gpointer data){
 gtk_main_quit();
}
/*
gint delete_event_handler(GtkWidget* widget, GdkEvent* event, gpointer data){
 //////g_print("delete event occured\n");
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
  sprintf (message, "%c %d %d", casted_iptr->controller_select, casted_iptr->link1_angle, casted_iptr->link2_angle);
  ////////g_printf("%s",message);
  s_sendmore (casted_iptr->publisher, "B");
  s_send (casted_iptr->publisher, message);
  ////////g_printf("Link 1: %d | Link 2: %d \n" ,casted_iptr->link1_angle, casted_iptr->link2_angle);
}

void value_changed1(GtkRange *range, gpointer* input_ptr) {
  input_bundle* casted_iptr = (input_bundle*)input_ptr;
  casted_iptr->link1_angle = (int)gtk_range_get_value(range);

  if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(casted_iptr->toggle))){
    ////g_printf("Hello world 1\n");
    zmq_update(input_ptr);
  }
}

void value_changed2(GtkRange *range, gpointer* input_ptr){
    input_bundle* casted_iptr = (input_bundle*)input_ptr;
    casted_iptr->link2_angle = (int)gtk_range_get_value(range);

  if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(casted_iptr->toggle))){
    ////g_printf("Hello world 2\n");
    zmq_update(input_ptr);
  }
}

void transmission_toggle(GtkWidget *widget, gpointer* input_ptr) {
  if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget))) {
    ////g_printf("toggle on\n");
    zmq_update(input_ptr);
  } else {
    ////g_printf("toggle off\n");
  }
}

//Controller is swapped by clicking on tabs
void controller_swap(GtkWidget *widget1, GtkWidget *widget2, int stuff, gpointer* input_ptr){
    input_bundle* casted_iptr = (input_bundle*)input_ptr;
    ////g_printf("%d\n",stuff);
    if (stuff == 0){
      casted_iptr->controller_select = 0b00000001;
    }  else if (stuff == 1){
      casted_iptr->controller_select = 0b00000010;
    }  else if (stuff == 2){
      casted_iptr->controller_select = 0b00000011;
    }  else if (stuff ==3){
      casted_iptr->controller_select = 0b00000100;
    }
    if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(casted_iptr->toggle))){
      ////g_printf("Hello world 2\n");
      zmq_update(input_ptr);
    }
}
*/

int main (int  argc, char *argv[])
{

    gtk_init(&argc, &argv);
    //flatbuffers::FlatBufferBuilder builder(1024);
    //input_bundle *input = g_new0(input_bundle, 1);
    //input->controller_select=0b00000001;
    //input->publisher = g_new0(void*, 1)
    //input->random_nunmber =  g_new0(void*, 1)
    //
    //ZMQ: Prepare our context and publisher//*
    /*
    printf("init\n");
    void *context = zmq_ctx_new ();
    printf("init\n");
    input->publisher = zmq_socket (context, ZMQ_PUB);
    printf("init\n");
    zmq_bind (input->publisher, "tcp://*:5563");
    //zmq_bind (input->publisher, "tcp://enp1s0f1:5563");
    */

    void* context = zmq_ctx_new ();
    void* publisher = zmq_socket (context, ZMQ_PUB);
    zmq_bind (publisher, "tcp://enp1s0f1:5563");
    //zmq_bind (publisher, "tcp://wlp1s0:5563");

    /*
    zmq::context_t context (1);
    zmq::socket_t publisher (context, ZMQ_PUB);
    publisher.bind("tcp://*:5556");
    publisher.bind("ipc://weather.ipc");
    */
    //zmq_bind (publisher, "tcp://*:5563");
    printf("init\n");
    //GTK: Widget pointers
    GtkWidget* window;
    GtkWidget* notebook;
    //Page one
      GtkWidget* tablabel1;
      GtkWidget* vbox_main_tab1;
        GtkWidget* align_add_del;
        GtkWidget* finger_frame_main;
          GtkWidget* finger_frame_vbox;
            finger finger1(0, context, publisher);
            finger finger2(1, context, publisher);
            finger finger3(2, context, publisher);
            finger finger4(3, context, publisher);
            finger finger5(4, context, publisher);

    //Create window
    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_CENTER);
    gtk_window_set_default_size(GTK_WINDOW(window), 300, 500);
    gtk_container_set_border_width(GTK_CONTAINER(window), 10);
    gtk_window_set_title(GTK_WINDOW(window), "Gripper");

    //Add notebook to window
    notebook = gtk_notebook_new();
    gtk_container_add(GTK_CONTAINER(window),notebook);

     //Main widget tab 1 is a vbox
    vbox_main_tab1=gtk_vbox_new(TRUE, 1);
    tablabel1 = gtk_label_new("Induvidual control");
    gtk_notebook_append_page(GTK_NOTEBOOK(notebook), vbox_main_tab1, tablabel1);
    //*****ADD THE ADD/DEL FINGER STUFF HERE
    //Add a frame for fingers
    finger_frame_main = gtk_frame_new(NULL);
    gtk_box_pack_start(GTK_BOX(vbox_main_tab1), finger_frame_main, TRUE, TRUE, 0);

    //Add a vertical box inside finger_frame_main
    finger_frame_vbox = gtk_vbox_new(TRUE, 1);
    gtk_container_add(GTK_CONTAINER(finger_frame_main), finger_frame_vbox);

    //Add a finger to the vertical box
    //finger_constructor(&finger1,1);



    gtk_widget_show_all(window);
    gtk_box_pack_start(GTK_BOX(finger_frame_vbox), finger1.upalign, TRUE, TRUE, 0);

    gtk_box_pack_start(GTK_BOX(finger_frame_vbox), finger2.upalign, TRUE, TRUE, 0);

    gtk_box_pack_start(GTK_BOX(finger_frame_vbox), finger3.upalign, TRUE, TRUE, 0);

    gtk_box_pack_start(GTK_BOX(finger_frame_vbox), finger4.upalign, TRUE, TRUE, 0);

    gtk_box_pack_start(GTK_BOX(finger_frame_vbox), finger5.upalign, TRUE, TRUE, 0);

//    gtk_widget_show (radio_button1_f1);
    /*
    hscale1 = gtk_hscale_new_with_range(0, 100, 1);
    gtk_scale_set_draw_value(GTK_SCALE(hscale1), TRUE);
    gtk_widget_set_size_request(hscale1, 150, -1);

    hscale2 = gtk_hscale_new_with_range(0, 100, 1);
    gtk_scale_set_draw_value(GTK_SCALE(hscale2), TRUE);
    gtk_widget_set_size_request(hscale2, 150, -1);

    init_button1 = gtk_button_new_with_label("Initialize");
    //  gtk_widget_set_hexpand (init_button1, TRUE);
    gtk_widget_set_size_request(init_button1, 100, 30);
    gtk_container_add(GTK_CONTAINER(halign), init_button1);
    //fixed = gtk_fixed_new();
    //gtk_fixed_put(GTK_FIXED(fixed), init_button1, 150, 50);

    input->toggle = gtk_check_button_new_with_label("Broadcast");
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(input->toggle), FALSE);
    GTK_WIDGET_UNSET_FLAGS(input->toggle, GTK_CAN_FOCUS);


    label1 = gtk_label_new("Link 1 angle: ");
    label2 = gtk_label_new("Link 2 angle: ");
    tablabel1 = gtk_label_new("Joint space PID");
    tablabel2 = gtk_label_new("Cartesian PID");
    tablabel3 = gtk_label_new("Hybrid position/force control [Cartesian]");
    tablabel4 = gtk_label_new("Hybrid position/force control [Polar]");

    grid_tab1 = gtk_table_new(3,4,FALSE);
    gtk_table_set_row_spacings(GTK_TABLE(grid_tab1), 2);
    gtk_table_set_col_spacings(GTK_TABLE(grid_tab1), 2);
    gtk_widget_set_size_request(grid_tab1, 450, 100);

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
    gtk_table_attach_defaults(GTK_TABLE(grid_tab1), label1, 1, 2, 0, 1);
    gtk_table_attach_defaults(GTK_TABLE(grid_tab1), hscale1, 2, 3, 0, 1);
    gtk_table_attach_defaults(GTK_TABLE(grid_tab1), label2, 1, 2, 1, 2);
    gtk_table_attach_defaults(GTK_TABLE(grid_tab1), hscale2, 2, 3, 1, 2);
    gtk_table_attach_defaults(GTK_TABLE(grid_tab1), halign, 0, 1, 0, 1);
    gtk_table_attach_defaults(GTK_TABLE(grid_tab1), input->toggle, 0, 1, 1, 2);





    gtk_container_add(GTK_CONTAINER(valign), grid_tab1);
    gtk_container_add(GTK_CONTAINER(finger_frame_1), valign);
    //gtk_container_add (GTK_CONTAINER (e_box3), tablabel3);
    //gtk_widget_show(e_box3);
    //gtk_widget_show (tablabel3);
    //gtk_widget_set_events (e_box3, GDK_BUTTON_PRESS_MASK);
    //gtk_signal_connect (GTK_OBJECT(e_box3), "button_press_event", GTK_SIGNAL_FUNC (tab3), NULL);

    gtk_notebook_append_page( notebook,  finger_frame_1, tablabel1);
    gtk_notebook_append_page( notebook,  grid_tab2, tablabel2);
    gtk_notebook_append_page( notebook,  grid_tab3, tablabel3);
    //gtk_notebook_append_page( notebook,  grid_tab4, tablabel4);

  //  notebook->connect('switch-page',GTK_SIGNAL_FUNC(tabs3));
    */
    /*
    gtk_notebook_append_page( notebook,  grid_tab4, e_box4);
    gtk_container_add (GTK_CONTAINER (e_box4), tablabel4);
    gtk_widget_show(e_box4);
    gtk_widget_show (tablabel4);
    gtk_widget_set_events (e_box4, GDK_BUTTON_PRESS_MASK);
    gtk_signal_connect (GTK_OBJECT(e_box4), "button_press_event", GTK_SIGNAL_FUNC (tab4), NULL);
    */
    /*
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

    g_signal_connect(notebook, "switch-page",GTK_SIGNAL_FUNC(controller_swap), (gpointer)input);
    */
    /*
  //  g_signal_connect(tablabel4, "clicked", G_CALLBACK(tab4), NULL);
    */


    GtkListStore *model;
    GtkWidget *c1, *c2;


  //  gtk_widget_hide(finger1.scale_label_1);
    //gtk_widget_hide(finger1.scale_label_3);
    //gtk_widget_hide(finger1.scale_label_4);
    //gtk_widget_hide(finger1.xpos);
    //gtk_widget_hide(finger1.ypos);

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
