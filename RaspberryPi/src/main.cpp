

pthread_t tid[6];

typedef struct spi{
	unsigned handle;
	spi_setup setup;
	char outBuf[4];
	char inBuf[4];
}spi;


//Variables used by joint space PID function
typedef struct jointspace_pid_var {
   char read_angle_cmd[2];
   unsigned char inBuf[2];
   short theta1;
   short theta2;
   short error1;
   short error2;
   short theta1_setpoint;
   short theta2_setpoint;
   double kp1;
   double kp2;
   short u1;
   short u2;
   char run;
	 int itr_counter;
}jointspace_pid_var;

//Variables used by cartesian PID function
typedef struct cartesian_pid_var {
   char read_angle_cmd[2];
   unsigned char inBuf[2];
   short theta1;
   short theta2;
   short error1;
   short error2;
   short theta1_setpoint;
   short theta2_setpoint;
   double kp1;
   double kp2;
   short u1;
   short u2;
   char run;
	 double temp;
	 double k1;
	 double k2;
	 double gamma;
	 double l1;
	 double l2;
	 double x;
	 double y;
	 int itr_counter;
}cartesian_pid_var;

typedef struct controller_variables{
	cartesian_pid_var	 cs;
	jointspace_pid_var js;
}controller_variables;

typedef struct zmq_payload{
	short data1;
	short data2;
	short data3;
	short data4;
	short data5;
}zmq_payload;

typedef struct zmq_instructions{
	void (*controller)(void*, void*, void*);
	zmq_payload payload;
}zmq_instructions;

typedef struct finger_data{
	zmq_instructions*  zmq;		             //ptr to shared memory
	zmq_instructions   zmq_local;	   		   //work memory
	controller_variables controller_var;	 //variables used by controllers
  spi spi_data;                          //Constants used for SPI
}finger_data;

typedef struct zmq_data{
	char* address;
	char* contents;
	void* context;
	void* subscriber;
	char function_flag;
	zmq_instructions instr_finger1;
	zmq_instructions instr_finger2;
	zmq_instructions instr_finger3;
	zmq_instructions instr_finger4;
	zmq_instructions instr_finger5;
	int finger_select;
	int controller_select;
  void* controller_ptr[4];
}zmq_data;



class finger{
  public:
    zmq_instructions*  zmq;		             //ptr to shared memory
    zmq_instructions   zmq_local;	   		   //work memory
    controller_variables controller_var;	 //variables used by controllers
    spi spi_data;                          //Constants used for SPI

    void calibration();
    void cartesian_ijc_pid();
    void jointspace_ijc_pid();
    void main_controller_loop();

    finger(int identity, zmq_data* shared){

      zmq = &(shared->instr_finger1);        //Add pointers to the relevant input data from zmq

      //Set default settings for controllers
      controller_var.js.kp1 = 1;                        //joint space controller vaules
      controller_var.js.kp2 = 0.5;                      //joint space controller vaules
      controller_var.js.read_angle_cmd[0] = 0b00000000; //SPI command
      controller_var.js.read_angle_cmd[1] = 0b00000000; //SPI command
      controller_var.cs.kp1 = 1;                        //cartesian space controller values
      controller_var.cs.kp2 = 0.5;                      //cartesian space controller values
      controller_var.cs.read_angle_cmd[0] = 0b00000000; //SPI command
      controller_var.cs.read_angle_cmd[1] = 0b00000000; //SPI command
      controller_var.cs.l1 = 5.3;
      controller_var.cs.l2 = 4.8;
    }


    void run(){
      run_flag = 1; //Thread safe variable
      while(run_flag){
        calibration();
        jointspace_ijc_pid();
        cartesian_ijc_pid();
      }
      pthread_join(tid[identity+1], NULL);
    }

};

class zmq_client{

  private:
  //ZMQ
  char* address;
  char* contents;
  void* context;
  void* subscriber;

  //Input data (payload)
  int data1, data2, data3, data4;
  char finger_select;
  char controller_select;

  //Memory shared by controllers and ZMQ_cleint.
  //Rows:     Finger 1-7  (There is only enough GPIO pins for 7 fingers)
  //Coloums:  run_flag, controller_select, data1, data2, data3, data4
  double commands[7][6];

  //An array of pointers to the functions that starts each finger
  void (* finger_run [7])()

  //Amount of fingers in use
  int finger_count;
  public:

    zmq_client(int num, void (* finger_run_fct_ptr [])()){
      //ZMQ setup
      context = zmq_ctx_new ();
      subscriber = zmq_socket (zmq_var.context, ZMQ_SUB);
      zmq_connect (subscriber, "tcp://localhost:5563");
      zmq_setsockopt (subscriber, ZMQ_SUBSCRIBE, "B", 1);



      //Clear the shared memory that will be used
      finger_count = num;
      std::fill(commands[0], commands[0] + finger_count*6, 0);

      //Load pointers to start functions
      for (int i = 0; i < finger_count; i++){
        finger_run[0] = finger_run_fct_ptr[0];
      }
    }

    run(){
      while(1){
        address = s_recv (subscriber);  //  Read envelope with address
        contents = s_recv (subscriber); //  Read message contents

        finger_select = contents[0];
        controller_select = contents[1];

        //If a viable finger is selected (finger 0-4)
        if ( ( 0 <= finger_select) && (finger_select <= finger_count - 1) ){
          //Read and unload data to shared memory
          sscanf(contents, "%*c %*c %d %d %d %d %d", &data1, &data2, &data3, &data4);
          phread_mutex_lock(&lock);
          commands[finger_select][1] = controller_select;
          commands[finger_select][2] = data1;
          commands[finger_select][3] = data2;
          commands[finger_select][4] = data3;
          commands[finger_select][5] = data4;

          //If the finger is not running, and the new command is not to stop
          if ( (commands[finger_select][0] == 0) && !(controller_select == 0) ){
            //Set a flag in shared memory showing that the finger thread is running
            commands[finger_select][0] = 1;
            //Start a the finger on a new thread.
            pthread_create(&(tid[2+finger_select]), NULL, finger_run[finger_select], NULL);
            //Note that the finger thread will terminate on its own
            //and set the run_flag low when controller_select = 0.
          }
          phread_mutex_unlock(&lock);
        }
      }
    };
};

class spi{
  private:
    int finger_count;
    int id_vec[7];

    int frequency;
    int spi_channel;
    int sclk;
    int mosi;
    int miso;

    int cs_f1_sens1, cs_f1_sens2, cs_f1_esp32;
    int cs_f2_sens1, cs_f2_sens2, cs_f2_esp32;
    int cs_f3_sens1, cs_f3_sens2, cs_f3_esp32;
    int cs_f4_sens1, cs_f4_sens2, cs_f4_esp32;
    int cs_f5_sens1, cs_f5_sens2, cs_f5_esp32;
    int cs_f6_sens1, cs_f6_sens2, cs_f6_esp32;
    int cs_f7_sens1, cs_f7_sens2, cs_f7_esp32;
    int cs_arr[7][3];

    //SHARED MEMORY
    unit16_t angle[7][2];
    unit16_t speed[7][2];
    unit16_t output[7][2];

    char inBuf[2];
    char outBuf[2];
    char read_command_8;
    char read_command_16;
    uint16_t temp;

    uint16_t temp_angle1;
    uint16_t temp_angle2;
    double temp_output1;
    double temp_output2;

  public:
    spi(int num, int id[]){
      finger_count = num;
      id_vec = id;
      //SPI frequency
      frequency = 15000000;

      //SPI channel. Using this channel means that GPIO 8 is used as chip select.
      //It will however not be connected to anything, and only used because the
      //SPI driver requires a channel to be chosen. Since there are only two channels,
      //and we need more, we manualy activate other GPIO pins
      spi_channel = 0;

      //Common pins. All SPI devies are connected to these.
      sclk = 11;
      int mosi = 10;
      int miso = 9;

      //The chip select that is actually connected to devices
      //Note that this is GPIO number and not pin number
      cs_f1_sens1 = 2;
      cs_f1_sens2 = 3;
      cs_f1_esp32 = 4;
      cs_f2_sens1 = 5;
      cs_f2_sens2 = 6;
      cs_f2_esp32 = 7;
      cs_f3_sens1 = 12;
      cs_f3_sens2 = 13;
      cs_f3_esp32 = 14;
      cs_f4_sens1 = 15;
      cs_f4_sens2 = 16;
      cs_f4_esp32 = 17;
      cs_f5_sens1 = 18;
      cs_f5_sens2 = 19;
      cs_f5_esp32 = 20;
      cs_f6_sens1 = 21;
      cs_f6_sens2 = 22;
      cs_f6_esp32 = 23;
      cs_f7_sens1 = 24;
      cs_f7_sens2 = 25;
      cs_f7_esp32 = 26;
      //Cs is packed into array so that it can be accessed by ID
      cs_arr[0] = {cs_f1_sens1, cs_f1_sens2, cs_f1_esp32};
      cs_arr[1] = {cs_f2_sens1, cs_f2_sens2, cs_f2_esp32};
      cs_arr[2] = {cs_f3_sens1, cs_f3_sens2, cs_f3_esp32};
      cs_arr[3] = {cs_f4_sens1, cs_f4_sens2, cs_f4_esp32};
      cs_arr[4] = {cs_f5_sens1, cs_f5_sens2, cs_f5_esp32};
      cs_arr[5] = {cs_f6_sens1, cs_f6_sens2, cs_f6_esp32};
      cs_arr[6] = {cs_f7_sens1, cs_f7_sens2, cs_f7_esp32};
      //Data is not transmitted when cs is high. Therefore; set all to high
      for (int i=0; i <= 6; i++){
        for (int j=0; j <=3; j++){
          gpio_result = gpioWrite(cs_arr[i][j], 1);
        }
      }

      read_command_8 = 0b00000000;
      read_command_16[0] = 0b00000000;
      read_command_16[1] = 0b00000000;
    }

    unit8_t read_angle_8(int &cs){
      outBuf[0] = read_command_8;
      gpio_result = gpioWrite(cs,0);
      spi_result = spiXfer(spi_handle, outBuf, inBuf, 1);
      gpio_result = gpioWrite(cs,1);
      return inBuf[0];
    }
    uint16_t read_angle_16(int &cs){
      outBuf[0] = read_command_16[0];
      outBuf[1] = read_command_16[1];
      gpio_result = gpioWrite(cs,0);
      spi_result = spiXfer(spi_handle, outBuf, inBuf, 2);
      gpio_result = gpioWrite(cs,1);
      temp = inBuf[0] << 8;
      temp = temp + inBuf[1];
      return temp;
    }

    run(){
      for (int = 0; i < finger_count; i++){
        //Read angle via SPI
        temp_angle1=read_angle_16(cs_arr[id_vec[i]][0]);
        temp_angle2=read_angle_16(cs_arr[id_vec[i]][1));

        //Write angle to shared memory. Read output from shared memory
        pthread_muted_locl(&lock);
        angle[id[i]][0] = temp_angle1;
        angle[id[i]][1] = temp_angle2;
        temp_output1 = output[id_vec[i]][0];
        temp_output2 = output[id_vec[i]][1];
        pthread_mutex_unlock(&lock);

        //Write output
        //***************************NOT DONE**********
      }
    }
};

main(){
  //Initiate finger objects. The arguments is the identity of the finger.
  //The identity corresponds to specific SPI pins. Choose a value between 0-6.
  //Additional fingers can be added (max 7 with the amount of GPIO pins on a RaspberryPi).
  int id1 = 0;
  int id2 = 1;
  finger finger1(id1);
  finger finger2(id2);

  //Pack the identities into a vector
  //Create spi object with number of fingers and ID-vector as arguments
  //Run the spi controller on a separate thread
  int identity[2] = [id1, id2];
  spi spi_controller(2, identity);
  pthread_create(&(tid[1]), NULL, &spi_controller.run(), NULL);

  //Create an array of function pointers
  //Fill the array with the address of the function that starts each finger
  //Create a ZMQ client. With number of fingers and the function pointer array as argument
  //Run the ZMQ client on separate thread.
  void (* finger_run_fct_ptr [2])();
  finger_run_fct_ptr[0] = &finger1.run();
  finger_run_fct_ptr[1] = &finger2.run();
  zmq_client zmq(2, finger_run_fct_ptr):
  pthread_create(&(tid[1]), NULL, &zmq.run(), NULL);


  pthread_join(tid[0], NULL);
  pthread_join(tid[1], NULL);
}
