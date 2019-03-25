//  Pubsub envelope subscriber

#include "zhelpers.h"
#include "pthread.h"

typedef struct read_zmq_bundle {
   char* address;
   char* contents;
   void* context;
   void* subscriber;
   int link1_angle;
   int link2_angle;
}read_zmq_bundle;


pthread_t tid[2];
pthread_mutex_t lock;

void* read_reference_angle(read_zmq_bundle* zmq_read){
  while (1) {
      //  Read envelope with address
      zmq_read->address = s_recv (zmq_read->subscriber);
      //  Read message contents
      zmq_read->contents = s_recv (zmq_read->subscriber);
      //printf("%s\n", contents);
      pthread_mutex_lock(&lock);
      sscanf(zmq_read->contents, "%d %d", &zmq_read->link1_angle, &zmq_read->link2_angle);
      //printf("| %s %s\n", garbage1,garbage2);
      //sscanf(contents, "%lf[^ ]%lf[^\n]", &link1_angle, &link2_angle);
      printf("%d %d\n", zmq_read->link1_angle, zmq_read->link2_angle);
      pthread_mutex_unlock(&lock);
      //printf("%s\n", contents);
      free (zmq_read->address);
      free (zmq_read->contents);
    }
}

int main (void)
{
    read_zmq_bundle zmq_read;
    //  Prepare our context and subscriber
    zmq_read.context = zmq_ctx_new ();
    zmq_read.subscriber = zmq_socket (zmq_read.context, ZMQ_SUB);

    //void *context = zmq_ctx_new ();
    //void *subscriber = zmq_socket (context, ZMQ_SUB);
    //zmq_connect (subscriber, "tcp://10.218.130.229:5563");
    zmq_connect (zmq_read.subscriber, "tcp://169.254.27.157:5563");
    zmq_setsockopt (zmq_read.subscriber, ZMQ_SUBSCRIBE, "B", 1);

    pthread_create(&(tid[0]), NULL, &read_reference_angle, &zmq_read);
    //read_reference_angle(address, contents, subscriber, &link1_angle, &link2_angle);

    if (pthread_mutex_init(&lock, NULL) != 0)
    {
        printf("\n mutex init failed\n");
        return 1;
    }

    while(1){
      printf("I am now reading from memory modified on another thread: %d | %d",&zmq_read.link1_angle, &zmq_read.link2_angle);
    }
  //  while (1) {
  //      //  Read envelope with address
  //      char *address = s_recv (subscriber);
  //      //  Read message contents
  //      char *contents = s_recv (subscriber);
  //      //printf("%s\n", contents);
  //      sscanf(contents, "%d %d", &link1_angle, &link2_angle);
  //      //printf("| %s %s\n", garbage1,garbage2);
  //      //sscanf(contents, "%lf[^ ]%lf[^\n]", &link1_angle, &link2_angle);
  //      printf("%d %d\n", link1_angle, link2_angle);
        //printf("%s\n", contents);
  //      free (address);
  //      free (contents);
  //  }
    //  We never get here, but clean up anyhow
    zmq_close (zmq_read.subscriber);
    zmq_ctx_destroy (zmq_read.context);
    return 0;
}
