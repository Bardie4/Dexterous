//  Pubsub envelope subscriber

#include "zhelpers.h"

int main (void)
{
    //  Prepare our context and subscriber
    void *context = zmq_ctx_new ();
    void *subscriber = zmq_socket (context, ZMQ_SUB);
    int link1_angle;
    int link2_angle;
    //zmq_connect (subscriber, "tcp://10.218.130.229:5563");
    zmq_connect (subscriber, "tcp://169.254.27.157:5563");
    zmq_setsockopt (subscriber, ZMQ_SUBSCRIBE, "B", 1);

    while (1) {
        //  Read envelope with address
        char *address = s_recv (subscriber);
        //  Read message contents
        char *contents = s_recv (subscriber);
        //printf("%s\n", contents);
        sscanf(contents, "%d %d", &link1_angle, &link2_angle);
        //printf("| %s %s\n", garbage1,garbage2);
        //sscanf(contents, "%lf[^ ]%lf[^\n]", &link1_angle, &link2_angle);
        printf("%d %d\n", link1_angle, link2_angle);
        //printf("%s\n", contents);
        free (address);
        free (contents);
    }
    //  We never get here, but clean up anyhow
    zmq_close (subscriber);
    zmq_ctx_destroy (context);
    return 0;
}
