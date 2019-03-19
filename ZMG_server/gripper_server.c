

//  Pubsub envelope publisher
//  Note that the zhelpers.h file also provides s_sendmore

#include "zhelpers.h"
#include <unistd.h>

int main (void)
{
    //  Prepare our context and publisher
    printf("init\n");
    void *context = zmq_ctx_new ();
    printf("init\n");
    void *publisher = zmq_socket (context, ZMQ_PUB);
    printf("init\n");
    zmq_bind (publisher, "tcp://192.168.0.69:5563");
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
