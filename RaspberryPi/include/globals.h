#include "pthread.h"
extern pthread_mutex_t zmqSubLock;
extern pthread_mutex_t periphLock;
extern pthread_mutex_t begin_control_iteration;
extern pthread_cond_t start_cond;
