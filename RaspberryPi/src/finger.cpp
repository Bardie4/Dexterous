#include <pthread.h>
static pthread_mutex_t zmqSubLock = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t periphLock = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t begin_control_iteration = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t start_cond = PTHREAD_COND_INITIALIZER;
