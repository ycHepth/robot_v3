#include <iostream>
//#include "robot.h"

pthread_mutex_t mutex;
pthread_cond_t  cond;

void *robotcontrol(void *pVoid);

int main() {
    /* Thread-related operation */
    pthread_mutex_init(&mutex, 0);

    pthread_t robot;            // new pthread object
    pthread_attr_t attr_robot;  // new pthread object attribute

    if (pthread_attr_init(&attr_robot) != 0)
        perror("[ROBOT-THREAD INIT FAILURE!]");
    if (pthread_create(&robot, &attr_robot, robotcontrol, (void *) NULL) != 0) {
        perror("[ROBOT-THREAD CREATE FAILURE!]");
        return EXIT_FAILURE;
    }

    pthread_join(robot, NULL); // wait for releasing

    std::cout << "[ROBOT-THREAD DESTROYED!]" << std::endl;

    pthread_attr_destroy(&attr_robot);

    pthread_cond_destroy(&cond);
    pthread_mutex_destroy(&mutex);

    return EXIT_SUCCESS;
}
