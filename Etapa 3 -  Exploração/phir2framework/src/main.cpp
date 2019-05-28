#include <pthread.h>
#include <iostream>
#include <string.h>

#include "Robot.h"
#include "Planning.h"
#include "GlutClass.h"

ConnectionMode connectionMode;
LogMode logMode;

std::string filename;
pthread_mutex_t* mutex;

void* startRobotThread (void* ref)
{
    Robot* robot=(Robot*) ref;

    robot->initialize(connectionMode, logMode, filename);

    while(robot->isRunning()){
        robot->run();
    }

	return NULL;
}

void* startGlutThread (void* ref)
{
    GlutClass* glut=GlutClass::getInstance();
    glut->setRobot((Robot*) ref);

    glut->initialize();

    glut->process();

	return NULL;
}

void* startPlanningThread (void* ref)
{
    Robot* robot=(Robot*) ref;
    while(!robot->isReady()){
        std::cout << "Planning is waiting..." << std::endl;
        usleep(100000);
    }

    while(robot->isRunning()){
        robot->plan->run();
        usleep(1000);
    }

    return NULL;
}

int main(int argc, char* argv[])
{
    connectionMode = SIMULATION;
    logMode = NONE;
    filename = "";

    if(argc > 1){
        if(!strncmp(argv[1], "sim", 3))
            connectionMode=SIMULATION;
        else if(!strncmp(argv[1], "wifi", 4))
            connectionMode=WIFI;
        else if(!strncmp(argv[1], "serial", 6))
            connectionMode=SERIAL;
    }

    if(argc > 2){
        if (!strncmp(argv[2], "-R", 2)){
            logMode = RECORDING;
        }else if (!strncmp(argv[2], "-r", 2)) {
            logMode = RECORDING;
        }
        else if (!strncmp(argv[2], "-p", 2)) {
            logMode = PLAYBACK;
            filename = argv[3];
        }
        else if (!strncmp(argv[2], "-P", 2)){
            logMode = PLAYBACK;
            filename = argv[3];
        }else if(!strncmp(argv[4], "-n", 2)){
            logMode = NONE;
        }
    }

    pthread_t robotThread, glutThread, potentialThread;

    Robot* r;
    r = new Robot();

    r->grid->mutex = new pthread_mutex_t;
    if (pthread_mutex_init(r->grid->mutex,NULL) != 0){
        printf("\n mutex init failed\n");
        return 1;
    }

    pthread_create(&(robotThread),NULL,startRobotThread,(void*)r);
    pthread_create(&(glutThread),NULL,startGlutThread,(void*)r);
    pthread_create(&(potentialThread),NULL,startPlanningThread,(void*)r);

    pthread_join(robotThread, 0);
    pthread_join(glutThread, 0);
    pthread_join(potentialThread, 0);

    pthread_mutex_destroy(r->grid->mutex);

    return 0;
}

