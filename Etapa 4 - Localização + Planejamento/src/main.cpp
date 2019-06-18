#include <pthread.h>
#include <iostream>
#include <string.h>

#include "Robot.h"
#include "Planning.h"
#include "GlutClass.h"

ConnectionMode connectionMode;
LogMode logMode;

std::string filename, mapName;
pthread_mutex_t* mutex;

void* startRobotThread (void* ref)
{
    Robot* robot=(Robot*) ref;

    robot->initialize(connectionMode, logMode, filename, mapName);

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
    mapName = "";

    int p = 1;
    while(p<argc)
    {
        // Connection type
        if(!strncmp(argv[p], "sim", 3)){
            connectionMode=SIMULATION;
            p++;
        }else if(!strncmp(argv[p], "wifi", 4)){
            connectionMode=WIFI;
            p++;
        }else if(!strncmp(argv[p], "serial", 6)){
            connectionMode=SERIAL;
            p++;
        }

        // Recording
        if (!strncmp(argv[p], "-R", 2) || (!strncmp(argv[p], "-r", 2) ) ){
            logMode = RECORDING;
            std::cout << "We are recording the robot path and sensor observations." << std::endl;
            p++;
            continue;
        }

        // playing back from previous file
        if (!strncmp(argv[p], "-p", 2) || (!strncmp(argv[p], "-P", 2)) ) {
            logMode = PLAYBACK;
            if(argc >=p+1 ) {
                filename = argv[p+1];
                p+=2;
                continue;
            } else {
                std::cout << "Please, provide the file name of the recorded path." << std::endl;
                exit(0);
            }
        }

        if (!strncmp(argv[p], "-m", 2) || (!strncmp(argv[p], "-M", 2)) ) {
            if(argc >=p+1 ) {
                mapName = argv[p+1];
                p+=2;
                continue;
            } else {
                std::cout << "Please, provide the file name of the map." << std::endl;
                exit(0);
            }
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

