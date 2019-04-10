#include "Robot.h"

#include <GL/glut.h>
#include <cmath>
#include <iostream>

//////////////////////////////////////
///// CONSTRUCTORS & DESTRUCTORS /////
//////////////////////////////////////

Robot::Robot()
{
    ready_ = false;
    running_ = true;

    grid = new Grid();

    // variables used for navigation
    isFollowingLeftWall_=false;

    // variables used for visualization
    viewMode=1;
    numViewModes=5;
}

Robot::~Robot()
{
    base.closeARIAConnection();
    if(grid!=NULL)
        delete grid;
}

////////////////////////////////////
///// INITIALIZE & RUN METHODS /////
////////////////////////////////////

void Robot::initialize(ConnectionMode cmode, LogMode lmode, std::string fname)
{
    logMode_ = lmode;
//    logFile_ = new LogFile(logMode_,fname);
    ready_ = true;

    // initialize ARIA
    if(logMode_!=PLAYBACK){
        bool success = base.initialize(cmode,lmode,fname);
        if(!success){
            printf("Could not connect to robot... exiting\n");
            exit(0);
        }
    }

    ready_ = true;
    controlTimer.startLap();
}

void Robot::run()
{
    controlTimer.waitTime(0.1);

    if(logMode_==PLAYBACK){
        bool hasEnded = base.readFromLog();
        if(hasEnded){
            std::cout << "PROCESS COMPLETE. CLOSING PROGRAM." << std::endl;
            exit(0);
        }
    }else{
        bool success = base.readOdometryAndSensors();
        if(!success){
            usleep(50000);
            return;
        }

        if(logMode_==RECORDING)
            base.writeOnLog();
    }

    currentPose_ = base.getOdometry();

    // Save path traversed by the robot
    if(base.isMoving() || logMode_==PLAYBACK){
        path_.push_back(base.getOdometry());
    }

    // Navigation
    switch(motionMode_){
        case WANDER:
            wanderAvoidingCollisions();
            break;
        case WALLFOLLOW:
            wallFollow();
            break;
        case ENDING:
            running_=false;
            break;
        default:
            break;
    }

    base.resumeMovement();

    usleep(50000);
}

//////////////////////////////
///// NAVIGATION METHODS /////
//////////////////////////////

void Robot::move(MovingDirection dir)
{
    switch(dir){
        case FRONT:
            std::cout << "moving front" << std::endl;
            break;
        case BACK:
            std::cout << "moving back" << std::endl;
            break;
        case LEFT:
            std::cout << "turning left" << std::endl;
            break;
        case RIGHT:
            std::cout << "turning right" << std::endl;
            break;
        case STOP:
            std::cout << "stopping robot" << std::endl;
    }

    if(motionMode_==MANUAL_SIMPLE)
        base.setMovementSimple(dir);
    else if(motionMode_==MANUAL_VEL)
        base.setMovementVel(dir);
    else if(motionMode_=WALLFOLLOW)
        if(dir==LEFT)
            isFollowingLeftWall_=true;
        else if(dir==RIGHT)
            isFollowingLeftWall_=false;
}

void Robot::wanderAvoidingCollisions()
{
    float minLeftSonar  = base.getMinSonarValueInRange(0,2);
    float minFrontSonar = base.getMinSonarValueInRange(3,4);
    float minRightSonar = base.getMinSonarValueInRange(5,7);

    float minLeftLaser  = base.getMinLaserValueInRange(0,74);
    float minFrontLaser = base.getMinLaserValueInRange(75,105);
    float minRightLaser = base.getMinLaserValueInRange(106,180);

    float linVel=0.5;
    float angVel=0;

    base.setWheelsVelocity_fromLinAngVelocity(linVel, angVel);

    float limitWallDistance = 0.7;

    bool free, leftFrontBlocked, rightFrontBlocked, leftBlocked, rightBlocked;

    leftFrontBlocked = (minLeftLaser <= limitWallDistance) && (minFrontLaser <= limitWallDistance);

    rightFrontBlocked = (minRightLaser <= limitWallDistance) && (minFrontLaser <= limitWallDistance);

    leftBlocked = (minLeftLaser <= limitWallDistance);

    rightBlocked = (minRightLaser <= limitWallDistance);

    free = (minFrontLaser > limitWallDistance);

    if(free)
    {
        std::cout << "Free." << std::endl;
        base.setMovementSimple(FRONT);
    }
    if(leftFrontBlocked || leftBlocked)
    {
        std::cout << "leftFrontBlocked" << std::endl;
        base.setMovementSimple(RIGHT);
    }
    if(rightFrontBlocked || rightBlocked)
    {
        std::cout << "rightFrontBlocked" << std::endl;
        base.setMovementSimple(LEFT);
    }
}

void Robot::wallFollow()
{
    float minLeftSonar  = base.getMinSonarValueInRange(0,2);
    float minFrontSonar = base.getMinSonarValueInRange(3,4);
    float minRightSonar = base.getMinSonarValueInRange(5,7);

    float minLeftLaser  = base.getMinLaserValueInRange(0,74);
    float minFrontLaser = base.getMinLaserValueInRange(75,105);
    float minRightLaser = base.getMinLaserValueInRange(106,180);

    float linVel=0;
    float angVel=0;

    if(isFollowingLeftWall_)
        std::cout << "Following LEFT wall" << std::endl;
    else
        std::cout << "Following RIGHT wall" << std::endl;

    //TODO - implementar wall following usando PID




    base.setWheelsVelocity_fromLinAngVelocity(linVel, angVel);
}

/////////////////////////////////////////////////////
////// METHODS FOR READING & WRITING ON LOGFILE /////
/////////////////////////////////////////////////////

// Prints to file the data that we would normally be getting from sensors, such as the laser and the odometry.
// This allows us to later play back the exact run.
void Robot::writeOnLog()
{
    logFile_->writePose("Odometry",currentPose_);
    logFile_->writeSensors("Sonar",base.getSonarReadings());
    logFile_->writeSensors("Laser",base.getLaserReadings());
}

// Reads back into the sensor data structures the raw readings that were stored to file
// While there is still information in the file, it will return 0. When it reaches the end of the file, it will return 1.
bool Robot::readFromLog() {

    if(logFile_->hasEnded())
        return true;

    base.setOdometry(logFile_->readPose("Odometry"));
    base.setSonarReadings(logFile_->readSensors("Sonar"));
    base.setLaserReadings(logFile_->readSensors("Laser"));

    return false;
}

////////////////////////
///// DRAW METHODS /////
////////////////////////

void Robot::draw(float xRobot, float yRobot, float angRobot)
{
    float scale = grid->getMapScale();
    glTranslatef(xRobot,yRobot,0.0);
    glRotatef(angRobot,0.0,0.0,1.0);

    glScalef(1.0/scale,1.0/scale,1.0/scale);

    // sonars and lasers draw in cm
    if(viewMode==1)
        base.drawSonars(true);
    else if(viewMode==2)
        base.drawSonars(false);
    else if(viewMode==3)
        base.drawLasers(true);
    else if(viewMode==4)
        base.drawLasers(false);

    // robot draw in cm
    base.drawBase();

    glScalef(scale,scale,scale);
    glRotatef(-angRobot,0.0,0.0,1.0);
    glTranslatef(-xRobot,-yRobot,0.0);
}

void Robot::drawPath()
{
    float scale = grid->getMapScale();

    if(path_.size() > 1){
        glScalef(scale,scale,scale);
        glLineWidth(3);
        glBegin( GL_LINE_STRIP );
        {
            for(unsigned int i=0;i<path_.size()-1; i++){
                glColor3f(1.0,0.0,1.0);

                glVertex2f(path_[i].x, path_[i].y);
                glVertex2f(path_[i+1].x, path_[i+1].y);
            }
        }
        glEnd();
        glLineWidth(1);
        glScalef(1.0/scale,1.0/scale,1.0/scale);

    }
}

/////////////////////////
///// OTHER METHODS /////
/////////////////////////

bool Robot::isReady()
{
    return ready_;
}

bool Robot::isRunning()
{
    return running_;
}

const Pose& Robot::getCurrentPose()
{
    return currentPose_;
}

