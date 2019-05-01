#include "PioneerBase.h"

#include <GL/glut.h>


PioneerBase::PioneerBase()
{
    // reset robot position in simulator
    resetSimPose_ = true;

    // sensors variables
    numSonars_ = 8;
    sonars_.resize(numSonars_, 0.0);
    numLasers_ = 181;
    lasers_.resize(numLasers_, 0.0);
    maxLaserRange_ = 6.5; // 5.0;

    // wheels' velocities
    vLeft_ = vRight_ = 0.0;
}

//////////////////////////////////
///// INITIALIZATION METHODS /////
//////////////////////////////////

bool PioneerBase::initialize(ConnectionMode cmode, LogMode lmode, std::string fname)
{
    // initialize logfile
    logFile_ = new LogFile(lmode,fname);

    int argc=0; char** argv;

    // initialize ARIA
    Aria::init();
    bool success=false;

    switch (cmode)
    {
        case SERIAL:
        {
            argc=6;
            argv =(char **) new char*[6];

            argv[0]= new char[4];
            argv[1]= new char[13];
            argv[2]= new char[15];
            argv[3]= new char[7];

            argv[4]= new char[4];
            argv[5]= new char[13];

            strcpy(argv[0],"-rp");
            strcpy(argv[1],"/dev/ttyUSB0");

            strcpy(argv[2],"-laserPortType");
            strcpy(argv[3],"serial");
            strcpy(argv[4],"-lp");
            strcpy(argv[5],"/dev/ttyUSB1");
            break;
        }
        case WIFI:
        {
            argc=4;
            argv =(char **) new char*[4];
            argv[0]= new char[4];
            argv[1]= new char[20];
            argv[2]= new char[20];
            argv[3]= new char[7];

            strcpy(argv[0],"-rh");
            strcpy(argv[1],"192.168.1.11");
            strcpy(argv[2],"-remoteLaserTcpPort");
            strcpy(argv[3],"10002");
            break;
        }
        case SIMULATION:
        {
            argc=2;
            argv =(char **) new char*[2];

            argv[0]= new char[4];
            argv[1]= new char[20];

            strcpy(argv[0],"-rh");
            strcpy(argv[1],"localhost");
            break;
        }
    }

    success = initARIAConnection(argc,argv);
    if(!success){
        printf("Could not connect to robot... exiting\n");
        return false;
    }

    if(cmode==SIMULATION && resetSimPose_)
        resetSimPose();

    return true;
}

bool PioneerBase::initARIAConnection(int argc, char** argv)
{
    parser_= new ArArgumentParser(&argc, argv);
    robotConnector_ = new ArRobotConnector(parser_,&robot_);
    int success=robotConnector_->connectRobot();
    if(!success){
        Aria::shutdown();
        return false;
    }

    robot_.addRangeDevice(&sick_);

    laserConnector_ = new ArLaserConnector(parser_, &robot_, robotConnector_);
    laserConnector_->setupLaser(&sick_);

    robot_.addRangeDevice(&(sonarDev_));

    sick_.runAsync();
    robot_.setHeading(0);
    robot_.runAsync(true);
    robot_.enableMotors();
    robot_.setRotVelMax(10);
    printf("Connecting...\n");
    if (!laserConnector_->connectLaser(&(sick_))){
        printf("Could not connect to lasers... exiting\n");
        Aria::shutdown();
        return false;
    }

    return true;
}

void PioneerBase::resetSimPose()
{
    ArRobotPacket pkt;
    pkt.setID(ArCommands::SIM_RESET);
    pkt.uByteToBuf(0); // argument type: ignored.
    pkt.finalizePacket();
    robot_.getDeviceConnection()->write(pkt.getBuf(), pkt.getLength());
}

void PioneerBase::closeARIAConnection()
{
    robot_.stopRunning(true);
    robot_.disconnect();
    sick_.lockDevice();
    sick_.stopRunning();
    Aria::exit(0);
    Aria::shutdown();
    if(parser_!=NULL)
        delete parser_;
    if(robotConnector_!=NULL)
        delete robotConnector_;
    if(laserConnector_!=NULL)
        delete laserConnector_;
}

///////////////////////////
///// DRAWING METHODS /////
///////////////////////////

void PioneerBase::drawBase()
{
    glColor3f(1.0,0.0,0.0);
    glBegin( GL_POLYGON );
    {
        glVertex2f(-20, -8);
        glVertex2f(-13, -15);
        glVertex2f(8, -15);
        glVertex2f(15, -8);
        glVertex2f(15, 8);
        glVertex2f(8, 15);
        glVertex2f(-13, 15);
        glVertex2f(-20, 8);
    }
    glEnd();
    glColor3f(0.0,0.0,0.0);
    glBegin( GL_LINE_STRIP );
    {
        glVertex2f(0, 0);
        glVertex2f(30, 0);
    }
    glEnd();
}

void PioneerBase::drawSonars(bool drawCones)
{
    float angles[8] = {DEG2RAD(-90), DEG2RAD(-50), DEG2RAD(-30), DEG2RAD(-10),
                       DEG2RAD(10), DEG2RAD(30), DEG2RAD(50), DEG2RAD(90)};
    std::vector<float> s = getSonarReadings();

    glRotatef(-90,0.0,0.0,1.0);

    if(drawCones){

        float fov=DEG2RAD(15);
        float hfov=fov/2.0;

        for(int i=0; i<8; i++){
            if(i%2==0)
                glColor4f(1.0,1.0,0.0,0.5);
            else
                glColor4f(0.0,1.0,1.0,0.3);

            glBegin( GL_POLYGON);
            {
                glVertex2f(s[i]*sin(angles[i]-fov)*100, s[i]*cos(angles[i]-fov)*100);
                glVertex2f(s[i]*sin(angles[i]-hfov)*100, s[i]*cos(angles[i]-hfov)*100);
                glVertex2f(s[i]*sin(angles[i])*100, s[i]*cos(angles[i])*100);
                glVertex2f(s[i]*sin(angles[i]+hfov)*100, s[i]*cos(angles[i]+hfov)*100);
                glVertex2f(s[i]*sin(angles[i]+fov)*100, s[i]*cos(angles[i]+fov)*100);
                glVertex2f(0, 0);
            }
            glEnd();
        }

    }else{
        for(int i=0; i<8; i++){
            if(i%2==0)
                glColor3f(0.7,0.7,0.0);
            else
                glColor3f(0.0,0.7,0.7);

            glBegin( GL_LINES);
            {
                glVertex2f(s[i]*sin(angles[i])*100, s[i]*cos(angles[i])*100);
                glVertex2f(0, 0);
            }
            glEnd();
        }

    }

    glRotatef(90,0.0,0.0,1.0);
}

void PioneerBase::drawLasers(bool fill)
{
    std::vector<float> s = getLaserReadings();
    float angle = DEG2RAD(-90.0);

    int inc = 2;
    float angleInc = DEG2RAD(inc);

    glRotatef(-90,0.0,0.0,1.0);

    if(fill){
        glColor4f(0.0,1.0,0.0,0.3);
        glBegin( GL_POLYGON);
        {
            for(int i=0;i<s.size(); i+=inc)
            {
                glVertex2f(s[i]*sin(angle)*100, s[i]*cos(angle)*100);
                angle += angleInc;
            }
            glVertex2f(0, 0);
        }
        glEnd();
    }else{
        glColor3f(0.0,0.7,0.0);
        glBegin( GL_LINES);
        {
            for(int i=0;i<s.size(); i+=inc)
            {
                glVertex2f(s[i]*sin(angle)*100, s[i]*cos(angle)*100);
                glVertex2f(0, 0);
                angle += angleInc;
            }
        }
        glEnd();
    }

    glRotatef(90,0.0,0.0,1.0);
}

////////////////////////////////////////////////////////////////
////// METHODS FOR READING ODOMETRY & SENSORS MEASUREMENTS /////
////////////////////////////////////////////////////////////////

bool PioneerBase::readOdometryAndSensors()
{
    std::vector < ArSensorReading > *readings;
    std::vector < ArSensorReading > ::iterator it;
    ArPose p;
    sick_.lockDevice();
    readings = sick_.getRawReadingsAsVector();
    it = readings->begin();

    if(!readings->empty()){
        p = (*it).getPoseTaken();
    }
    else{
        sick_.unlockDevice();
        return false;
    }

    // coordinates are given in mm, we convert to m
    odometry_.x = p.getX()/1000.0;
    odometry_.y = p.getY()/1000.0;
    odometry_.theta = p.getTh();

    while (odometry_.theta > 180.0)
        odometry_.theta -= 360.0;
    while (odometry_.theta < -180.0)
        odometry_.theta += 360.0;

    // sensors readings are given in mm, we convert to m
    int i = 0;
    for (it = readings->begin(); it!=readings->end(); it++){
        lasers_[i++] = (float)(*it).getRange()/1000.0;
    }

    sick_.unlockDevice();

    for(int i=0;i<numSonars_;i++)
        sonars_[i]=(float)(robot_.getSonarRange(i))/1000.0;

    return true;
}

float PioneerBase::getMinSonarValueInRange(int idFirst, int idLast)
{
    float min = sonars_[idFirst];
    for(int i=idFirst+1; i<=idLast; i++)
        if(sonars_[i]<min)
            min = sonars_[i];

    return min;
}

float PioneerBase::getMinLaserValueInRange(int idFirst, int idLast, int kernelSize)
{
    float min = 1000000;
    for(int i=idFirst; i<=idLast; i++){
        float val=0;
        int c=0;
        for(int j=i-kernelSize;j<=i+kernelSize;j++)
            if(j>=0 && j<numLasers_){
                val+=lasers_[j];
                c++;
            }
        val /= c;

        if(val<min)
            min = val;
    }

    return min;
}

float PioneerBase::getMaxLaserRange()
{
    return maxLaserRange_;
}

int PioneerBase::getNumLasers()
{
    return numLasers_;
}

int PioneerBase::getNumSonars()
{
    return numSonars_;
}

const Pose& PioneerBase::getOdometry()
{
    return odometry_;
}

const std::vector<float> &PioneerBase::getLaserReadings()
{
    return lasers_;
}

const std::vector<float> &PioneerBase::getSonarReadings()
{
    return sonars_;
}

void PioneerBase::setOdometry(const Pose &o)
{
    odometry_ = o;
}

void PioneerBase::setSonarReadings(const std::vector<float> &s)
{
    sonars_ = s;
}

void PioneerBase::setLaserReadings(const std::vector<float> &l)
{
    lasers_ = l;
}

//////////////////////////////
///// NAVIGATION METHODS /////
//////////////////////////////

void PioneerBase::setMovementSimple(MovingDirection dir)
{
    switch(dir){
        case FRONT:
            vLeft_ = 300;
            vRight_ = 300;
            break;
        case BACK:
            vLeft_ = -300;
            vRight_ = -300;
            break;
        case LEFT:
            vLeft_ = -60;
            vRight_ = 60;
            break;
        case RIGHT:
            vLeft_ = 60;
            vRight_ = -60;
            break;
        case STOP:
            vLeft_  = 0;
            vRight_ = 0;
            linVel_ = 0;
            angVel_ = 0;
            oldVLeft_  = vLeft_;
            oldVRight_ = vRight_;
            break;
        case RESTART:
            vLeft_  = oldVLeft_;
            vRight_ = oldVRight_;
            break;
    }

    std::cout << "vLeft_:" << vLeft_ << " vRight_:" << vRight_ << std::endl;
}

void PioneerBase::setMovementVel(MovingDirection dir)
{
    switch(dir){
        case DEC_LIN_VEL:
            if(linVel_ > -0.5)
                linVel_ -= 0.1;
            break;
        case INC_LIN_VEL:
            if(linVel_ < 0.5)
                linVel_ += 0.1;
            break;
        case DEC_ANG_VEL:
            if(angVel_ > -1.0)
                angVel_ -= 0.05;
            break;
        case INC_ANG_VEL:
            if(angVel_ < 1.0)
                angVel_ += 0.05;
            break;
        case STOP:
            linVel_ = 0;
            angVel_ = 0;
            break;
    }

    setWheelsVelocity_fromLinAngVelocity(linVel_,angVel_);
}

void PioneerBase::setWheelsVelocity_fromLinAngVelocity(float linV, float angV)
{
    float b=0.38;

    vLeft_  = linV - angV*b/(2.0);
    vRight_ = linV + angV*b/(2.0);

    // the robot's velocity is given in mm/s
    vLeft_ *= 1000;
    vRight_ *= 1000;

    double maxVel = 500;
    if(vLeft_ > maxVel)
        vLeft_ = maxVel;
    else if(vLeft_ < -maxVel)
        vLeft_ = -maxVel;
    if(vRight_ > maxVel)
        vRight_ = maxVel;
    else if(vRight_ < -maxVel)
        vRight_ = -maxVel;

    std::cout << "linVel: " << linV << " angVel: " << angV << std::endl;
    std::cout << "vLeft_: " << vLeft_ << " vRight_: " << vRight_ << std::endl;

}

void PioneerBase::setWheelsVelocity(float lv, float rv)
{
    vLeft_ = lv;
    vRight_ = rv;
}

void PioneerBase::stopMovement()
{
    robot_.stop();
}

void PioneerBase::resumeMovement()
{
    robot_.setVel2(vLeft_, vRight_);
}

bool PioneerBase::isMoving()
{
    if(robot_.getRightVel()!=0.0)
        return true;
    if(robot_.getLeftVel()!=0.0)
        return true;
    return false;
}

/////////////////////////////////////////////////////
////// METHODS FOR READING & WRITING ON LOGFILE /////
/////////////////////////////////////////////////////

// Prints to file the data that we would normally be getting from sensors, such as the laser and the odometry.
// This allows us to later play back the exact run.
void PioneerBase::writeOnLog()
{
    logFile_->writePose("Odometry",odometry_);
    logFile_->writeSensors("Sonar",sonars_);
    logFile_->writeSensors("Laser",lasers_);
}

// Reads back into the sensor data structures the raw readings that were stored to file
// While there is still information in the file, it will return 0. When it reaches the end of the file, it will return 1.
bool PioneerBase::readFromLog() {

    if(logFile_->hasEnded())
        return true;

    setOdometry(logFile_->readPose("Odometry"));
    setSonarReadings(logFile_->readSensors("Sonar"));
    setLaserReadings(logFile_->readSensors("Laser"));

    return false;
}

