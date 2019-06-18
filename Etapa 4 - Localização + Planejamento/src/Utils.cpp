#include "Utils.h"

#include <unistd.h>
#include <sstream>
#include <iomanip>
#include <errno.h>
#include <string.h>

float normalizeAngleDEG(float a)
{
    while(a>180.0)
        a -= 360.0;
    while(a<=-180.0)
        a += 360.0;
    return a;
}

float normalizeAngleRAD(float a)
{
    while(a>M_PI)
        a -= 2*M_PI;
    while(a<=-M_PI)
        a += 2*M_PI;
    return a;
}

/////////////////////////////////
///// METHODS OF CLASS POSE /////
/////////////////////////////////

Pose::Pose(){
    x=y=theta=0.0;
}

Pose::Pose(float a, float b, float c){
    x=a; y=b; theta=c;
}

std::ostream& operator<<(std::ostream& os, const Pose& p)
{
    os << "(" << p.x << ',' << p.y << ',' << p.theta << ")";
    return os;
}

////////////////////////////////////
///// METHODS OF CLASS LOGFILE /////
////////////////////////////////////

LogFile::LogFile(LogMode mode, std::string name)
{
    time_t t = time(0);
    struct tm *now = localtime(&t);
    std::stringstream ss;

    if(mode == RECORDING)
    {
        ss << "../phir2framework/Sensors/sensors-" << -100+now->tm_year
                        << std::setfill('0') << std::setw(2) << 1+now->tm_mon
                        << std::setfill('0') << std::setw(2) << now->tm_mday << '-'
                        << std::setfill('0') << std::setw(2) << now->tm_hour
                        << std::setfill('0') << std::setw(2) << now->tm_min
                        << std::setfill('0') << std::setw(2) << now->tm_sec << ".txt";
        filename = ss.str();

        file.open(filename.c_str(), std::fstream::out);
    }
    else if(mode == PLAYBACK)
    {
        filename = "../phir2framework/Sensors/"+name;
        std::cout << filename << std::endl;
        file.open(filename.c_str(), std::fstream::in);
        if(file.fail()){
            std::cerr << "Error: " << strerror(errno) << std::endl;
            exit(1);
        }
    }
}

Pose LogFile::readPose(std::string info)
{
    std::string tempStr;
    Pose p;

    file >> tempStr >> p.x >> p.y >> p.theta;
    getline(file,tempStr);

    return p;
}

std::vector<float> LogFile::readSensors(std::string info)
{
    int max;
    std::string tempStr;
    std::vector<float> sensors;

    file >> tempStr >> max;
    sensors.resize(max);
    for (int i = 0; i < max; i++) {
        file >> sensors[i];
    }
    getline(file,tempStr);

    return sensors;
}

void LogFile::writePose(std::string info, Pose pose)
{
    file << info << ' ' << pose.x << ' ' << pose.y << ' ' << pose.theta << std::endl;
}

void LogFile::writeSensors(std::string info, std::vector<float> sensors)
{
    file << info << ' ' << sensors.size() << ' ';
    for (int i = 0; i < sensors.size(); i++)
        file << sensors[i] << ' ';
    file << std::endl;
}

bool LogFile::hasEnded()
{
    return file.peek() == std::fstream::traits_type::eof();
}

//////////////////////////////////
///// METHODS OF CLASS TIMER /////
//////////////////////////////////

Timer::Timer()
{
    startCounting();
}

void Timer::startCounting()
{
    gettimeofday(&tstart, NULL);
    gettimeofday(&tlapstart, NULL);
}

void Timer::startLap()
{
    gettimeofday(&tlapstart, NULL);
}

void Timer::stopCounting()
{
    gettimeofday(&tnow, NULL);
}

float Timer::getTotalTime()
{
    gettimeofday(&tnow, NULL);

    if (tstart.tv_usec > tnow.tv_usec) {
        tnow.tv_usec += 1000000;
        tnow.tv_sec--;
    }

    return (float)(tnow.tv_sec - tstart.tv_sec) +
           ((float)tnow.tv_usec - (float)tstart.tv_usec)/1000000.0;
}

float Timer::getLapTime()
{
    gettimeofday(&tnow, NULL);

    if (tlapstart.tv_usec > tnow.tv_usec) {
        tnow.tv_usec += 1000000;
        tnow.tv_sec--;
    }
    return (float)(tnow.tv_sec - tlapstart.tv_sec) +
           ((float)tnow.tv_usec - (float)tlapstart.tv_usec)/1000000.0;
}

void Timer::waitTime(float t){
    float l;
    do{
        usleep(1000);
        l = getLapTime();
    }while(l < t);
    startLap();
}

