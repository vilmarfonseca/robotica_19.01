#ifndef UTILS_H
#define UTILS_H

#include <sys/time.h>
#include <fstream>
#include <iostream>
#include <cmath>
#include <vector>

enum ConnectionMode {SIMULATION, SERIAL, WIFI};
enum LogMode { NONE, RECORDING, PLAYBACK};
enum MotionMode {MANUAL_SIMPLE, MANUAL_VEL, WANDER, WALLFOLLOW, ENDING};
enum MovingDirection {STOP, FRONT, BACK, LEFT, RIGHT, RESTART, DEC_ANG_VEL, INC_ANG_VEL, INC_LIN_VEL, DEC_LIN_VEL};

#define DEG2RAD(x) x*M_PI/180.0

class Pose{
    public:
        Pose();
        Pose(float a, float b, float c);

        friend std::ostream& operator<<(std::ostream& os, const Pose& p);

        float x, y, theta;
};

class LogFile
{
    public:
        LogFile(LogMode mode, std::string name);

        Pose readPose(std::string info);
        std::vector<float> readSensors(std::string info);

        void writePose(std::string info, Pose pose);
        void writeSensors(std::string s, std::vector<float> sensors);

        bool hasEnded();

    private:
        std::fstream file;
        std::string filename;
};

class Timer{
    public:
        Timer();

        void startCounting();
        void startLap();
        void stopCounting();

        float getTotalTime();
        float getLapTime();

        void waitTime(float t);

    private:
        struct timeval tstart, tlapstart, tnow;
};

#endif // UTILS_H
