#ifndef ROBOT_H
#define ROBOT_H

#include <vector>

#include "Grid.h"
#include "PioneerBase.h"
#include "Utils.h"

class Robot
{
public:
    Robot();
    ~Robot();

    void initialize(ConnectionMode cmode, LogMode lmode, std::string fname);
    void run();

    void move(MovingDirection dir);
    void draw(float xRobot, float yRobot, float angRobot);
    void drawPath();

    const Pose& getCurrentPose();

    bool isReady();
    bool isRunning();

    Grid* grid;
    MotionMode motionMode_;
    int viewMode;
    int numViewModes;


protected:

    Pose currentPose_;
    std::vector<Pose> path_;

    bool ready_;
    bool running_;

    // ARIA stuff
    PioneerBase base;

    // Log stuff
    LogFile* logFile_;
    LogMode logMode_;
    void writeOnLog();
    bool readFromLog();

    // Navigation stuff
    void wanderAvoidingCollisions();
    void wallFollow();
    bool isFollowingLeftWall_;

    Timer controlTimer;
};

#endif // ROBOT_H
