#ifndef ROBOT_H
#define ROBOT_H

#include <vector>

#include "Grid.h"
#include "PioneerBase.h"
#include "Planning.h"
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
    Planning* plan;
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

    void followPotentialField();

    // Mapping stuff
    float getOccupancyFromLogOdds(float logodds);
    void mappingWithHIMMUsingLaser();
    void mappingWithLogOddsUsingLaser();
    void mappingUsingSonar();
    void drawPotGradient(double scale);


    Timer controlTimer;
    void waitTime(float t);

};

#endif // ROBOT_H
