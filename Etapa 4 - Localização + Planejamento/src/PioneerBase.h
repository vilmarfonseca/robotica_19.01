#ifndef PIONEERBASE_H
#define PIONEERBASE_H

#include <Aria.h>

#include "Utils.h"

class PioneerBase
{
public:
    PioneerBase();

    // ARIA stuff
    bool initialize(ConnectionMode cmode, LogMode lmode, std::string fname);
    void closeARIAConnection();

    // Drawing stuff
    void drawBase();
    void drawSonars(bool drawCones=false);
    void drawLasers(bool fill=true);

    // Navigation stuff
    void setMovementSimple(MovingDirection dir);
    void setMovementVel(MovingDirection dir);

    void setWheelsVelocity_fromLinAngVelocity(float linV, float angV);
    void setWheelsVelocity(float vl, float vr);
    bool isMoving();
    void resumeMovement();
    void stopMovement();

    // Sensors stuff
    bool readOdometryAndSensors();
    const Pose& getOdometry();
    void setOdometry(const Pose &o);

    const std::vector<float>& getSonarReadings();
    int getNumSonars();
    float getMaxSonarRange();
    void setSonarReadings(const std::vector<float> &s);
    float getMinSonarValueInRange(int idFirst, int idLast);
    int getNearestSonarBeam(float angle);
    float getAngleOfSonarBeam(int k);
    float getKthSonarReading(int k);

    const std::vector<float>& getLaserReadings();
    int getNumLasers();
    float getMaxLaserRange();
    void setLaserReadings(const std::vector<float> &l);
    float getMinLaserValueInRange(int idFirst, int idLast, int kernelSize=0);
    int getNearestLaserBeam(float angle);
    float getAngleOfLaserBeam(int k);
    float getKthLaserReading(int k);

    // Log stuff
    void writeOnLog();
    bool readFromLog();

private:
    Pose odometry_;
    Pose truePose_;

    // ARIA stuff
    ArRobot robot_;
    ArRobotConnector *robotConnector_;
    ArArgumentParser *parser_;
    ArSonarDevice sonarDev_;
    ArSick sick_;
    ArLaserConnector *laserConnector_;
    bool initARIAConnection(int argc, char** argv);
    void resetSimPose();

    bool resetSimPose_;

    // Navigation stuff
    double vLeft_, vRight_;
    double oldVLeft_, oldVRight_;
    double linVel_, angVel_;

    // Sensors stuff
    int numSonars_;
    std::vector<float> sonars_;
    float maxSonarRange_;
    int numLasers_;
    std::vector<float> lasers_;
    float maxLaserRange_;

    LogFile* logFile_;
};

#endif // PIONEERBASE_H
