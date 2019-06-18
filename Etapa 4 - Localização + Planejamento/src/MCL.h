#ifndef MCL_H
#define MCL_H

#include <string>
#include <vector>
#include <random>

#include "Grid.h"
#include "Utils.h"

typedef struct{
    double rot1;
    double trans;
    double rot2;
} Action;

typedef struct{
    Pose p;
    double w;
} MCLparticle;

class MCL
{
public:
    MCL(float maxRange, std::string mapName, pthread_mutex_t* m);
    ~MCL();

    void run(const Action &u, const std::vector<float> &z);

    void draw();
    int mapWidth;
    int mapHeight;

    bool transparency;

    Pose goal;

    CellOccType** mapCells;
    Pose meanParticlePose;
    float covAngle, covMajorAxis, covMinorAxis;

private:
    void readMap(std::string mapName);
    void initParticles();

    void sampling(const Action &u);
    void weighting(const std::vector<float> &z);
    void resampling();

    float computeExpectedMeasurement(int index, Pose &pose);
    double measurementLikelihood(double value, double mean, double var);

    void updateMeanAndCovariance();

    std::default_random_engine* generator;

    double scale;
    float maxRange;

    int numParticles;
    std::vector<MCLparticle> particles;



    pthread_mutex_t* mutex;
};

#endif // MCL_H
