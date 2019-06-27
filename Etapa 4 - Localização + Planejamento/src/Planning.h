#ifndef __PLANNING_H__
#define __PLANNING_H__

class Planning;

#include <pthread.h>
#include <queue>
#include "Robot.h"
#include "Grid.h"
#include "Utils.h"
#include "MCL.h"

typedef struct
{
    int x,y;
} point2d;

typedef struct
{
    int minX, maxX, minY, maxY;
} bbox;


class Planning {
	public:
        Planning();
        ~Planning();

		void run();

        void initialize();

        void setNewRobotPose(Pose p);
        void setGoalPose(Pose p);
        void setGrid(Grid* g);
        void setMaxUpdateRange(int r);
        void setMapFromMCL(int mapWidth, int mapHeight, CellOccType** mapcells);

        Pose *goalPose;
        Grid* grid;
        double curPref;

	private:

        void resetCellsTypes();
        void updateCellsTypes();
        void expandObstacles();
        void detectFrontiers();

        void computeHeuristic();
        void computeAStar();
        void markPathCells();
        void findLocalGoal();

        void initializePotentials();
        void iteratePotentials();
        void updateGradient();


        point2d robotPosition;
        bbox gridLimits;

        point2d newRobotPosition;
        bbox newGridLimits;

        std::vector<Cell*> frontierCenters;

        Cell *goal, *localGoal;

        int maxUpdateRange;

        int halfWindowSize;
        int localGoalRadius;

        bool foundFirstFrontier;

};


#endif /* __PLANNING_H__ */
