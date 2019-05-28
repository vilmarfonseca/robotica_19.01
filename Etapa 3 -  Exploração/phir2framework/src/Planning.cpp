#include "Planning.h"

#include <queue>
#include <float.h> //DBL_MAX

////////////////////////
///                  ///
/// Métodos Públicos ///
///                  ///
////////////////////////

Planning::Planning()
{
    curPref = 0.0;

    newRobotPosition.x = 0;
    newRobotPosition.y = 0;

    newGridLimits.minX = newGridLimits.minY = 1000;
    newGridLimits.maxX = newGridLimits.maxY = -1000;

    halfWindowSize = 30;
    localGoalRadius = 20;

    goal = NULL;
    localGoal = NULL;

    foundFirstFrontier = false;
}

Planning::~Planning()
{}

void Planning::setGrid(Grid *g)
{
    grid = g;
}

void Planning::setMaxUpdateRange(int r)
{
    maxUpdateRange = 1.2*r*grid->getMapScale();
}

void Planning::setNewRobotPose(Pose p)
{
    newRobotPosition.x = (int)(p.x*grid->getMapScale());
    newRobotPosition.y = (int)(p.y*grid->getMapScale());

    newGridLimits.minX = std::min(newGridLimits.minX,newRobotPosition.x-maxUpdateRange);
    newGridLimits.maxX = std::max(newGridLimits.maxX,newRobotPosition.x+maxUpdateRange);
    newGridLimits.minY = std::min(newGridLimits.minY,newRobotPosition.y-maxUpdateRange);
    newGridLimits.maxY = std::max(newGridLimits.maxY,newRobotPosition.y+maxUpdateRange);
}

void Planning::run()
{
    pthread_mutex_lock(grid->mutex);

    resetCellsTypes();

    // update robot position and grid limits using last position informed by the robot
    robotPosition = newRobotPosition;
    gridLimits = newGridLimits;

    updateCellsTypes();
    expandObstacles();
    detectFrontiers();

    pthread_mutex_unlock(grid->mutex);


    if(!frontierCenters.empty()){
        foundFirstFrontier = true;

        computeHeuristic();
        computeAStar();
        markPathCells();
        findLocalGoal();

    }else{
        // don't have any frontiers remaining

        if(foundFirstFrontier){
            localGoal = NULL;
            std::cout << "EXPLORATION COMPLETE!" << std::endl;
        }
    }

    initializePotentials();

    for(int i=0; i<100; i++)
        iteratePotentials();

    updateGradient();


}

/////////////////////////////////////////////
///                                       ///
/// Métodos para classificacao de celulas ///
///                                       ///
/////////////////////////////////////////////

void Planning::resetCellsTypes()
{
    for(int i=gridLimits.minX;i<=gridLimits.maxX;i++){
        for(int j=gridLimits.minY;j<=gridLimits.maxY;j++){

            Cell* c = grid->getCell(i,j);

            if(c->occType == NEAROBSTACLE)
                c->occType = FREE;

            c->planType = REGULAR;

            c->g = DBL_MAX;
            c->h = DBL_MAX;
            c->f = DBL_MAX;
            c->pi = NULL;
        }
    }

    goal = NULL;
    localGoal = NULL;
}

void Planning::updateCellsTypes()
{
    Cell* c;

    // the type of a cell can be defined as:
    // c->occType = UNEXPLORED
    // c->occType = OCCUPIED
    // c->occType = FREE


    // TODO: classify cells surrounding the robot
    //
    //  (robotPosition.x-maxUpdateRange, robotPosition.y+maxUpdateRange)  -------  (robotPosition.x+maxUpdateRange, robotPosition.y+maxUpdateRange)
    //                     |                       \                         |
    //                     |                        \                        |
    //                     |                         \                       |
    //  (robotPosition.x-maxUpdateRange, robotPosition.y-maxUpdateRange)  -------  (robotPosition.y+maxUpdateRange, robotPosition.y-maxUpdateRange)






}

void Planning::expandObstacles()
{
    int width=1;

    for(int i=robotPosition.x-maxUpdateRange;i<=robotPosition.x+maxUpdateRange;i++){
        for(int j=robotPosition.y-maxUpdateRange;j<=robotPosition.y+maxUpdateRange;j++){
            Cell* c = grid->getCell(i,j);

            if(c->occType == OCCUPIED){
                for(int x=i-width;x<=i+width;x++){
                    for(int y=j-width;y<=j+width;y++){
                        Cell* n = grid->getCell(x,y);
                        if(n->occType == FREE){
                            n->occType = NEAROBSTACLE;
                        }
                    }
                }
            }

        }
    }
}

void Planning::detectFrontiers()
{
    frontierCenters.clear();
    Cell *c, *n;

    int width=1;
    int minFrontierSize = 5;

    // Mark all UNEXPLORED cells in the frontier of FREE space:  planType = FRONTIER
    for(int i=gridLimits.minX;i<=gridLimits.maxX;i++){
        for(int j=gridLimits.minY;j<=gridLimits.maxY;j++){
            c = grid->getCell(i,j);
            if(c->occType == FREE){
                for(int x=i-width;x<=i+width;x++){
                    for(int y=j-width;y<=j+width;y++){
                        n = grid->getCell(x,y);
                        if(n->occType == UNEXPLORED){
                            n->planType = FRONTIER;
                        }
                    }
                }
            }
        }
    }

    // Group all FRONTIER cells
    for(int i=gridLimits.minX;i<=gridLimits.maxX;i++){
        for(int j=gridLimits.minY;j<=gridLimits.maxY;j++){
            c = grid->getCell(i,j);

            // detect a frontier cell that is not MARKED yet
            if(c->planType == FRONTIER){
                c->planType = MARKED_FRONTIER;

                std::vector<Cell*> frontier;

                point2d center;
                center.x = center.y = 0;
                float count = 0;

                // mark all neighbor frontier cells as MARKED_FRONTIER
                // breadth-first search using a queue
                std::queue<Cell*> q;
                q.push(c);
                while(!q.empty())
                {
                    Cell* c = q.front();
                    q.pop();
                    frontier.push_back(c);

                    center.x += c->x;
                    center.y += c->y;
                    count++;

                    for(int x=c->x-width;x<=c->x+width;x++){
                        for(int y=c->y-width;y<=c->y+width;y++){
                            n = grid->getCell(x,y);
                            if(n->planType == FRONTIER){
                                n->planType = MARKED_FRONTIER;
                                q.push(n);
                            }
                        }
                    }
                }

                // keep frontiers that are larger than minFrontierSize
                if(count > minFrontierSize){
                    center.x /= count;
                    center.y /= count;
                    Cell* tmp = grid->getCell(center.x,center.y);

                    // find cell closest to frontierCenter
                    float minDist=FLT_MAX;
                    Cell* closest=NULL;

                    for(unsigned int k=0;k<frontier.size();k++){
                        float dist = sqrt(pow(frontier[k]->x-center.x,2.0)+pow(frontier[k]->y-center.y,2.0));
                        if(dist < minDist){
                            minDist = dist;
                            closest = frontier[k];
                        }
                    }

                    // add center of frontier to list of Goals
                    frontierCenters.push_back(closest);

                }else{

                    // ignore small frontiers
                    for(unsigned int k=0;k<frontier.size();k++){
                        frontier[k]->planType = REGULAR;
                    }
                }
            }
        }
    }


    std::cout << "Number of frontiers: " << frontierCenters.size() << std::endl;
    for(int k=0;k<frontierCenters.size();k++){
        frontierCenters[k]->planType = FRONTIER;
    }

}

//////////////////////////////////////////////////////
///                                                ///
/// Métodos para planejamento de caminhos - A-STAR ///
///                                                ///
//////////////////////////////////////////////////////

void Planning::computeHeuristic()
{
    Cell* c;

    // TODO: update h-value of all FREE cells and all FRONTIER cells in the grid
    //
    //  (gridLimits.minX, gridLimits.maxY)  -------  (gridLimits.maxX, gridLimits.maxY)
    //                     |                       \                         |
    //                     |                        \                        |
    //                     |                         \                       |
    //  (gridLimits.minX, gridLimits.minY)  -------  (gridLimits.maxX, gridLimits.minY)







}

// eight neighbors offset
// usage, i-th neighbor:    n = grid->getCell(c->x+offset[i][0],c->y+offset[i][1]);
int offset[8][2] = {{-1,  1}, { 0,  1}, { 1,  1}, { 1,  0}, { 1, -1}, { 0, -1}, {-1, -1}, {-1,  0}};

// eight neighbors travel cost
// usage, i-th neighbor:    n->g = c->g + cost[i];
double cost[8] = {sqrt(2), 1, sqrt(2), 1, sqrt(2), 1, sqrt(2), 1};

// comparison function used in the priority queue
class Compare
{
public:
    bool operator() (const Cell* a, const Cell* b)
    {
        return a->f > b->f;
    }
};

void Planning::computeAStar()
{
    Cell* c;

    // Priority queue of Cell pointers ordered by key-value c->f
    std::priority_queue<Cell*,std::vector<Cell*>,Compare> pq;

    // pq.push(c)   -- to insert cell in queue
    // c = pq.top() -- to get top cell (the one with the smallest f value)
    // pq.pop()     -- to remove top cell from queue


    // TODO: implement A-Star





}


void Planning::markPathCells()
{
    if(goal != NULL){

        Cell* c = goal->pi;
        while(c != NULL){
            c->planType = PATH;
            c = c->pi;
        }
    }
}

void Planning::findLocalGoal()
{
    Cell* c = goal;

    while(c != NULL){
        double dist = sqrt(pow(c->x-robotPosition.x,2.0)+pow(c->y-robotPosition.y,2.0));
        if(dist < localGoalRadius){
            localGoal = c;
            break;
        }
        c = c->pi;
    }

    localGoal->planType = LOCALGOAL;
}


///////////////////////////////////////////////////
///                                             ///
/// Métodos de atualizacao de campos potenciais ///
///                                             ///
///////////////////////////////////////////////////

void Planning::initializePotentials()
{
    Cell *c;
    for(int i=robotPosition.x-2*halfWindowSize;i<=robotPosition.x+2*halfWindowSize;i++){
        for(int j=robotPosition.y-2*halfWindowSize;j<=robotPosition.y+2*halfWindowSize;j++){
            if(i>=robotPosition.x-halfWindowSize && i<=robotPosition.x+halfWindowSize &&
               j>=robotPosition.y-halfWindowSize && j<=robotPosition.y+halfWindowSize )
                continue;

            c = grid->getCell(i,j);

            c->pot = 0.5;
            c->dirX = c->dirY = 0;
        }
    }

    for(int i=robotPosition.x-halfWindowSize-1;i<=robotPosition.x+halfWindowSize+1;i++){
        c = grid->getCell(i,robotPosition.y-halfWindowSize-1);
        c->pot = 1.0;
        c = grid->getCell(i,robotPosition.y+halfWindowSize+1);
        c->pot = 1.0;
    }
    for(int j=robotPosition.y-halfWindowSize-1;j<=robotPosition.y+halfWindowSize+1;j++){
        c = grid->getCell(robotPosition.x-halfWindowSize-1,j);
        c->pot = 1.0;
        c = grid->getCell(robotPosition.x+halfWindowSize+1,j);
        c->pot = 1.0;
    }

    for(int i=robotPosition.x-halfWindowSize;i<=robotPosition.x+halfWindowSize;i++){
        for(int j=robotPosition.y-halfWindowSize;j<=robotPosition.y+halfWindowSize;j++){
            c = grid->getCell(i,j);

            if(c->occType == OCCUPIED || c->occType == NEAROBSTACLE)
                c->pot = 1.0;
        }
    }

    if(localGoal != NULL)
        localGoal->pot = 0.0;

}

void Planning::iteratePotentials()
{
    Cell* c;

    // the potential of a cell is stored in:
    // c->pot

    Cell *left,*right,*up,*down;

    // the update of a FREE cell in position (i,j) will use the potential of the four adjacent cells
    // where, for example:
    //     left  = grid->getCell(i-1,j);


    // TODO: iterate the potential field in a local window surrounding the robot
    //
    //  (robotPosition.x-halfWindowSize, robotPosition.y+halfWindowSize)  -------  (robotPosition.x+halfWindowSize, robotPosition.y+halfWindowSize)
    //                     |                       \                         |
    //                     |                        \                        |
    //                     |                         \                       |
    //  (robotPosition.x-halfWindowSize, robotPosition.y-halfWindowSize)  -------  (robotPosition.y+halfWindowSize, robotPosition.y-halfWindowSize)






}

void Planning::updateGradient()
{
    Cell* c;

    // the components of the descent gradient of a cell are stored in:
    // c->dirX and c->dirY

    Cell *left,*right,*up,*down;

    // the gradient of a FREE cell in position (i,j) is computed using the potential of the four adjacent cells
    // where, for example:
    //     left  = grid->getCell(i-1,j);


    // TODO: compute the gradient of the FREE cells in the local window
    //
    //  (robotPosition.x-halfWindowSize, robotPosition.y+halfWindowSize)  -------  (robotPosition.x+halfWindowSize, robotPosition.y+halfWindowSize)
    //                     |                       \                         |
    //                     |                        \                        |
    //                     |                         \                       |
    //  (robotPosition.x-halfWindowSize, robotPosition.y-halfWindowSize)  -------  (robotPosition.y+halfWindowSize, robotPosition.y-halfWindowSize)









}
