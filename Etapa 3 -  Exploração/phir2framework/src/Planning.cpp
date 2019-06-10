#include "Planning.h"

#include <queue>
#include <math.h>
#include <float.h> //DBL_MAX

////////////////////////
///                  ///
/// Métodos Públicos ///
///                  ///
////////////////////////
    int temp_var =0;
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
//        std::cout << "Saiu AStar.";
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

    int maxX, maxY, minX, minY = 0;
    minX = robotPosition.x - maxUpdateRange;
    minY = robotPosition.y - maxUpdateRange;
    maxX = robotPosition.x + maxUpdateRange;
    maxY = robotPosition.y + maxUpdateRange;

    int i, j = 0;
    for(i = minX; i <= maxX; i++)
    {
        for(j = minY; j <= maxY; j++)
        {
            c = grid->getCell(i,j);

            if(c->occType == UNEXPLORED)
            {
                if(c->logodds > 0.5)
                {
                    c->occType = OCCUPIED;
                }
                else
                {
                    c->occType = FREE;
                }
            }
            else
            {
                if(c->logodds >= 0.6)
                {
                    c->occType = OCCUPIED;
                }
                else if(c->logodds <= 0.4)
                {
                    c->occType = FREE;
                }
            }
        }
    }
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

    int i, j = 0;
    float d = FLT_MAX;  // Distância da célula para célula_da_fronteira[i], começa com maior valor int para substituir depois.
    float tempD = 0.0;
    Cell *temp;

    for(i = gridLimits.minX; i <= gridLimits.maxX; i++)
    {
        for(j = gridLimits.minY; j <= gridLimits.maxY; j++)
        {
//            std::cout << "i: " << i << " j: " << j << std::endl;
//            sleep(1);

            c = grid->getCell(i,j);

//          Nas células de objetivo, o valor de c->h será obviamente 0

            if(c->planType == FRONTIER)
            {
                c->h = 0;
//                std::cout << "Célula objetivo: " << c->x << "," << c->y << std::endl;
//                sleep(3);
            }
//          Nas demais células livres (c->occType == FREE), é preciso determinar a distância euclidiana para
//          a célula de objetivo mais próxima dentre todas em std::vector<Cell*> frontierCenters.
            else if (c->occType == FREE)
            {
//                std::cout << "--Célula free--" << std::endl;
//                sleep(4);
                int k = 0;
                int size = frontierCenters.size();
//                std::cout << "Tamanho:" << size << std::endl;
                for(k = 0; k < size; k++)
                {
//                    std::cout << "LOOP: " << k << std::endl;
//                    sleep(4);
                    temp = frontierCenters[k];
                    tempD = sqrt(pow(c->x - temp->x , 2.0) + pow(c->y - temp->y , 2.0));
//                    std::cout << "d:" << d << "tempD:" << tempD << std::endl;
                    if(tempD < d)
                    {
//                        std::cout << "Entrou if!" << std::endl;
                        d = tempD;
                    }
                }
//                std::cout << "k:" << k << std::endl;
                c->h = d;
//                std::cout << "HCost:" << c->h << std::endl;
            }
        }
    }
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
    std::cout << "Entrou para computar o AStar." << std::endl;
    Cell *c;
    Cell *neighbor;
    Cell *start;

    // Priority queue of Cell pointers ordered by key-value c->f
    std::priority_queue<Cell*,std::vector<Cell*>,Compare> pq;
    //std::priority_queue<Cell*,std::vector<Cell*>,Compare> empty;


    // pq.push(c)   -- to insert cell in queue
    // c = pq.top() -- to get top cell (the one with the smallest f value)
    // pq.pop()     -- to remove top cell from queue


    // TODO: implement A-Star

    int x = robotPosition.x;
    int y = robotPosition.y;
    // Start node.
    start = grid->getCell(x,y);
    start->g = start->f = 0;
//    std::cout << "Custo H célula start: " << start->h << std::endl;
//    std::cout << "Posição start: (" << start->x << "," << start->y << ")" << std::endl;
//    sleep(3);

    // Coloca nodo start na fila.
    pq.push(start);

    // Loop para achar o caminho.
    while(pq.top() != NULL && goal == NULL)
    {
        //sleep(4);
//        std::cout << "------------While AStar------------" << std::endl;
        // Pegar primeiro nodo da priority queue.
        c = pq.top();
//        std::cout << "[WHILE]Célula c a ser analisado: (" << c->x << "," << c->y << ")" << std::endl;

        // Como nodo já foi analisado, removê-lo da priority queue.
        pq.pop();

        // Analisar seus vizinhos.
        int i = 0;
        for(i = 0; i <= 7; i++)
        {
            // Altera entre os 8 vizinhos
//            if((c->x+offset[i][0] <= gridLimits.maxX) && (c->x+offset[i][0] >= gridLimits.minX) && (c->y+offset[i][1] <= gridLimits.maxY) && (c->y+offset[i][1] >= gridLimits.minY))
//            {
                neighbor = grid->getCell(c->x+offset[i][0],c->y+offset[i][1]);
                if(neighbor->f == DBL_MAX && goal == NULL)
                {
                    //std::cout << "[IF]Definindo vizinho." << std::endl;
                    neighbor->g = c->g + cost[i];
                    neighbor->f = neighbor->g + neighbor->h;
                    neighbor->pi = c;
    //                std::cout << "[IF]Vizinho custo F." << neighbor->f << std::endl;
                    // A busca do menor caminho deve ser propagada adicionando células vizinhas na fila ATÉ que se encontre
                    // uma célula de fronteira (i.e. até achar uma célula onde c->planType == FRONTIER). Esta célula deve
                    // ser setada como objetivo, i.ie. fazer goal = c.
                    if(neighbor->planType == FRONTIER)
                    {
//                        std::cout << "[IF-FRONTIER]Definindo goal." << std::endl;
                        goal = neighbor;
                    }
                    else
                    {
                        // Depois de atualizar o neighbor, inserí-lo na priority queue.
    //                    std::cout << "[ELSE]Insere vizinho na pq." << std::endl;
                        pq.push(neighbor);
                    }
//                }
            }

//            std::cout << "[FOR]Vizinho a ser analisado: (" << neighbor->x << "," << neighbor->y << ")" << std::endl;
//            std::cout << "[FOR]Custo G = " << neighbor->g << std::endl;

            // Se neighbor ainda não foi setado anteriormente, atualizar suas informações.

//            std::cout << "Numero de iterações do i: " << i << std::endl;
        }
    }
//    pq = std::priority_queue<Cell*,std::vector<Cell*>,Compare>();
//    std::swap(pq,empty);

//    sleep(2);
//    std::cout << "Acabou while AStar." << std::endl;
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

    for(int i=robotPosition.x-halfWindowSize;i<=robotPosition.x+halfWindowSize;i++)
    {
        for(int j=robotPosition.y-halfWindowSize;j<=robotPosition.y+halfWindowSize;j++)
        {
            c = grid->getCell(i,j);
            if(c->planType == LOCALGOAL)
            {
                c->pot = 0;
            }
            else if(c->occType == FREE || c->occType == UNEXPLORED)
            {
                left = grid->getCell(i-1,j);
                right = grid->getCell(i+1,j);
                down = grid->getCell(i,j-1);
                up = grid->getCell(i,j+1);

                c->pot = (left->pot + right->pot + up->pot + down->pot) / 4;
            }
        }
    }

}

void Planning::updateGradient()
{
    Cell* c;

    // the components of the descent gradient of a cell are stored in:
    // c->dirX and c->dirY

    Cell *left,*right,*up,*down;

    double dirX, dirY;
    double dirX_normalized, dirY_normalized;

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


    for(int i=robotPosition.x-halfWindowSize;i<=robotPosition.x+halfWindowSize;i++)
    {
        for(int j=robotPosition.y-halfWindowSize;j<=robotPosition.y+halfWindowSize;j++)
        {
            c = grid->getCell(i,j);
            if(c->occType == FREE)
            {
                left = grid->getCell(i-1,j);
                right = grid->getCell(i+1,j);
                down = grid->getCell(i,j-1);
                up = grid->getCell(i,j+1);

                dirX = -(right->pot - left->pot) / 2;
                dirY = -(up->pot - down->pot) / 2;

                dirX_normalized = dirX / sqrt(pow(dirX,2) + pow(dirY,2));
                dirY_normalized = dirY / sqrt(pow(dirX,2) + pow(dirY,2));

                c->dirX = dirX_normalized;
                c->dirY = dirY_normalized;
            }
            else
            {
                c->dirX = 0;
                c->dirY = 0;
            }
        }
    }
}
