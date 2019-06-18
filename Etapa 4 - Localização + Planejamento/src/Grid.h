#ifndef __GRID_H__
#define __GRID_H__

#include <pthread.h>

enum CellOccType {OCCUPIED, UNEXPLORED, FREE, NEAROBSTACLE};
enum CellPlanType {REGULAR, FRONTIER, MARKED_FRONTIER, PATH, LOCALGOAL};

#define UNDEF -10000000

class Cell
{
    public:
        int x,y;        
        int himm;
        double logodds,occupancy;
        double logoddsSonar,occupancySonar;
        double distWalls, dirX, dirY;

        // used in A-Star
        double f, g, h;
        Cell* pi;

        double pot;

        CellOccType occType;
        CellPlanType planType;
};

class Grid
{
    public:
        Grid();
        Cell* getCell(int x, int y);

        int getMapScale();
        int getMapWidth();
        int getMapHeight();

        void draw(int xi, int yi, int xf, int yf);

        int numViewModes;
        int viewMode;
        bool showValues;
        bool showArrows;

        pthread_mutex_t* mutex;

    private:
        int mapScale_; // Number of cells per meter
        int mapWidth_, mapHeight_; // in cells
        int numCellsInRow_, halfNumCellsInRow_;

        Cell* cells_;

        void drawCell(unsigned int i);
        void drawVector(unsigned int i);
        void drawText(unsigned int n);
};

#endif // __GRID_H__
