#include "Robot.h"

#include <unistd.h>
#include <GL/glut.h>
#include <cmath>
#include <iostream>


//////////////////////////////////////
///// CONSTRUCTORS & DESTRUCTORS /////
//////////////////////////////////////

Robot::Robot()
{
    ready_ = false;
    running_ = true;

    grid = new Grid();

    plan = new Planning();
    plan->setGrid(grid);
    plan->setMaxUpdateRange(base.getMaxLaserRange());

    // variables used for navigation
    isFollowingLeftWall_=false;

    // variables used for visualization
    viewMode=0;
    numViewModes=5;
    motionMode_=MANUAL_SIMPLE;

}

Robot::~Robot()
{
    base.closeARIAConnection();
    if(grid!=NULL)
        delete grid;
}

////////////////////////////////////
///// INITIALIZE & RUN METHODS /////
////////////////////////////////////

void Robot::initialize(ConnectionMode cmode, LogMode lmode, std::string fname)
{
    logMode_ = lmode;
//    logFile_ = new LogFile(logMode_,fname);
    ready_ = true;

    // initialize ARIA
    if(logMode_!=PLAYBACK){
        bool success = base.initialize(cmode,lmode,fname);
        if(!success){
            printf("Could not connect to robot... exiting\n");
            exit(0);
        }
    }

    ready_ = true;
    controlTimer.startLap();
}

void Robot::run()
{
    controlTimer.waitTime(0.1);

    if(logMode_==PLAYBACK){
        bool hasEnded = base.readFromLog();
        if(hasEnded){
            std::cout << "PROCESS COMPLETE. CLOSING PROGRAM." << std::endl;
            exit(0);
        }
    }else{
        bool success = base.readOdometryAndSensors();
        if(!success){
            usleep(50000);
            return;
        }

        if(logMode_==RECORDING)
            base.writeOnLog();
    }

    currentPose_ = base.getOdometry();

    pthread_mutex_lock(grid->mutex);

    // Mapping
    mappingWithHIMMUsingLaser();
    mappingWithLogOddsUsingLaser();
    mappingUsingSonar();

    pthread_mutex_unlock(grid->mutex);

    plan->setNewRobotPose(currentPose_);

    // Save path traversed by the robot
    if(base.isMoving() || logMode_==PLAYBACK){
        path_.push_back(base.getOdometry());
    }

    // Navigation
    switch(motionMode_){
        case WANDER:
            wanderAvoidingCollisions();
            break;
        case WALLFOLLOW:
            wallFollow();
            break;
        case POTFIELD:
            followPotentialField();
            break;
        case ENDING:
            running_=false;
            break;
        default:
            break;
    }

    base.resumeMovement();

    usleep(50000);
}

//////////////////////////////
///// NAVIGATION METHODS /////
//////////////////////////////

void Robot::move(MovingDirection dir)
{
    switch(dir){
        case FRONT:
            std::cout << "moving front" << std::endl;
            break;
        case BACK:
            std::cout << "moving back" << std::endl;
            break;
        case LEFT:
            std::cout << "turning left" << std::endl;
            break;
        case RIGHT:
            std::cout << "turning right" << std::endl;
            break;
        case STOP:
            std::cout << "stopping robot" << std::endl;
    }

    if(motionMode_==MANUAL_SIMPLE)
        base.setMovementSimple(dir);
    else if(motionMode_==MANUAL_VEL)
        base.setMovementVel(dir);
    else if(motionMode_=WALLFOLLOW)
        if(dir==LEFT)
            isFollowingLeftWall_=true;
        else if(dir==RIGHT)
            isFollowingLeftWall_=false;
}

void Robot::wanderAvoidingCollisions()
{
    float minLeftSonar  = base.getMinSonarValueInRange(0,2);
    float minFrontSonar = base.getMinSonarValueInRange(3,4);
    float minRightSonar = base.getMinSonarValueInRange(5,7);

    float minLeftLaser  = base.getMinLaserValueInRange(0,74);
    float minFrontLaser = base.getMinLaserValueInRange(75,105);
    float minRightLaser = base.getMinLaserValueInRange(106,180);

    float linVel=0;
    float angVel=0;

    //TODO - implement obstacle avoidance




    base.setWheelsVelocity_fromLinAngVelocity(linVel, angVel);
}

void Robot::wallFollow()
{
    float minLeftSonar  = base.getMinSonarValueInRange(0,2);
    float minFrontSonar = base.getMinSonarValueInRange(3,4);
    float minRightSonar = base.getMinSonarValueInRange(5,7);

    float minLeftLaser  = base.getMinLaserValueInRange(0,74);
    float minFrontLaser = base.getMinLaserValueInRange(75,105);
    float minRightLaser = base.getMinLaserValueInRange(106,180);

    float linVel=0;
    float angVel=0;

    if(isFollowingLeftWall_)
        std::cout << "Following LEFT wall" << std::endl;
    else
        std::cout << "Following RIGHT wall" << std::endl;

    //TODO - implement wall following with a PID controller




    base.setWheelsVelocity_fromLinAngVelocity(linVel, angVel);
}

void Robot::followPotentialField()
{
    int scale = grid->getMapScale();
    int robotX=currentPose_.x*scale;
    int robotY=currentPose_.y*scale;
    float robotAngle = currentPose_.theta;

    Cell* c=grid->getCell(robotX,robotY);

    float linVel, angVel;

    // defining the robot velocities using a control strategy
    // based on the direction of the gradient of c given by c->dirX and c->dirY

    linVel=0.2;
    angVel=0.0;

    float dirX, dirY;
    dirX = c->dirX;
    dirY = c->dirY;

    if(dirX == 0.0 && dirY == 0.0){
        linVel = angVel = 0.0;
    }else{

        float phi = RAD2DEG(atan2(dirY,dirX)) - robotAngle;
        phi = normalizeAngleDEG(phi);

        if(phi<-90.0){
            linVel = 0.0;
            angVel = -0.5;
        }else if(phi>90.0){
            linVel = 0.0;
            angVel = 0.5;
        }else{
            angVel = (phi/90.0)*(linVel*3.0);

        }

    }


    base.setWheelsVelocity_fromLinAngVelocity(linVel,angVel);
}

///////////////////////////
///// MAPPING METHODS /////
///////////////////////////

float Robot::getOccupancyFromLogOdds(float logodds)
{
    return 1.0 - 1.0/(1.0+exp(logodds));
}

void Robot::mappingWithLogOddsUsingLaser()
{
    float alpha = 0.1; //  10 cm
    float beta = 1.0;  // 1.0 degrees

    int scale = grid->getMapScale();// Escala do grid -> indica quantas células correspondem a um metro.
                                    // Por padrão o scale é 10, logo se robô fizer leitura de
                                    // 5m o método deve atualizar uma distância de 50 células.
    float maxRange = base.getMaxLaserRange(); //Valor em metros, multiplicar por scale para obter em células.
    int maxRangeInt = maxRange*scale;

    int robotX=currentPose_.x*scale; // Posição já mapeada para uma célula do grid.
    int robotY=currentPose_.y*scale;
    float robotAngle = currentPose_.theta;  //Teta do robô.

    // TODO: define fixed values of occupancy
    float locc, lfree;

    // how to access a grid cell
    // Cell* c=grid->getCell(robotX,robotY);

    // how to set occupancy of cell
    // c->logodds += lfree;

    // how to convert logodds to occupancy values
    // c->occupancy = getOccupancyFromLogOdds(c->logodds);


    // TODO: update cells in the sensors' field-of-view
    // ============================================================================
    // you only need to check the cells at most maxRangeInt from the robot position
    // that is, in the following square region:
    //
    //  (robotX-maxRangeInt,robotY+maxRangeInt)  -------  (robotX+maxRangeInt,robotY+maxRangeInt)
    //                     |                       \                         |
    //                     |                        \                        |
    //                     |                         \                       |
    //  (robotX-maxRangeInt,robotY-maxRangeInt)  -------  (robotX+maxRangeInt,robotY-maxRangeInt)

    // Defina valores fixos de ocupação para áreas ocupadas (pocc) e áreas livres (pfree)
    float pocc, pfree;
    // 0.0 < pfree < 0.5 < pocc < 1.0
    pocc = 0.75;
    pfree = 0.25;

    // Os valores escolhidos impactarão na velocidade de atualização da ocupação das células. Na prática, serão
    // utilizados os valores em log-odds, isto é:
    locc = log(pocc/(1-pocc));
    lfree = log(pfree/(1-pfree));

    int maxX, maxY, minX, minY = 0;
    minX = robotX - maxRangeInt;
    minY = robotY - maxRangeInt;
    maxX = robotX + maxRangeInt;
    maxY = robotY + maxRangeInt;

    int i, j = 0;
    for(i = minX; i <= maxX; i++)
    {
        for(j = minY; j <= maxY; j++)
        {
            float r;
            float phi;
            int k;
            // Celula a ser analisada.
            Cell* c = grid->getCell(i,j);

            // Computar a distância r até a célula onde está o robô.
            //             (xi - x)^2         (yi - y)^2
            r = sqrt((pow(i - robotX, 2) + pow(j - robotY, 2)));
            // Dividí-la por scale para convertê-la para metros e poder compará-la com as medidas dos sensores.
            r = r/scale;

            // Computar a orientação φ da célula em relação ao robô em coordenadas locais.
            phi = RAD2DEG(atan2(j - robotY, i - robotX)) - robotAngle;
            // OBS: Todos os ângulos computados devem estar devidamente normalizados (entre −180◦ e 180◦).
            // Para auxiliar use a função: phi = normalizeAngleDEG(phi);
            phi = normalizeAngleDEG(phi);

            // Encontrar a medida do sensor k mais próxima da orientação da célula em relação ao robô.
            // base.getNearestSonarBeam(phi) ou base.getNearestLaserBeam(phi),
            // retornam o índice da medida mais próxima do ângulo phi.
            k = base.getNearestLaserBeam(phi);

            // Atualizar a ocupação da célula como ocupada ou livre dependendo da região do sensor em que se
            // enquadrar.
            // Para isso deve-se testar se a célula está dentro da abertura do campo-de-visão do sensor
            // através da diferença entre a orientação φ da célula e a orientação da medida k, dada pelas funções
            // base.getAngleOfSonarBeam(k) ou base.getAngleOfLaserBeam(k). E também
            // verificar se adistância r é próxima ou menor da medida do sensor k, dada pelas funções
            // base.getKthSonarReading(k) ou base.getKthLaserReading(k).
            if((fabs(phi - base.getAngleOfLaserBeam(k)) > beta/2) ||(r > std::min(maxRange, (base.getKthLaserReading(k)+(alpha/2)))))
            {
               c->logodds += 0;
            }

            else if((base.getKthLaserReading(k) < maxRange) && (fabs(r - base.getKthLaserReading(k))< alpha/2))
            {
               c->logodds += locc;
            }

            else if(r <= base.getKthLaserReading(k))
            {
               c->logodds += lfree;
            }

            c->occupancy = getOccupancyFromLogOdds(c->logodds);
        }
    }
}

void Robot::mappingUsingSonar()
{
    // TODO: update cells in the sensors' field-of-view
    // Follow the example in mappingWithLogOddsUsingLaser()



}

void Robot::mappingWithHIMMUsingLaser()
{
    // TODO: update cells in the sensors' field-of-view
    // Follow the example in mappingWithLogOddsUsingLaser()



}

/////////////////////////////////////////////////////
////// METHODS FOR READING & WRITING ON LOGFILE /////
/////////////////////////////////////////////////////

// Prints to file the data that we would normally be getting from sensors, such as the laser and the odometry.
// This allows us to later play back the exact run.
void Robot::writeOnLog()
{
    logFile_->writePose("Odometry",currentPose_);
    logFile_->writeSensors("Sonar",base.getSonarReadings());
    logFile_->writeSensors("Laser",base.getLaserReadings());
}

// Reads back into the sensor data structures the raw readings that were stored to file
// While there is still information in the file, it will return 0. When it reaches the end of the file, it will return 1.
bool Robot::readFromLog() {

    if(logFile_->hasEnded())
        return true;

    base.setOdometry(logFile_->readPose("Odometry"));
    base.setSonarReadings(logFile_->readSensors("Sonar"));
    base.setLaserReadings(logFile_->readSensors("Laser"));

    return false;
}

////////////////////////
///// DRAW METHODS /////
////////////////////////

void Robot::draw(float xRobot, float yRobot, float angRobot)
{
    float scale = grid->getMapScale();
    glTranslatef(xRobot,yRobot,0.0);

    drawPotGradient(scale);

    glRotatef(angRobot,0.0,0.0,1.0);
    glScalef(1.0/scale,1.0/scale,1.0/scale);

    // sonars and lasers draw in cm
    if(viewMode==1)
        base.drawSonars(true);
    else if(viewMode==2)
        base.drawSonars(false);
    else if(viewMode==3)
        base.drawLasers(true);
    else if(viewMode==4)
        base.drawLasers(false);

    // robot draw in cm
    base.drawBase();

    glScalef(scale,scale,scale);
    glRotatef(-angRobot,0.0,0.0,1.0);
    glTranslatef(-xRobot,-yRobot,0.0);
}

void Robot::drawPotGradient(double scale)
{
    Cell* c;
    int robotX=currentPose_.x*scale;
    int robotY=currentPose_.y*scale;
    c = grid->getCell(robotX,robotY);

    glColor3f(0.0,0.6,0.2);
    glLineWidth(3);
    glBegin( GL_LINE_STRIP );
    {
        glVertex2f(0, 0);
        glVertex2f(5*c->dirX, 5*c->dirY);
    }
    glEnd();
}

/////////////////////////
///// OTHER METHODS /////
/////////////////////////

bool Robot::isReady()
{
    return ready_;
}

bool Robot::isRunning()
{
    return running_;
}

const Pose& Robot::getCurrentPose()
{
    return currentPose_;
}

void Robot::drawPath()
{
    float scale = grid->getMapScale();

    if(path_.size() > 1){
        glScalef(scale,scale,scale);
        glLineWidth(3);
        glBegin( GL_LINE_STRIP );
        {
            for(unsigned int i=0;i<path_.size()-1; i++){
                glColor3f(1.0,0.0,1.0);

                glVertex2f(path_[i].x, path_[i].y);
                glVertex2f(path_[i+1].x, path_[i+1].y);
            }
        }
        glEnd();
        glLineWidth(1);
        glScalef(1.0/scale,1.0/scale,1.0/scale);

    }
}

void Robot::waitTime(float t){
    float l;
    do{
        usleep(1000);
        l = controlTimer.getLapTime();
    }while(l < t);
    controlTimer.startLap();
}
