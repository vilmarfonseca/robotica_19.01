#include "MCL.h"

#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
//#include <cmath>

#include <GL/glut.h>

MCL::MCL(float maxRange, std::string mapName, pthread_mutex_t* m):
    maxRange(maxRange), mutex(m)
{
    // construct a trivial random generator engine from a time-based seed:
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    generator = new std::default_random_engine(seed);

    readMap(mapName);
    scale = 10;
    transparency = false;

    numParticles = 1000;

    initParticles();
}

void MCL::run(const Action &u, const std::vector<float> &z)
{
    sampling(u);
    weighting(z);
    resampling();

    updateMeanAndCovariance();
}


//////////////////////////////////////////////////
//// Métodos SAMPLING, WEIGHTING e RESAMPLING ////
//////////////////////////////////////////////////

void MCL::sampling(const Action &u)
{
    /// Odometria definida pela estrutura Action, composta por 3 variaveis double:
    /// rot1, trans e rot2
    std::cout << "rot1 " << RAD2DEG(u.rot1) << " trans " << u.trans << " rot2 " << RAD2DEG(u.rot2) << std::endl;

    /// Seguindo o modelo de Thrun, devemos gerar 3 distribuicoes normais, uma para cada componente da odometria

    /// Para definir uma distribuição normal X de media M e variancia V, pode-se usar:
    std::normal_distribution<double> normalDistRot1(0,((0.01*u.rot1) + (0.01*u.trans)));
    std::normal_distribution<double> normalDistTrans(0,((0.01*u.trans) + (0.01*u.rot2)));
    std::normal_distribution<double> normalDistRot2(0,((0.01*u.rot2) + (0.01*u.rot1)));
    /// Para gerar amostras segundo a distribuicao acima, usa-se:
    double amostraRot1 = 0.0;
    double amostraTrans = 0.0;
    double amostraRot2 = 0.0;

    /// onde *generator é um gerador de numeros aleatorios (definido no construtor da classe)
    for(int i = 0; i < numParticles; i++)
    {
        amostraRot1 = u.rot1 - normalDistRot1(*generator);
        amostraTrans = u.trans - normalDistTrans(*generator);
        amostraRot2 = u.rot2 - normalDistRot2(*generator);
        particles[i].p.x = particles[i].p.x + amostraTrans*cos(particles[i].p.theta + amostraRot1);
        particles[i].p.y = particles[i].p.y + amostraTrans*sin(particles[i].p.theta + amostraRot1);
        particles[i].p.theta = particles[i].p.theta + (amostraRot1 + amostraRot2);
    }
}

void MCL::weighting(const std::vector<float> &z)
{

    float kParticleObservation = 0.0;
    float kRobotObservation = 0.0;
    float individualProb = 0.0;
    float totalProb = 1.0;
    float var = 0.1;
    float totalWeight = 0.0;
    float normalizedWeight = 0.0;

    for(int i = 0; i < numParticles; i++)
    {
//        sleep(2);
//        std::cout << "Partícula: " << i << std::endl;
        individualProb = 0.0;
        totalProb = 1.0;
        totalWeight = 0.0;
        normalizedWeight = 0.0;
        int count = 0;
         /// 1: elimine particulas fora do espaco livre
        if(!(mapCells[(int)(particles[i].p.x*scale)][(int)(particles[i].p.y*scale)] == FREE))
        {
            particles[i].w = 0;
        }
        else
        {
            /// 2: compare as observacoes da particula com as observacoes z do robo
            // Use a funcao computeExpectedMeasurement(k, particles[i].p)
            // para achar a k-th observacao esperada da particula i
            for(int k = 0; k < 180; k+=15)
            {
                count++;
//                std::cout << "count: " << count << std::endl;

                // A probabilidade final associada à particula p pode ser aproximada pelo produto das probabilidades individuais.
                kRobotObservation = z[k];
                kParticleObservation = computeExpectedMeasurement(k, particles[i].p);
                // A probabilidade de uma medição individual pode ser definida de acordo com o modelo visto em aula.
                individualProb = (1 / (sqrt(2 * M_PI * var))) * exp((-1/2)*(pow((kRobotObservation - kParticleObservation),2)/var));
//                std::cout << "indivProb: " << individualProb << std::endl;
                totalProb = totalProb + (totalProb * individualProb);
//                std::cout << "totalProb: " << totalProb << std::endl;
            }
            particles[i].w = totalProb;
        }
        /// 3: normalize os pesos
        totalWeight += particles[i].w;
//        std::cout << "totalWeight: " << totalWeight << std::endl;
    }

    if(totalWeight != 0)
    {
        normalizedWeight = totalWeight / numParticles;
//        std::cout << "[IF-TOTALWEIGHT!=0]normalizedWeight " << normalizedWeight << std::endl;
    }
    else
    {
//        std::cout << "Variância muito pequena!!!" << std::endl;
        normalizedWeight = 1 / numParticles;
//        std::cout << "normalizedWeight " << normalizedWeight << std::endl;
    }

    for(int i = 0; i < numParticles; i++)
    {
//        std::cout << "Normalizando partícula " << i << std::endl;
        particles[i].w = normalizedWeight;
    }

}

void MCL::resampling()
{
    // gere uma nova geração de particulas com o mesmo tamanho do conjunto atual
    std::vector<MCLparticle> nextGeneration;
    nextGeneration.resize(numParticles);

    /// TODO: Implemente o Low Variance Resampling
    //https://github.com/JuliaStats/StatsBase.jl/issues/124

    /// Para gerar amostras de uma distribuição uniforme entre valores MIN e MAX, pode-se usar:
    std::uniform_real_distribution<double> samplerU(0,1/numParticles);
    /// Para gerar amostras segundo a distribuicao acima, usa-se:
    double r = samplerU(*generator);
    double c = particles[1].w;
    double u = 0.0;
    int i = 1;

    for(int j = 1; j <= numParticles; j++)
    {
        u = 0.0;
        r = samplerU(*generator); //Não sei se tem q gerar um r novo pra cada partícula ou não. No algoritmo parece que não.
        u = r + (1/numParticles)*(j - 1);
        while(u > c)
        {
            i++;
            c += particles[i].w;
        }
        nextGeneration[j-1] = particles[i-1];
    }
    particles = nextGeneration;
    /// onde *generator é um gerador de numeros aleatorios (definido no construtor da classe)
}

/////////////////////////////////////////////////////
//// Método Auxiliar para o Modelo de Observacao ////
/////////////////////////////////////////////////////

float MCL::computeExpectedMeasurement(int index, Pose &pose)
{
    double angle = pose.theta + double(90-index)*M_PI/180.0;

    // Ray-casting using DDA
    double dist;
    double difX=cos(angle);
    double difY=sin(angle);
    double deltaX, deltaY;

    if(tan(angle)==1 || tan(angle)==-1){
        deltaX=deltaY=1.0;
        dist = difX*maxRange;
    }else if(difX*difX > difY*difY){
        deltaX=1.0;
        deltaY=difY/difX;
        dist = difX*maxRange;
    }else{
        deltaX=difX/difY;
        deltaY=1.0;
        dist = difY*maxRange;
    }
    if(deltaX*difX < 0.0)
        deltaX = -deltaX;
    if(deltaY*difY < 0.0)
        deltaY = -deltaY;
    if(dist < 0.0)
        dist = -dist;

    dist *= scale;

    double i=pose.x*scale;
    double j=pose.y*scale;
    for(int k=0;k<(int)(dist);k++){

        if(mapCells[(int)i][(int)j] == OCCUPIED){
            // the real obstacle is one step ahead due to wall thickening
            return sqrt(pow(pose.x*scale-(i+deltaX),2)+pow(pose.y*scale-(j+deltaY),2))/scale;
        }

        i+=deltaX;
        j+=deltaY;
    }

    return maxRange;
}

//////////////////////////////////
//// Métodos de Inicializacao ////
//////////////////////////////////

void MCL::readMap(std::string mapName)
{
    std::string name("/home/nicholas/UFRGS/robotica_19.01/Etapa 4 - Localização + Planejamento/DiscreteMaps/");
    name += mapName;
    std::ifstream file;
    file.open(name.c_str(), std::ifstream::in);

    if( !file.good() )
    {
        std::cerr << "The file \"" << name << "\"  does not exit!" << std::endl;
        return;
    }

    // Read goal pose
    file >> goal.x >> goal.y;
    std::cout << "Goal x " << goal.x << " y " << goal.y << std::endl;

    // Read dimensions.
    file >> mapWidth >> mapHeight;
    std::cout << "map.width " << mapWidth << " map.height " << mapHeight << std::endl;

    mapCells = new CellOccType*[mapWidth];
        for(int i=0;i<mapWidth;i++)
            mapCells[i] = new CellOccType[mapHeight];

    // Read grid from file.
    char read;
    for(int y=0; y < mapHeight; y++)
    {
        for(int x=0; x < mapWidth; x++)
        {
            file >> read;
            switch(read)
            {
                case '1':
                    mapCells[x][y] = OCCUPIED;
                    break;
                case '0':
                    mapCells[x][y] = FREE;
                    break;
                case '-':
                    mapCells[x][y] = UNEXPLORED;
                    break;
            }
        }
    }

    file.close();
}

void MCL::initParticles()
{
    particles.resize(numParticles);

    std::uniform_real_distribution<double> randomX(0.0,mapWidth/scale);
    std::uniform_real_distribution<double> randomY(0.0,mapHeight/scale);
    std::uniform_real_distribution<double> randomTh(-M_PI,M_PI);

    // generate initial set
    for(int i=0; i<numParticles; i++){

        bool valid = false;
        do{
            // sample particle pose
            particles[i].p.x = randomX(*generator);
            particles[i].p.y = randomY(*generator);
            particles[i].p.theta = randomTh(*generator);

            // check if particle is valid (known and not obstacle)
            if(mapCells[(int)(particles[i].p.x*scale)][(int)(particles[i].p.y*scale)] == FREE)
                valid = true;

        }while(!valid);

        std::cout << "Particle (" << i << "): "
                  << particles[i].p.x << ' '
                  << particles[i].p.y << ' '
                  << RAD2DEG(particles[i].p.theta) << std::endl;
    }
}

//////////////////////////////////////////////////////
//// Método de Atualização da Media e Covariancia ////
//////////////////////////////////////////////////////

void MCL::updateMeanAndCovariance()
{
    // Compute Mean
    float sx=0, cx=0;
    meanParticlePose.x = meanParticlePose.y = 0.0;
    for(unsigned int i=0; i<numParticles; i++){
        meanParticlePose.x += particles[i].p.x;
        meanParticlePose.y += particles[i].p.y;
        sx += sin(particles[i].p.theta);
        cx += cos(particles[i].p.theta);
    }
    meanParticlePose.x /= numParticles;
    meanParticlePose.y /= numParticles;
    meanParticlePose.theta = atan2(sx,cx);

    // Compute Covariance Matrix 2x2 (considering only x and y)
    float covariance[2][2];
    for(unsigned int l=0; l<2; l++)
        for(unsigned int c=0; c<2; c++)
            covariance[l][c] = 0;

    float diffx, diffy;
    for(unsigned int i=0; i<numParticles; i++){
        diffx  = meanParticlePose.x-particles[i].p.x;
        diffy  = meanParticlePose.y-particles[i].p.y;

        covariance[0][0] += diffx*diffx;    covariance[0][1] += diffx*diffy;
        covariance[1][0] += diffy*diffx;    covariance[1][1] += diffy*diffy;
    }

    for(unsigned int l=0; l<2; l++)
        for(unsigned int c=0; c<2; c++)
            covariance[l][c] /= numParticles;

    // Compute EigenValues and EigenVectors of covariance matrix
    float T = covariance[0][0] + covariance[1][1]; // Trace
    float D = covariance[0][0]*covariance[1][1] - covariance[0][1]*covariance[1][0]; // Determinant

    if((pow(T,2.0)/4.0 - D)<0.0)
        return;

//    std::cout << "Covariance [" << covariance[0][0] << " " << covariance[0][1]
//                        << "; " << covariance[1][0] << " " << covariance[1][1] << std::endl;

    float lambda1 = T/2.0 + sqrt(pow(T,2.0)/4.0 - D);
    float lambda2 = T/2.0 - sqrt(pow(T,2.0)/4.0 - D);
    float eigvec1[2], eigvec2[2];

    if(covariance[1][0]!=0.0){
        eigvec1[0] = lambda1 - covariance[1][1];    eigvec2[0] = lambda2 - covariance[1][1];
        eigvec1[1] = covariance[1][0];              eigvec2[1] = covariance[1][0];
    }else if(covariance[0][1]!=0.0){
        eigvec1[0] = covariance[0][1];              eigvec2[0] = covariance[0][1];
        eigvec1[1] = lambda1 - covariance[0][0];    eigvec2[1] = lambda2 - covariance[0][0];
    }else if(covariance[1][0]==0.0 && covariance[0][1]==0.0){
        eigvec1[0] = 1;    eigvec2[0] = 0;
        eigvec1[1] = 0;    eigvec2[1] = 1;
    }

//    std::cout << "lambda " << lambda1 << " and " << lambda2 << std::endl;
//    std::cout << "eigvectors [" << eigvec1[0] << "; " << eigvec1[1]
//              << "] and [" << eigvec1[0] << "; " << eigvec1[1] << "]" << std::endl;

    // Compute direction of covariance ellipse
    //1st - Calculate the angle between the largest eigenvector and the x-axis
    covAngle = RAD2DEG(atan2(eigvec1[1], eigvec1[0]));

    //2nd - Calculate the size of the minor and major axes
    covMajorAxis = sqrt(lambda1);
    covMinorAxis = sqrt(lambda2);
//    std::cout << "covAngle " << covAngle << " covMajorAxis " << covMajorAxis << " covMinorAxis " << covMinorAxis << std::endl;
}

////////////////////////////
//// Métodos de desenho ////
////////////////////////////

void Ellipse(float rx, float ry, float angle, int num_segments=80)
{
    float theta = 2 * M_PI / float(num_segments);
    float c = cos(theta);//precalculate the sine and cosine
    float s = sin(theta);
    float t;

    float x = 1;//we start at angle = 0
    float y = 0;

    glRotatef(angle,0,0,1);
    glBegin(GL_LINE_LOOP);
    for(int ii = 0; ii < num_segments; ii++)
    {
        glVertex2f(x*rx, y*ry);//output vertex

        //apply the rotation matrix
        t = x;
        x = c * x - s * y;
        y = s * t + c * y;
    }
    glEnd();
    glRotatef(-angle,0,0,1);
}

void MCL::draw()
{
    // Draw map
    for(int x=0;x<mapWidth;x++){
        for(int y=0;y<mapHeight;y++){

            if(mapCells[x][y] == OCCUPIED)
                glColor3f(0.0,0.0,0.0);
            else if (mapCells[x][y] == UNEXPLORED)
                glColor3f(0.5,0.5,0.5);
            else
                glColor3f(1.0,1.0,1.0);

            glBegin( GL_QUADS );
            {
                glVertex2f(x  ,y  );
                glVertex2f(x+1,y  );
                glVertex2f(x+1,y+1);
                glVertex2f(x  ,y+1);
            }
            glEnd();
        }
    }

    double dirScale=5;
    glPointSize(4);
    glLineWidth(2);

    float alpha;
    if(transparency)
        alpha = 100.0/numParticles;
    else
        alpha = 1.0;

    // Draw particles
    for(int p=0;p<particles.size();p++){

        double x=particles[p].p.x*scale;
        double y=particles[p].p.y*scale;
        double th=particles[p].p.theta;

        // Draw point
        glColor4f(1.0,0.0,0.0,alpha);
        glBegin( GL_POINTS );
        {
            glVertex2f(x, y);
        }
        glEnd();

        // Draw direction
        glColor4f(0.0, 0.0, 1.0, alpha);
        glBegin( GL_LINES );
        {
            glVertex2f(x, y);
            glVertex2f(x+dirScale*cos(th), y+dirScale*sin(th));
        }
        glEnd();
    }
    glLineWidth(1);

    // Draw RED X at goal pose
    double xGoal = goal.x*scale;
    double yGoal = goal.y*scale;
    glTranslatef(xGoal,yGoal,0.0);
    glScalef(1.0/2.0,1.0/2.0,1.0/2.0);
    glColor3f(1.0,0.0,0.0);
    glBegin( GL_POLYGON );
    {
        glVertex2f(-12, -8);
        glVertex2f(  8, 12);
        glVertex2f( 12,  8);
        glVertex2f( -8,-12);
    }
    glEnd();
    glBegin( GL_POLYGON );
    {
        glVertex2f(-12,  8);
        glVertex2f( -8, 12);
        glVertex2f( 12, -8);
        glVertex2f(  8,-12);
    }
    glEnd();
    glScalef(2,2,2);
    glTranslatef(-xGoal,-yGoal,0.0);

    // Draw mean particle pose
    double xRobot = meanParticlePose.x*scale;
    double yRobot = meanParticlePose.y*scale;
    double angRobot = RAD2DEG(meanParticlePose.theta);

    glTranslatef(xRobot,yRobot,0.0);
    glRotatef(angRobot,0.0,0.0,1.0);
    glScalef(1.0/5.0,1.0/5.0,1.0/5.0);
    glColor3f(0.0,1.0,0.0);
    glBegin( GL_POLYGON );
    {
        glVertex2f(-20, -8);
        glVertex2f(-13, -15);
        glVertex2f(8, -15);
        glVertex2f(15, -8);
        glVertex2f(15, 8);
        glVertex2f(8, 15);
        glVertex2f(-13, 15);
        glVertex2f(-20, 8);
    }
    glEnd();
    glColor3f(0.0,0.0,0.0);
    glBegin( GL_LINE_STRIP );
    {
        glVertex2f(0, 0);
        glVertex2f(30, 0);
    }
    glEnd();
    glScalef(5,5,5);
    glRotatef(-angRobot,0.0,0.0,1.0);
    glTranslatef(-xRobot,-yRobot,0.0);

    // Draw Covariance Ellipse
    glColor3f(0.0,0.4,0.0);
    glLineWidth(2);
    double chisquare_val = 2.4477; // 95% confidence interval
    glTranslatef(xRobot,yRobot,0.0);
    Ellipse(chisquare_val*covMajorAxis*scale, chisquare_val*covMinorAxis*scale,covAngle);
    glTranslatef(-xRobot,-yRobot,0.0);
    glLineWidth(1);
}
