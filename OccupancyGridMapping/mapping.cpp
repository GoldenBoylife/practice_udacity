#include <iostream>
#include <math.h>
#include <vector>
using namespace std;


double Zmax = 5000, Zmin = 170;
//beam의 최대 최소 거리

double l0 = 0, locc = 0.4, lfree = -0.4;
//l은 cell 한개의 state값을 나타냄.
//3가지의 state 값이 존재: free, occupied, unknown


double gridWidth = 100, gridHeight = 100;
//grid 가로 세로


double mapWidth = 30000, mapHeight = 15000;
//map 가로 세로


double robotXOffset = mapWidth / 5, robotYOffset = mapHeight / 3;
//map크기에 따른 robot의 X사이즈는 6000, Y사이즈는 50000

// Defining an l vector to store the log odds values of each cell
vector< vector<double> > l(mapWidth/gridWidth, vector<double>(mapHeight/gridHeight));
//??각셀마다 1vector를 log odds값으로 저장한다

double inverseSensorModel(double x, double y, double theta, double xi, double yi, double sensorData[])
{
    //******************Code the Inverse Sensor Model Algorithm**********************//
    // Defining Sensor Characteristics
    double Zk, thetaK, sensorTheta;
    double minDelta = -1;
    double alpha = 200, beta = 20;
    //

    //******************Compute r and phi**********************//
    double r = sqrt(pow(xi - x, 2) + pow(yi - y, 2));
    //로봇의 위치와 mi사이의 거리를 측정함
    //이 mi가 Zmax(측정가능 최대값)보다 너머에 있으면 알수 없는 값이므로, unknown값이다. 

    double phi = atan2(yi - y, xi - x) - theta; 
    //?? 로봇 프레임 기준의 랜드마크 i 방향각(x축으로부터 회전한 각도) 라고 들었음. 
    //로봇pose의 앵글값과 mi중심점(xi,yi)사이의 각도값
    

    //Scaling Measurement to [-90 -37.5 -22.5 -7.5 7.5 22.5 37.5 90]
    //??센서의Theta가 -90도에서 90까지 회전하는 것인가??
    for (int i = 0; i < 8; i++) {
        if (i == 0) {
            sensorTheta = -90 * (M_PI / 180);
        }
        else if (i == 1) {
            sensorTheta = -37.5 * (M_PI / 180);
        }
        else if (i == 6) {
            sensorTheta = 37.5 * (M_PI / 180);
        }
        else if (i == 7) {
            sensorTheta = 90 * (M_PI / 180);
        }
        else {
            sensorTheta = (-37.5 + (i - 1) * 15) * (M_PI / 180);
        }

        /*??무슨 용도지?*/
        if (fabs(phi - sensorTheta) < minDelta || minDelta == -1) {
            //fabs  절대값 나옴. 
            Zk = sensorData[i];
            thetaK = sensorTheta;
            minDelta = fabs(phi - sensorTheta);
            //fabs  절대값 나옴. 
        }
    }

    //******************Evaluate the three cases**********************//
    /*
        l0의 조건
        1. mi의 중심점과 로봇위치점 사이의 거리인 r보다 더 큰 Z(측정치)니까 바깥 mi에 대한 것이다.
        혹은 2. 센서의측정각도인 beta를 넘어가는 각.
     */
    if (r > min((double)Zmax, Zk + alpha / 2) || fabs(phi - thetaK) > beta / 2 || Zk > Zmax || Zk < Zmin) {
        return l0;
    }
    else if (Zk < Zmax && fabs(r - Zk) < alpha / 2) {
        return locc;
    }
    else if (r <= Zk) {
        return lfree;
    }
}

void occupancyGridMapping(double Robotx, double Roboty, double Robottheta, double sensorData[])
{
    //******************Code the Occupancy Grid Mapping Algorithm**********************//
    //mapWidth/gridWidth는 맵가로의 grid갯수
    //mapHeight/gridHeight는 맵세로의grid갯수
    for (int x = 0; x < mapWidth / gridWidth; x++) {
        for (int y = 0; y < mapHeight / gridHeight; y++) {
            double xi = x * gridWidth + gridWidth / 2 - robotXOffset;
            //현재 로봇 위치에서 빔이 닿은 mi의 중심점의 x
            double yi = -(y * gridHeight + gridHeight / 2) + robotYOffset;
            //현재 로봇 위치에서 빔이 닿은 mi의 중심점의 y

            /*로봇의 현재 위치와 빔이 닿은 위치값과의 거리가 빔의 최대거리보다 작거나 같으면,*/
            if (sqrt(pow(xi - Robotx, 2) + pow(yi - Roboty, 2)) <= Zmax) {
                l[x][y] = l[x][y] + inverseSensorModel(Robotx, Roboty, Robottheta, xi, yi, sensorData) - l0;
                //inverseSensorModel은 현재 로봇의 pose와 mi의 중심점(xi,yi)와 센서 데이터 값 8개가 들어간다. 
                //그리고 출력으로 3가지 중에서 하나가 나온다.
                //각 l0 = 0, locc = 0.4, lfree = -0.4; 
                //mi의 상태값에 따라서 맵이 갱신됨. 
                //l0는 unknown이다.


            }
        }
    }
}

int main()
{
    double timeStamp;
    double measurementData[8];
    double robotX, robotY, robotTheta;

    FILE* posesFile = fopen("poses.txt", "r");
    FILE* measurementFile = fopen("measurement.txt", "r");

    // Scanning the files and retrieving measurement and poses at each timestamp
    while (fscanf(posesFile, "%lf %lf %lf %lf", &timeStamp, &robotX, &robotY, &robotTheta) != EOF) {
        //poses.txt파일은 시간, x,y,세타값으로 이루어져 있다. 
        /*measurement.txt파일은 시간 , 측정값8개로 이루어져 있다. */
        fscanf(measurementFile, "%lf", &timeStamp);
        for (int i = 0; i < 8; i++) {
            fscanf(measurementFile, "%lf", &measurementData[i]);
        }
        occupancyGridMapping(robotX, robotY, (robotTheta / 10) * (M_PI / 180), measurementData);
        //같은 timestamp에 pose값과 측정값 8개를 넣고 occupancyGridMapping한다. 
    }

    /*모든 데이터 읽은 다음에  맵핑해서 visual화 한다.*/
    
    for (int x = 0; x < mapWidth / gridWidth; x++) {
        for (int y = 0; y < mapHeight / gridHeight; y++) {
            cout << l[x][y] << " ";
        }
    }
    
    return 0;
}
