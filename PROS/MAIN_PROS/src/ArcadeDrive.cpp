#include "main.h"
#include "ArcadeDrive.h"
#include <algorithm>
using namespace std;

double* getVelocitys(double x, double y, double deadzoneX, double deadzoneY){
    if (x < deadzoneX && x > -deadzoneX)
        x = 0;
    if (y < deadzoneY && y > -deadzoneY)
        y = 0;
    double LeftDrive = 0;
    double RightDrive = 0;
    if (!(y == 0 && x == 0)){
        double maximum = max(abs(x), abs(y));
        double total = y + x;
        double difference = y - x;
        bool Left = true;
        bool Up = true;
        if (x >= 0){
            Left = false;
        }
        else Left = true;
        if (y >= 0){
            Up = true;
        }else Up = false;

        if (Up && !Left){
            LeftDrive = maximum;
            RightDrive = difference;
        }
        else if (!Up && !Left) 
        {
            LeftDrive = total;
            RightDrive = maximum;
        }else if (!Up && Left){
            LeftDrive = total;
            RightDrive = -maximum;
        }else{
            LeftDrive = -maximum;
            RightDrive = difference;
        }
    }
    double Output[2] = {LeftDrive, RightDrive};
    return Output;
    
    
}

