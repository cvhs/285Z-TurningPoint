#include "robot-config.h"

void driveLeft(){
    int scaledPower {Controller.Axis3.value()};
    if (abs(scaledPower) < 5)
        scaledPower = 0;
    else
        scaledPower = ( (scaledPower^3/100) * scaledPower ) / 100;
    
    if (Controller.Axis3.value() < 0)
        scaledPower = scaledPower * -1;
    else
        scaledPower = scaledPower *  1;
    leftDriveFront.spin(directionType::fwd, scaledPower, velocityUnits::pct);
    leftDriveBack.spin(directionType::fwd, scaledPower, velocityUnits::pct);
}

void driveRight(){
    int scaledPower {Controller.Axis2.value()};
    if (abs(scaledPower) < 5)
        scaledPower = 0;
    else
        scaledPower = ( (scaledPower^3/100) * scaledPower ) / 100;
    
    if (Controller.Axis2.value() < 0)
        scaledPower = scaledPower * -1;
    else
        scaledPower = scaledPower *  1;
    rightDriveFront.spin(directionType::fwd, scaledPower, velocityUnits::pct);
    rightDriveBack.spin(directionType::fwd, scaledPower, velocityUnits::pct);
}

static bool reverse {false};
static bool toggle {false};
static int mode {1}; 

void toggleIntake()
{
    toggle = !toggle;

    if (toggle)
        ballIntake.spin(directionType::fwd, 100, velocityUnits::pct);
    else
        ballIntake.spin(directionType::fwd, 0, velocityUnits::pct);
}

void puIntake()
    {ballIntake.spin(directionType::fwd, 100, velocityUnits::pct);}
void pdIntake()
{
    ballIntake.spin(directionType::fwd, 0, velocityUnits::pct);
    reverse = false;
}
void rIntake()
{
    ballIntake.spin(directionType::fwd, -100, velocityUnits::pct);
    reverse = true;
}

void puIndexer()
    {indexer.spin(directionType::fwd, 100, velocityUnits::pct);}
void pdIndexer()
    {indexer.spin(directionType::fwd, 0, velocityUnits::pct);}
void rIndexer()
    {indexer.spin(directionType::fwd, -100, velocityUnits::pct);}

int main() {
    
    Controller.ButtonR1.pressed(puIndexer);   //> indexer 
    Controller.ButtonR1.released(pdIndexer);

    Controller.ButtonR2.pressed(toggleIntake);      //> intake toggle

   
    Controller.ButtonL2.pressed(rIntake);     //> reverse
    Controller.ButtonL2.pressed(rIndexer);

    Controller.ButtonL2.released(pdIntake);
    Controller.ButtonL2.released(pdIndexer);
    
    while(1){
        //flywheel.spin(directionType::fwd, 170, velocityUnits::rpm);
        
        driveLeft();
        driveRight();
        
    }
/************* Ball Intake & Indexer *************/
    
    
}
