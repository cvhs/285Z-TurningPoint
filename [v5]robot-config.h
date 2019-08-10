using namespace vex;
    
// PARTS INITIALIZATION //

brain Brain;
controller Controller = controller();

motor leftDriveFront(PORT1, false);
motor leftDriveBack(PORT2, false);
motor rightDriveFront(PORT3, true);
motor rightDriveBack(PORT4, true);

motor flywheel(PORT5, false);
motor ballIntake(PORT6, true);
motor indexer(PORT7, true);

motor arm(PORT8, true);

line aLine = line(Brain.ThreeWirePort.A);
line bLine = line(Brain.ThreeWirePort.B);

gyro Gyro = gyro(Brain.ThreeWirePort.C);


// HANDLER FUNCTIONS //

int printShit(){ 
    static int leftAvg {0};
    static int rightAvg {0};
    static int rot {0};
    
    while(1){
        leftAvg = (leftDriveFront.rotation(rotationUnits::deg) + leftDriveBack.rotation(rotationUnits::deg)) / 2;
        rightAvg = (rightDriveFront.rotation(rotationUnits::deg) + rightDriveBack.rotation(rotationUnits::deg)) / 2;
        rot = Gyro.value(vex::rotationUnits::deg);
        Brain.Screen.printAt(10, 30, "%d", leftAvg);
        Brain.Screen.printAt(10, 60, "%d", rightAvg);
        Brain.Screen.printAt(10, 90, "%d", rot);
    }
    
    return 0;
}

void printShitAlso(){ 
    static int leftAvg {0};
    static int rightAvg {0};
    static int rot {0};
    
    while(1){
        leftAvg = (leftDriveFront.rotation(rotationUnits::deg) + leftDriveBack.rotation(rotationUnits::deg)) / 2;
        rightAvg = (rightDriveFront.rotation(rotationUnits::deg) + rightDriveBack.rotation(rotationUnits::deg)) / 2;
        rot = Gyro.value(vex::rotationUnits::deg);
        Brain.Screen.printAt(10, 30, "%d", leftAvg);
        Brain.Screen.printAt(10, 60, "%d", rightAvg);
        Brain.Screen.printAt(10, 90, "%d", rot);
    }
}

bool brakeToggle = false;

void toggleBrake(){
    brakeToggle = !brakeToggle;
    if(brakeToggle){
        leftDriveFront.setStopping(brakeType::brake);
        leftDriveBack.setStopping(brakeType::brake);
        rightDriveFront.setStopping(brakeType::brake);
        rightDriveBack.setStopping(brakeType::brake);
    }
    else{
        leftDriveFront.setStopping(brakeType::coast);
        leftDriveBack.setStopping(brakeType::coast);
        rightDriveFront.setStopping(brakeType::coast);
        rightDriveBack.setStopping(brakeType::coast);
    }
}

void driveLeft(){
    int scaledPower {Controller.Axis3.value()}; 
     
    if (abs(scaledPower) < 5)
        scaledPower = 0;
    else
        scaledPower = ( (scaledPower^2/100) * scaledPower ) / 100;
    
    if (Controller.Axis3.value() < 0) 
        scaledPower = scaledPower * -0.6;
    else
        scaledPower = scaledPower *  0.6;
    
    leftDriveFront.spin(directionType::fwd, scaledPower, velocityUnits::pct); 
    leftDriveBack.spin(directionType::fwd, scaledPower, velocityUnits::pct); 
}

void driveRight(){
    int scaledPower {Controller.Axis2.value()}; 
    
    if (abs(scaledPower) < 5)
        scaledPower = 0;
    else
        scaledPower = ( (scaledPower^2/100) * scaledPower ) / 100;
    
    if (Controller.Axis2.value() < 0) 
        scaledPower = scaledPower * -0.6;
    else
        scaledPower = scaledPower *  0.6;
    
    rightDriveFront.spin(directionType::fwd, scaledPower, velocityUnits::pct); 
    rightDriveBack.spin(directionType::fwd, scaledPower, velocityUnits::pct); 
}

void moveLeft(int distance, int power){
    leftDriveFront.startRotateFor(distance, rotationUnits::deg, power, velocityUnits::pct); 
    leftDriveBack.startRotateFor(distance, rotationUnits::deg, power, velocityUnits::pct); 
}

void moveRight(int distance, int power){
    rightDriveFront.startRotateFor(distance, rotationUnits::deg, power, velocityUnits::pct); 
    rightDriveBack.startRotateFor(distance, rotationUnits::deg, power, velocityUnits::pct); 
}

void resetMotors(){
    rightDriveFront.setRotation(0, rotationUnits::deg); 
    rightDriveBack.setRotation(0, rotationUnits::deg); 
    leftDriveFront.setRotation(0, rotationUnits::deg); 
    leftDriveBack.setRotation(0, rotationUnits::deg); 
}

static bool toggle {false};

void toggleIntake()
{
    toggle = !toggle;

    if (toggle)
        ballIntake.spin(directionType::fwd, 100, velocityUnits::pct); 
    else
        ballIntake.spin(directionType::fwd, 0, velocityUnits::pct);
}

void puIntake()
{
    ballIntake.spin(directionType::fwd, 100, velocityUnits::pct);
}

void pdIntake()
{
    ballIntake.stop();
}

void puIndexer()
{
    if(Controller.ButtonR1.pressing()){
        indexer.spin(directionType::rev, 100, velocityUnits::pct);
        ballIntake.spin(directionType::rev, 100, velocityUnits::pct);
    }
    else{
        indexer.spin(directionType::fwd, 100, velocityUnits::pct);
        ballIntake.spin(directionType::fwd, 100, velocityUnits::pct);
    }
    
}

void pdIndexer()
{
    indexer.stop();
    ballIntake.stop();
}

void raiseArm(){
    arm.spin(directionType::fwd, 40, velocityUnits::pct);
}

void lowerArm(){
    arm.spin(directionType::rev, 40, velocityUnits::pct);
}

void stopArm(){
    arm.stop();
}
