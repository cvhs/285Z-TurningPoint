#pragma config(Sensor, dgtl3,  armEncoder,     sensorQuadEncoder)
#pragma config(Sensor, dgtl1,  encoder,        sensorQuadEncoder)
#pragma config(Motor,  port1,           frontLeftDrive,            tmotorVex393TurboSpeed_HBridge, openLoop)
#pragma config(Motor,  port2,           midLeftDrive,            tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port3,           backLeftDrive,            tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port8,           backRightDrive,            tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port9,           midRightDrive,            tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port10,          frontRightDrive,            tmotorVex393TurboSpeed_HBridge, openLoop)


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*        Description: Competition template for VEX EDR                      */
/*                                                                           */
/*---------------------------------------------------------------------------*/



// This code is for the VEX cortex platform
#pragma platform(VEX2)

// Select Download method as "competition"
#pragma competitionControl(Competition)

//Main competition background code...do not modify!
#include "Vex_Competition_Includes.c"
//#include "drivePIDtest.c"



/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

//PID FORWARDS:
void moveForward(int power){
	motor[frontLeftDrive] = power + 15;
	motor[midLeftDrive] = power + 15;
	motor[backLeftDrive] = -power + 15;

	motor[frontRightDrive] = - power;
	motor[midRightDrive] = - power;
	motor[backRightDrive] = - power;
}

float cmToDegrees(float cm){
	float ticks;
	ticks = cm * 10.746;
	return ticks;
}

void PIDForward(float target, float waitTime, float powerHardLimit = 1){ //waitTime affects accuracy, needs tuning, in ms, should be at least 200; powerHardLimit is optional and is a decimal represeenting a percentage
	float Kp = 1.0; //needs to be tuned
	float Ki = 0.03; //smaller for faster loops, larger for slower loops
	float Kd = 0.7; //no clue about this value

	int error;
	int lastError;

	float proportion;
	int integralRaw;
	float integral;
	int derivative;

	float integralActiveZone = cmToDegrees(8); //needs to be tested by turning off integral and seeing where proportional stops relative to target
	int integralPowerLimit = 50/Ki; //set me to 50/Ki after tuning

	int finalPower;

	bool timerBool = true;

	SensorValue[encoder] = 0;

	clearTimer(T1);

	if (waitTime < 200){
		waitTime = 200;
	}

	while(time1[T1] < waitTime) { //for now
		error = cmToDegrees(target) - (SensorValue[encoder]); //I think it is an average, not like in the tutorial
		clearLCDLine(0);
		displayLCDNumber(0, 2, error, 0);

		proportion = Kp*error;

		if (error < integralActiveZone && error != 0) {
			integralRaw = integralRaw + error;
		}
		else {
			integralRaw = 0;
		}

		if (integralRaw > integralPowerLimit) {
			integralRaw = integralPowerLimit;
		}
		if (integralRaw < - integralPowerLimit) {
			integralRaw = -integralPowerLimit;
		}

		integral = integralRaw*Ki;

		derivative = Kd*(error - lastError);
		lastError = error;

		if (error == 0) {
			derivative = 0;
		}

		finalPower = proportion + integral;

		if (finalPower > powerHardLimit*127) {
			finalPower = powerHardLimit*127;
		}
		else if (finalPower < -powerHardLimit*127) {
			finalPower = -powerHardLimit*127;
		}

		moveForward(finalPower);

		wait1Msec(40);

		if (error < 32){
			timerBool = false;
		}
		if (timerBool) {
			clearTimer(T1);
		}
	}
	displayLCDNumber(0, 2, error, 0);
	moveForward(0);
}

//PID BACKWARDS:
void moveBackward(int power){
	motor[frontLeftDrive] = -power;
	motor[midLeftDrive] = -power;
	motor[backLeftDrive] = power;

	motor[frontRightDrive] = power;
	motor[midRightDrive] = power;
	motor[backRightDrive] = power;
}

void PIDBackward(float target, float waitTime, float powerHardLimit = 1){ //waitTime affects accuracy, needs tuning, in ms, should be at least 200; powerHardLimit is optional and is a decimal represeenting a percentage
	float Kp = 1.0; //needs to be tuned
	float Ki = 0.03; //smaller for faster loops, larger for slower loops
	float Kd = 0.7; //no clue about this value

	int error;
	int lastError;

	float proportion;
	int integralRaw;
	float integral;
	int derivative;

	float integralActiveZone = cmToDegrees(8); //needs to be tested by turning off integral and seeing where proportional stops relative to target
	int integralPowerLimit = 50/Ki; //set me to 50/Ki after tuning

	int finalPower;

	bool timerBool = true;

	SensorValue[encoder] = 0;

	clearTimer(T1);

	if (waitTime < 200){
		waitTime = 200;
	}

	while(time1[T1] < waitTime) { //for now
		error = cmToDegrees(target) - (-1* SensorValue[encoder]); //I think it is an average, not like in the tutorial
		clearLCDLine(0);
		displayLCDNumber(0, 2, error, 0);

		proportion = Kp*error;

		if (error < integralActiveZone && error != 0) {
			integralRaw = integralRaw + error;
		}
		else {
			integralRaw = 0;
		}

		if (integralRaw > integralPowerLimit) {
			integralRaw = integralPowerLimit;
		}
		if (integralRaw < - integralPowerLimit) {
			integralRaw = -integralPowerLimit;
		}

		integral = integralRaw*Ki;

		derivative = Kd*(error - lastError);
		lastError = error;

		if (error == 0) {
			derivative = 0;
		}

		finalPower = proportion + integral;

		if (finalPower > powerHardLimit*127) {
			finalPower = powerHardLimit*127;
		}
		else if (finalPower < -powerHardLimit*127) {
			finalPower = -powerHardLimit*127;
		}

		moveBackward(finalPower);

		wait1Msec(40);

		if (error < 32){
			timerBool = false;
		}
		if (timerBool) {
			clearTimer(T1);
		}
	}
	displayLCDNumber(0, 2, error, 0);
	moveBackward(0);
}

static float  pid_Kp = 2;
static float  pidRequestedValue;

/*-----------------------------------------------------------------------------*/
/*  pid control task                                                           */
/*-----------------------------------------------------------------------------*/

task pidController()
{
	float  pidSensorCurrentValue;
	float  pidError;
	float  pidDrive;

	while( true )
	{
		// Read the sensor value and scale
		pidSensorCurrentValue = SensorValue[ armEncoder ];

		// calculate error
		pidError =  pidRequestedValue - pidSensorCurrentValue;

		// calculate drive
		pidDrive = (pid_Kp * pidError);

		// limit drive
		if( pidDrive > 127 )
			pidDrive = 127;
		if( pidDrive < (-127) )
			pidDrive = (-127);

		// send to motor
		motor[ port4 ] = pidDrive;
		motor[ port5 ] = pidDrive;
		motor[ port6 ] = pidDrive;
		motor[ port7 ] = pidDrive;


		// Don't hog cpu
		wait1Msec( 25 );
	}
}

void pre_auton()
{
	motor[port7] = -20;
	motor[port6] = -20;
	motor[port5] = -20;
	motor[port4] = -20;
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

task autonomous()
{
	//Stationary Goal Auton
		int armTarget;
			SensorValue[armEncoder] = 0;

			//lift arm 70 degrees
			armTarget = 80;
			while(SensorValue[armEncoder] < armTarget){
				motor[port4] = motor[port5] = motor[port6] = motor[port7] = 45;
			}
			motor[port4] = motor[port5] = motor[port6] = motor[port7] = 0;
			wait1Msec(500);


			//move forward a little
			PIDForward(70, 200, 0.3);

			//lower arm to stack cone
			SensorValue[armEncoder] = 0;
			armTarget = -15;
			while(SensorValue[armEncoder] > armTarget){
				motor[port4] = motor[port5] = motor[port6] = motor[port7] = - 80;
			}
			motor[port4] = motor[port5] = motor[port6] = motor[port7] = 0;
			wait1Msec(500);

			//back away
			PIDBackward(40, 200, 0.7);
			SensorValue[armEncoder] = 0;
			armTarget = -40;
			while(SensorValue[armEncoder] > armTarget){
				motor[port4] = motor[port5] = motor[port6] = motor[port7] = - 40;
			}


}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

task usercontrol()
{
	SensorValue[ armEncoder ] = 0;
	// set initial position as the value of the pot
	pidRequestedValue = SensorValue[ armEncoder ];
	startTask(pidController);
	//bool for button level arm mappings
	bool reset = false;
	bool restart = false;
	bool levelOne = false;
	bool stationaryLevel = false;
	bool mogoUp = false;
	while(true){

		if(vexRT[Btn8R]){
			levelOne = false;
			stationaryLevel = false;
			mogoUp = false;
			reset = false;
			restart = true;
		}

		if (restart){
			//stopTask(pidController);
		  SensorValue[armEncoder] = 0;
			pidRequestedValue = SensorValue[armEncoder];
			startTask(pidController);
			restart = false;
		}
		/*if(vexRT[Btn8L]){
			levelOne = false;
			stationaryLevel = false;
			mogoUp = false;
			reset = true;
		}

		if (reset){
			stopTask(pidController);
			//startTask(pidController);
			reset = false;
		}*/

		if(vexRT[Btn8U]){
			levelOne = false;
			stationaryLevel = false;
			mogoUp = true;
			reset = false;
		}

		if (mogoUp){
			pidRequestedValue = 30;
		}
		/*if(vexRT[Btn7L]){
		mogoUp = false;
		stationaryLevel = false;
		levelOne = true;
		}

		if(levelOne){
		pidRequestedValue = 30;
		}*/

		if(vexRT[Btn7U]){
			levelOne = false;
			mogoUp = false;
			stationaryLevel = true;
			reset = false;
		}

		if(stationaryLevel){
			pidRequestedValue = 70;
		}

		//Drive/Joystick mappings
		motor[frontLeftDrive] = vexRT[Ch3] + vexRT[Ch1];
		motor[midLeftDrive] = vexRT[Ch3] + vexRT[Ch1];
		motor[backLeftDrive] =  -vexRT[Ch3] - vexRT[Ch1];
		motor[frontRightDrive] = - vexRT[Ch3] + vexRT[Ch1];
		motor[midRightDrive] = - vexRT[Ch3] + vexRT[Ch1];
		motor[backRightDrive] = - vexRT[Ch3] + vexRT[Ch1];

		/*
		bool driveSlow = true;
		while(true){

		if(vexRT[Btn8L]){
		driveSlow = true;
		}

		if(vexRT[Btn8R]){
		driveSlow = false;
		}

		if (driveSlow){
		motor[frontLeftDrive] = vexRT[Ch3];
		motor[midLeftDrive] = vexRT[Ch3];
		motor[backLeftDrive] =  -vexRT[Ch3];
		motor[frontRightDrive] = - vexRT[Ch2];
		motor[midRightDrive] = - vexRT[Ch2];
		motor[backRightDrive] = - vexRT[Ch2];
		} else {
		motor[frontLeftDrive] = vexRT[Ch3]/2;
		motor[midLeftDrive] =  vexRT[Ch3]/2;
		motor[backLeftDrive] =  - vexRT[Ch3]/2;
		motor[frontRightDrive] = - vexRT[Ch2]/2;
		motor[midRightDrive] = - vexRT[Ch2]/2;
		motor[backRightDrive] = - vexRT[Ch2]/2;
		driveSlow = false;
		} */


		if(vexRT[Btn5U]){
			levelOne = false;
			stationaryLevel = false;
			mogoUp = false;
			reset = false;
			pidRequestedValue = pidRequestedValue + 0.01;
			if (pidRequestedValue > 85){
				pidRequestedValue = 85;
			}
			}	else if (vexRT[Btn5D]){
			levelOne = false;
			stationaryLevel = false;
			mogoUp = false;
			reset = false;
			/*motor[port4] = -40;
			motor[port5] = -40;
			motor[port6] = -40;
			motor[port7] = -40;*/
			pidRequestedValue = pidRequestedValue - 0.01;
			if (pidRequestedValue < -15){
				pidRequestedValue = -15;
			}

		}

		else if (vexRT[Btn6U]){
			levelOne = false;
			stationaryLevel = false;
			mogoUp = false;
			reset = false;
			/*motor[port4] = 63;
			motor[port5] = 63;
			motor[port6] = 63;
			motor[port7] = 63;*/
			stopTask(pidController);
			motor[port4] = 80;
			motor[port5] = 80;
			motor[port6] = 80;
			motor[port7] = 80;
			while(vexRT[Btn6U]){}
			motor[port4] = 0;
			motor[port5] = 0;
			motor[port6] = 0;
			motor[port7] = 0;
			SensorValue[armEncoder] = 0;
			pidRequestedValue = 0;
			startTask(pidController);
			restart = false;
			/*pidRequestedValue = pidRequestedValue + 0.03;
			if (pidRequestedValue >= 80){
				pidRequestedValue = 80;
			}*/
			}	else if (vexRT[Btn6D]){
			levelOne = false;
			stationaryLevel = false;
			mogoUp = false;
			reset = false;
			stopTask(pidController);
			motor[port4] = -127;
			motor[port5] = -127;
			motor[port6] = -127;
			motor[port7] = -127;
			while(vexRT[Btn6D]){}
			motor[port4] = 0;
			motor[port5] = 0;
			motor[port6] = 0;
			motor[port7] = 0;
			SensorValue[armEncoder] = 0;
			pidRequestedValue = -5;
			startTask(pidController);
			restart = false;
			/*pidRequestedValue = pidRequestedValue - 0.03;
			if (pidRequestedValue < -15){
			pidRequestedValue = -15;
			}*/
		} /*else {
		motor[port4] = 10;
		motor[port5] = 10;
		motor[port6] = 10;
		motor[port7] = 10;*/

	}

	wait1Msec( 50 );

}
