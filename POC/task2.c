#pragma config(Sensor, S1,     gyro,           sensorI2CHiTechnicGyro)
#pragma config(Motor,  motorB,          motorLeft,     tmotorNXT, PIDControl, driveLeft, encoder)
#pragma config(Motor,  motorC,          motorRight,    tmotorNXT, PIDControl, driveRight, encoder)

#include "gyroPid.h"

static int xCoorStart, yCoorStart, xCoorEnd, yCoorEnd, xDiff, yDiff;
const unsigned char noError = 0x01; // hex for 0000 0001
const unsigned char manOveride = 0x02; // hex for 0000 0010
const unsigned char outBound = 0x04; // hex for 0000 0100
const unsigned char noALV = 0x08; // hex for 0000 1000
const unsigned char LSTSError = 0x10; // hex for 0001 0000
const unsigned char BUSY = 0x20; // hex for 0010 0000
const unsigned char errorTypes[6] = {noError, manOveride, outBound, noALV, LSTSError, BUSY};


static int masterPower = 120;
static int slavePower = 120;
static int distCount = 0;

task driveStraight()
{

	int error = 0;

	int kp = 5;

	nMotorEncoder[motorLeft] = 0;
	nMotorEncoder[motorRight] = 0;

	while(true)
	{

		motor[motorLeft] = masterPower;
		motor[motorRight] = slavePower;

		error = nMotorEncoder[motorLeft] - nMotorEncoder[motorRight]; // find diference between two motor positions

		slavePower += error / kp; // adjust slave power to match master speed

		distCount += nMotorEncoder[motorLeft]; // sum total distance

		nMotorEncoder[motorLeft] = 0; // reset encoders at end of loop
		nMotorEncoder[motorRight] = 0;

		wait1Msec(100);

	}
}

void driveUntil(int distance){

	//take distance in meters

	distCount = 0;
	startTask(driveStraight);

	while (distCount < distance * 0.3048 * 1000){
	}

	stopTask(driveStraight);

	motor[motorLeft] = 0;
	motor[motorRight] = 0;
}

void messageCheck(){
	int i = 0;
	ClearMessage();
	wait1Msec(20);
	sendMessage(5);
	while (!messageParm[0]){
		ClearMessage();
		wait1Msec(1);
	}

	for (i = 0; i < 6; i++){ // use bitmaps to determine which errors are present
		switch (errorTypes[i] & messageParm[0]){
		case noError:
			displayTextLine(i, "%s", "noError");
			xCoorStart = messageParm[1];
			yCoorStart = messageParm[2];
			displayString(4, "%d, %d", xCoorStart, yCoorStart);
			break;
		case manOveride:
			displayTextLine(i, "%s", "manOveride");
			xCoorStart = messageParm[1];
			yCoorStart = messageParm[2];
			displayString(4, "%d, %d", xCoorStart, yCoorStart);
			break;
		case outBound:
			displayTextLine(i, "%s", "outBound");
			messageCheck();
			break;
		case noALV:
			displayTextLine(i, "%s", "noALV");
			messageCheck();
			break;
		case LSTSError:
			displayTextLine(i, "%s", "LSTSError");
			messageCheck();
			break;
		case BUSY:
			displayTextLine(i, "%s", "BUSY");
			messageCheck();
			break;
		}
	}
}

task main(){


	calibrate(); //calibrate gyro value
	pidRequestedValue = 90; //turn will be 90 degrees ccw
	xCoorEnd = 75; //goal coordinates
	yCoorEnd = 135;

	eraseDisplay();

	lastMessage = nPgmTime;
	messageCheck(); // get message from LSTS

	xDiff = (xCoorStart - xCoorEnd) / 100.0; //convert to meters
	yDiff = (yCoorStart - yCoorEnd) / 100.0;

	displayString(1, "XDiff = %d", xDiff);
	displayString(2, "YDiff = %d", yDiff);

	driveUntil(xDiff); // drive xDiff meters in x direction

	while (nPgmTime - lastMessage < 1500){ // min 15 seconds between messages
		wait1Msec(1);
	}
	messageCheck(); // get message from LSTS

	// calculate y distances needed to travel in meters

	yDiff = (yCoorStart - yCoorEnd) / 100.0;

	// turn 90 degrees clockwise

	startTask(pidController);

	while (fabs(angle - pidRequestedValue) > .1) // wait until we are within .1 of desired location
	{
	}

	stopTask(pidController);

	// drive yDiff meters in the y direction

	driveUntil(yDiff);

	wait1Msec(1000);

	// beep three times
	for (int i = 0; i < 3; i++){
		playSound(soundBlip);
		wait10Msec(10);
	}

}
