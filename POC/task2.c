#pragma config(Sensor, S1,     gyro,           sensorI2CHiTechnicGyro)
#pragma config(Motor,  motorB,          motorLeft,     tmotorNXT, PIDControl, driveLeft, encoder)
#pragma config(Motor,  motorC,          motorRight,    tmotorNXT, PIDControl, driveRight, encoder)

#include "gyroPid.h"

int xCoorStart, yCoorStart, xCoorEnd, yCoorEnd, xDiff, yDiff;
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

    error = nMotorEncoder[motorLeft] - nMotorEncoder[motorRight];

    slavePower += error / kp;

    distCount += nMotorEncoder[motorLeft];

    nMotorEncoder[motorLeft] = 0;
  	nMotorEncoder[motorRight] = 0;

    wait1Msec(100);

  }
}

task main(){

	int i = 0;
	calibrate();

	xCoorEnd = 75;
	yCoorEnd = 135;

	eraseDisplay();

	ClearMessage();
	wait1Msec(20);
	sendMessage(5);
	while (!messageParm[0]){
		wait1Msec(1);
	}


	for (i = 0; i < 6; i++){
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
				break;
			case noALV:
				displayTextLine(i, "%s", "noALV");
				break;
			case LSTSError:
				displayTextLine(i, "%s", "LSTSError");
				break;
			case BUSY:
				displayTextLine(i, "%s", "BUSY");
				break;

			}
	}

	xDiff = (xCoorStart - xCoorEnd);
	yDiff = (yCoorStart - yCoorEnd);

	displayString(1, "XDiff = %d", xDiff);
	displayString(2, "YDiff = %d", yDiff);

	startTask(driveStraight);
	while (distCount < xDiff){
	}
	stopTask(driveStraight);

	motor[motorLeft] = 0;
	motor[motorRight] = 0;

	distCount = 0;

	ClearMessage();
	wait1Msec(20);
	sendMessage(5);
	while (!messageParm[0]){
		wait1Msec(1);
	}


	for (i = 0; i < 6; i++){
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
				break;
			case noALV:
				displayTextLine(i, "%s", "noALV");
				break;
			case LSTSError:
				displayTextLine(i, "%s", "LSTSError");
				break;
			case BUSY:
				displayTextLine(i, "%s", "BUSY");
				break;
			}
		}

	yDiff = (yCoorStart - yCoorEnd);

	pidRequestedValue = 90;

	startTask(pidController);

	while (fabs(angle - pidRequestedValue) > .1)
  {
	}

	stopTask(pidController);

	startTask(driveStraight);
	while (distCount < yDiff){
	}

	stopTask(driveStraight);

	motor[motorLeft] = 0;
	motor[motorRight] = 0;

	wait1Msec(1000);

	for (int i = 0; i < 3; i++){
		playSound(soundBlip);
		wait10Msec(10);
	}

}
