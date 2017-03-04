#pragma config(Sensor, S1,     gyro,           sensorI2CHiTechnicGyro)
#pragma config(Motor,  motorB,          motorLeft,     tmotorNXT, PIDControl, driveLeft, encoder)
#pragma config(Motor,  motorC,          motorRight,    tmotorNXT, PIDControl, driveRight, encoder)

#include "gyroPid.c"

task main{
	wait1Msec(20);
	while (1){

		sendMessage(5);
		displayTextLine(3, "%s %s %s", messageParm[0], messageParm[1], messageParm[2]);
		wait1Msec(10);
	}

}
