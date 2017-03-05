#pragma config(Sensor, S1,     gyro,           sensorI2CHiTechnicGyro)
#pragma config(Motor,  motorB,          motorLeft,     tmotorNXT, PIDControl, driveLeft, encoder)
#pragma config(Motor,  motorC,          motorRight,    tmotorNXT, PIDControl, driveRight, encoder)


task main()
{
	nMotorEncoder[motorLeft] = 0;
	while (1){
		displayString(4, "%d", nMotorEncoder[motorLeft]);
	}
}
