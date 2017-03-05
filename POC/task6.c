#pragma config(Sensor, S1,     gyro,           sensorI2CHiTechnicGyro)
#pragma config(Motor,  motorB,          motorLeft,     tmotorNXT, PIDControl, driveLeft, encoder)
#pragma config(Motor,  motorC,          motorRight,    tmotorNXT, PIDControl, driveRight, encoder)


int xCoorStart, yCoorStart;
const unsigned char noError = 0x01; // hex for 0000 0001
const unsigned char manOveride = 0x02; // hex for 0000 0010
const unsigned char outBound = 0x04; // hex for 0000 0100
const unsigned char noALV = 0x08; // hex for 0000 1000
const unsigned char LSTSError = 0x10; // hex for 0001 0000
const unsigned char BUSY = 0x20; // hex for 0010 0000
const unsigned char errorTypes[6] = {noError, manOveride, outBound, noALV, LSTSError, BUSY};


task main(){

	while (1){
		eraseDisplay();

		ClearMessage();
		wait1Msec(20);
		sendMessage(5);
		while (!messageParm[0]){
			wait1Msec(1);
		}


		for (int i = 0; i < 6; i++){
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
					xCoorStart = messageParm[1];
					yCoorStart = messageParm[2];
					displayString(4, "coordinates invalid");
					break;
				case noALV:
					displayTextLine(i, "%s", "noALV");
					xCoorStart = messageParm[1];
					yCoorStart = messageParm[2];
					displayString(4, "coordinates invalid");
					break;
				case LSTSError:
					displayTextLine(i, "%s", "LSTSError");
					xCoorStart = messageParm[1];
					yCoorStart = messageParm[2];
					displayString(4, "coordinates invalid");
					break;
				case BUSY:
					displayTextLine(i, "%s", "BUSY");
					xCoorStart = messageParm[1];
					yCoorStart = messageParm[2];
					displayString(4, "coordinates invalid");
					break;

				}
		}

		wait10Msec(3000);
	}
}
