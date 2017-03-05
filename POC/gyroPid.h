#define PID_SENSOR_SCALE 1
#define PID_MOTOR_SCALE -1
#define PID_DRIVE_MAX 127
#define PID_DRIVE_MIN (-127)

#define PID_INTEGRAL_LIMIT 50

float pid_Kp = 1.3;
float pid_Ki = 0.005;
float pid_Kd = 0.00;

static int pidRunning = 1;
static float pidRequestedValue;

float offset = 0;
float gyroValue = 0;
float gyroRate = 0;
static float angle = 0;

float gyroAngle(){

	gyroValue = SensorRaw[gyro];
	gyroRate = gyroValue - offset;
	angle += (gyroRate / 100.0);

	displayTextLine(5, "%d\n", angle);

	return angle;
}


void calibrate(){
	float i = 0.0;
	int startTime;
	startTime = nPgmTime;
	while (nPgmTime - startTime < 3000){
		offset += SensorRaw[gyro];
		i++;
	}
	offset /= i;
	playSound(soundBlip);

}

task pidController()
{
    float  pidSensorCurrentValue;

    float  pidError;
    float  pidLastError;
    float  pidIntegral;
    float  pidDerivative;
    float  pidDrive;

    // Init the variables - thanks Glenn :)
    pidLastError  = 0;
    pidIntegral   = 0;
		//calibrate();
    while( true )
        {
        // Is PID control active ?
        if( pidRunning ) {
            angle = gyroAngle();
            // Read the sensor value and scale
            pidSensorCurrentValue = angle * PID_SENSOR_SCALE;

            // calculate error
            pidError = pidSensorCurrentValue - pidRequestedValue;

            // integral - if Ki is not 0
            if( pid_Ki != 0 )
                {
                // If we are inside controlable window then integrate the error
                if( fabs(pidError) < PID_INTEGRAL_LIMIT ){

                    pidIntegral = pidIntegral + pidError;
                  }
                else{

                    pidIntegral = 0;
                  }
                }
            else
                pidIntegral = 0;

            // calculate the derivative
            pidDerivative = pidError - pidLastError;
            pidLastError  = pidError;

            // calculate drive
            pidDrive = (pid_Kp * pidError) + (pid_Ki * pidIntegral) + (pid_Kd * pidDerivative);

            // limit drive
            if( pidDrive > PID_DRIVE_MAX )
                pidDrive = PID_DRIVE_MAX;
            if( pidDrive < PID_DRIVE_MIN )
                pidDrive = PID_DRIVE_MIN;

            // send to motor
            motor[motorLeft] = pidDrive * PID_MOTOR_SCALE;
            motor[motorRight] = - pidDrive * PID_MOTOR_SCALE;
            }
        else
            {
            // clear all
            pidError      = 0;
            pidLastError  = 0;
            pidIntegral   = 0;
            pidDerivative = 0;
            motor[motorLeft] = 0;
            motor[motorRight] = 0;
            }

        // Run at 20Hz
        wait1Msec( 10 );
        }
}
