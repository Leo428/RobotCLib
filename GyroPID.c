#ifndef __GyroPID__
#define __GyroPID__

typedef struct{
	tSensors port;
	tMotor rightMasterMotor;
	tMotor leftMasterMotor;
	float setPoint;
	float absoluteTolerance;
	float inputRangeMax;
	float inputRangeMin;
	float outputRangeMax;
	float outputRangeMin;
	bool isRunning;
	bool onTarget;
} GyroPID;

const static float  pid_Kp = 2.0;
static GyroPID gyroPID;
static float requestedValue;
static float currentValue;

void resetGyro(){
	SensorValue[gyroPID.port] = 0;
}

float rectifyInput(float input){
	if(input > gyroPID.inputRangeMax) return gyroPID.inputRangeMax;
	else if (input < gyroPID.inputRangeMin) return gyroPID.inputRangeMin;
	else return input;
}

float rectifyOutput(float output){
	if(output > gyroPID.outputRangeMax) return gyroPID.outputRangeMax;
	else if (output < gyroPID.outputRangeMin) return gyroPID.outputRangeMin;
	else return output;
}

//determine whether it has reached the target within the tolerance
bool isOnTarget(float offset){
	if(abs(offset) < gyroPID.absoluteTolerance) return true;
	else return false;
}

task turnWithGyro(){
	resetGyro();
	float error;
	float output;
	while(true){
		if (gyroPID.isRunning){
			currentValue = rectifyInput(SensorValue[gyroPID.port]);
			requestedValue = gyroPID.setPoint;
			error = currentValue - requestedValue;
			gyroPID.onTarget = isOnTarget(error);
			if (!gyroPID.onTarget){
				output =  rectifyOutput(pid_Kp * error);
				motor[gyroPID.leftMasterMotor] = -output;
				motor[gyroPID.rightMasterMotor] = output;
			}
		}
		wait1Msec(25);
	}
}

void initGyroPID(){
	startTask(turnWithGyro);
}

void startGyroPID(){
	gyroPID.isRunning = true;
}

void stopGyroPID(){
	gyroPID.isRunning = false;
}

bool gyroPIDState(){
	return gyroPID.isRunning;
}
#endif
