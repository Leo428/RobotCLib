#ifndef __EncoderPID__
#define __EncoderPID__

typedef struct{
	tSensors port;
	tMotor rightMasterMotor;
	tMotor leftMasterMotor;
	float setPoint;
	float absoluteTolerance;
	float outputRangeMax;
	float outputRangeMin;
	bool isRunning;
	bool onTarget;
} EncoderPID;

const static float  pid_Kp = 2.0;
static EncoderPID pidController;
static float requestedValue;
static float currentValue;

void resetEncoder(){
	SensorValue[pidController.port] = 0;
}

float rectifyOutput(float output){
	if(output > pidController.outputRangeMax) return pidController.outputRangeMax;
	else if (output < pidController.outputRangeMin) return pidController.outputRangeMin;
	else return output;
}

//determine whether it has reached the target within the tolerance
bool isOnTarget(float offset){
	if(abs(offset) < pidController.absoluteTolerance) return true;
	else return false;
}

task moveWithEncoder(){
	resetEncoder();
	float error;
	float output;
	while(true){
		if (pidController.isRunning){
			currentValue = SensorValue[pidController.port];
			requestedValue = pidController.setPoint;
			error = currentValue - requestedValue;
			pidController.onTarget = isOnTarget(error);
			if (!pidController.onTarget){
				output =  rectifyOutput(pid_Kp * error);
				motor[pidController.leftMasterMotor] = output;
				motor[pidController.rightMasterMotor] = output;
			}
		}
		wait1Msec(25);
	}
}

void initEncoderPID(){
	startTask(moveWithEncoder);
}

void startEncoderPID(){
	pidController.isRunning = true;
}

void stopEncoderPID(){
	pidController.isRunning = false;
}

bool encoderPIDState(){
	return pidController.isRunning;
}
#endif
