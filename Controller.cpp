#include "Controller.h"

Controller::Controller( void ){
	this->reset();
}

Controller::Controller( float Kp, float Ki, float Kd){

  this->reset();

	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;

}

void Controller::run(float setpoint, float input ){

    float deltaTime = (float)(micros() - lastTime)/1000000;
    lastTime = micros();

    error = setpoint - input; 

    // Add this cycle's integral to the integral cumulation
    integral += error * deltaTime; 

    // Calculate the slope with the data from the current and last cycle
    derivative = (error - lastError);

    // Prevent the integral cumulation from becoming overwhelmingly huge if maxIntegral isset
    if (maxIntegral != 0.0){
      if(integral > maxIntegral) integral = maxIntegral;
      if(integral < -maxIntegral) integral = -maxIntegral;
    }
    
    // Calculate the controller output based on the data and PID gains
    output = (error * Kp) + (integral * Ki) + (derivative * Kd);

    if( maxOutput != 0.0 ){
      if(output > maxOutput) output = maxOutput;
      if(output < -maxOutput) output = -maxOutput;
    }
    
    lastError = error;
}

float Controller::getOutput(){
    return output;
}

void Controller::reset(){
    this->error = 0.0;
    this->derivative = 0.0;
    this->integral = 0.0;
    this->output = 0.0;
}

void Controller::setConstants( float p, float i, float d ){
	this->Kp = p;
	this->Ki = i;
	this->Kd = d;
}

void Controller::setConstants( float * params ){
  this->Kp = params[0];
  this->Ki = params[1];
  this->Kd = params[2];
}

void Controller::setMaxIntegral( float value ){
  maxIntegral = value;
}

void Controller::setMaxOutput( float value ){
  maxOutput = value;
}
