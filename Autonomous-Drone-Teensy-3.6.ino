#include <Wire.h>
#include "Settings.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Stabilizer.h"
#include "Wireless."

// Initialise Flight Controller (FC)
Stabilizer FC;

// Initialise Communication Controller (CC)
Wireless CC;

IntervalTimer speedTimer;

// Interrupt pin for IMU
uint8_t IMUPin = 2;

// Last time a "ALIVE" packet was received from interface, to safety shutoff motors if connection is lost
uint32_t lastAlive = 0;

// Variables to be used by interrupts
volatile uint32_t lastDShot = 0;
volatile uint16_t countDmp = 0;
volatile bool newDmpData = false;


void setup() {

	// Begin Serial for DEBUGGING
	Serial.begin(115200);

	// Setup wireless communication
	CC.setup();

	// Setup flight stablizer (IMU, altitude etc.)
	FC.setup();

	// The IMU pulls a pin HIGH whenever DMP data is ready to be read
	// Attach interrupt to this pin, to read when new data is ready
	pinMode(IMUPin, INPUT);
	attachInterrupt( digitalPinToInterrupt(IMUPin), dmpDataReady, RISING);

	// Enable interrupt timer at 333 Hz for throttle update.
	// Set priority to highest as to not be interrupted by other interrupts
	speedTimer.begin(interruptThrottle, 3000); 	
	speedTimer.priority(0);

	// Makes sure the DMP values have settled (1000 readings)
	while( countDmp < 1000 ){
		if (newDmpData){
			FC.readDMPAngles();
			newDmpData = false;  
		}
	}
	// Set home (current real world yaw = drones zero reference)
	FC.setHome();
	
}

void loop() {

	if( millis() - lastAlive < 500 ) {
		FC.motorsOn = false;
	}

	// New angle is ready, and there is at least 2900us to next dShot 
	if( newDmpData && (micros()- lastDShot <= 100) ) {

		// Calculate the new motor speeds from the newest Eulor angles
		FC.motorMixing();

		newDmpData = false;
	} 

	// If command was received from interface
	if( CC.listen() ){

		// Process command
		commandList();

	}

	FC.getTelemetry();

}

// Function called when new DMP data is ready
void dmpDataReady(){
	newDmpData = true;
	countDmp++;
}

// Function to update motor speeds
void interruptThrottle(){
	lastDShot = micros();
	FC.setMotorSpeeds();
}

void commandList(){

	if( CC.check( "CONFIG" ) ){
		sendConfig();
	}

	else if( CC.check( "ALIVE" ) ){
		lastAlive = millis();  
	}

	if( CC.check( "SAVE" ) ){
		FC.config.save();
	}

	if( CC.check( "START" ) ){
		FC.motorsOn = true;
	}

	if( CC.check( "STOP" ) ){
		FC.motorsOn = false;
	}

	if( CC.check( "ROLL" ) ){
		
		// Save the PID parameters in config
		CC.value( "P", &FC.config.rollKp );
		CC.value( "I", &FC.config.rollKi );
		CC.value( "D", &FC.config.rollKd );

		// And load them into the controller
		FC.Roll.setConstants( FC.config.rollKp, FC.config.rollKi, FC.config.rollKd );	

	}

	if( strstr( buffer, "PITCH" ) != NULL ){
		
		// Save the PID parameters in config
		CC.value( "P", &FC.config.pitchKp );
		CC.value( "I", &FC.config.pitchKi );
		CC.value( "D", &FC.config.pitchKd );

		// And load them into the controller
		FC.Pitch.setConstants( FC.config.pitchKp, FC.config.pitchKi, FC.config.pitchKd );	
		
	}

	if( strstr( buffer, "YAW" ) != NULL ){
		
		// Save the PID parameters in config
		CC.value( "P", &FC.config.yawKp );
		CC.value( "I", &FC.config.yawKi );
		CC.value( "D", &FC.config.yawKd );

		// And load them into the controller
		FC.Yaw.setConstants( FC.config.yawKp, FC.config.yawKi, FC.config.yawKd );	
		
	}

	if( strstr( buffer, "OFFSET" ) != NULL ){
		
		CC.value( "H", &FC.config.hoverOffset );
		CC.value( "A", &FC.config.offset1 );
		CC.value( "B", &FC.config.offset2 );
		CC.value( "C", &FC.config.offset3 );
		CC.value( "D", &FC.config.offset4 );
		
	}


}

void sendConfig() {

	CC.clearBuffer();

	CC.addToBuffer("CONFIG");

	// Add roll config
	CC.addToBuffer( "P" ); CC.addToBuffer( FC.config.rollKp );
	CC.addToBuffer( "I" ); CC.addToBuffer( FC.config.rollKi );
	CC.addToBuffer( "D" ); CC.addToBuffer( FC.config.rollKd );

	// Add pitch config
	CC.addToBuffer( "P" ); CC.addToBuffer( FC.config.pitchKp );
	CC.addToBuffer( "I" ); CC.addToBuffer( FC.config.pitchKi );
	CC.addToBuffer( "D" ); CC.addToBuffer( FC.config.pitchKd );
	
	// Add yaw config
	CC.addToBuffer( "P" ); CC.addToBuffer( FC.config.yawKp );
	CC.addToBuffer( "I" ); CC.addToBuffer( FC.config.yawKi );
	CC.addToBuffer( "D" ); CC.addToBuffer( FC.config.yawKd );

	// Add altitude config
	CC.addToBuffer( "P" ); CC.addToBuffer( FC.config.altKp );
	CC.addToBuffer( "I" ); CC.addToBuffer( FC.config.altKi );
	CC.addToBuffer( "D" ); CC.addToBuffer( FC.config.altKd );

	// Add also motor offsets
	CC.addToBuffer( "H" ); CC.addToBuffer( FC.config.hoverOffset );
	CC.addToBuffer( "A" ); CC.addToBuffer( FC.config.offset1 );
	CC.addToBuffer( "B" ); CC.addToBuffer( FC.config.offset2 );
	CC.addToBuffer( "C" ); CC.addToBuffer( FC.config.offset3 );
	CC.addToBuffer( "D" ); CC.addToBuffer( FC.config.offset4 );

	// Send everything
	CC.sendBuffer();
	
}


void sendTelemetry( const char * type ){

	char tempBuffer[20] = {'\0'};
	memset(buffer, 0, sizeof(buffer));
	
	if( strcmp( type, "ANGLE" ) == 0 ) {
		
		strcpy(buffer, type);
	
		// Build telemetry buffer
		sprintf(tempBuffer, "%.2f", FC.angles[0]);
		strcat(buffer, " Y");
		strcat(buffer, tempBuffer);
	
		sprintf(tempBuffer, "%.2f", FC.angles[2]);
		strcat(buffer, " P");
		strcat(buffer, tempBuffer);
		
		sprintf(tempBuffer, "%.2f", FC.angles[1]);
		strcat(buffer, " R");
		strcat(buffer, tempBuffer);
	
		WIFI.println(buffer);
	}
	
	else if( strcmp( type, "ALTITUDE" ) == 0 ) {
		
		strcpy(buffer, type);
	
		sprintf(tempBuffer, "%d", altitudeVal);
		strcat(buffer, " A");
		strcat(buffer, tempBuffer);
	
		WIFI.println(buffer);
	}

	else if( strcmp( type, "BATTERY" ) == 0 ) {

		// Find voltage of battery from average

		float voltage = 0.0;

		/* for( uint8_t i = 0; i < 4; i++ ){
			voltage += telemetry[i].voltage; 
		}

		voltage = voltage/4.0;*/
		
		strcpy(buffer, type);
	
		sprintf(tempBuffer, "%.2f", voltage);
		strcat(buffer, " V");
		strcat(buffer, tempBuffer);
	
		WIFI.println(buffer);

	}

	else if( strcmp( type, "SPEED" ) == 0 ) {
	
		strcpy(buffer, type);
	
		// Build telemetry buffer
		sprintf(tempBuffer, "%d", FC.s1);
		strcat(buffer, " A");
		strcat(buffer, tempBuffer);
	
		sprintf(tempBuffer, "%d", FC.s2);
		strcat(buffer, " B");
		strcat(buffer, tempBuffer);
		
		sprintf(tempBuffer, "%d", FC.s3);
		strcat(buffer, " C");
		strcat(buffer, tempBuffer);

		sprintf(tempBuffer, "%d", FC.s4);
		strcat(buffer, " D");
		strcat(buffer, tempBuffer);
	
		WIFI.println(buffer);
	}
} 