#include <Wire.h>
#include "Settings.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Stabilizer.h"
#include "Wireless.h"

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
volatile uint32_t lastDShot = 0;	// Last time DShot was fired. Used to time other tasks
volatile uint16_t countDmp = 0; 	// Number of received DMP interrupts. Used to determine when the DMP is stabilized
volatile bool newDmpData = false;	// True if new data from DMP hasn't been read yet.

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

	// Makes sure the DMP values have settled (500 readings)
	while( countDmp < 500 ){
		if (newDmpData){
			FC.readDMPAngles();
			newDmpData = false;  
		}
	}
	// Set home (current real world yaw = drones zero reference)
	FC.setHome();
	
}

void loop() {

	// Turn motors off if last packet was received to long ago
	if( millis() - lastAlive > 500 ) {
		FC.motorsOn = false;
	}

	// New angle is ready, and there is at least 2900us to next DShot pulse
	// We would like to do our MMA calculations within a "free" periode, such that the interrupt doesn't interfere with the MMA
	if( newDmpData && (micros()- lastDShot <= 100) ) {

		// Calculate the new motor speeds from the newest Eulor angles
		FC.motorMixing();
		newDmpData = false;

		// Send telemetry over WiFi to GUI (100 Hz)
		sendTelemetry();
	} 

	// Listen for commands from WiFi
	if( CC.listen() ){
		// Process command
		commandList();
	}

	// Read ESC telemetry
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
    Serial.println( " -- MOTORS ON -- " );
		FC.motorsOn = true;
	}

	if( CC.check( "STOP" ) ){
    Serial.println( " -- MOTORS OFF -- " );
		FC.motorsOn = false;
	}

	if( CC.check( "ROLL" ) ){
		
		// Save the PID parameters in config
		CC.value( "P", &FC.config.rollPID[0] );
		CC.value( "I", &FC.config.rollPID[1] );
		CC.value( "D", &FC.config.rollPID[2] );

		// And load them into the controller
		FC.Roll.setConstants( FC.config.rollPID );	

	}

	if( CC.check( "PITCH" ) ){
    
		// Save the PID parameters in config
		CC.value( "P", &FC.config.pitchPID[0] );
		CC.value( "I", &FC.config.pitchPID[1] );
		CC.value( "D", &FC.config.pitchPID[2] );

		// And load them into the controller
		FC.Pitch.setConstants( FC.config.pitchPID );	
		
	}

	if( CC.check( "YAW" ) ){
		
		// Save the PID parameters in config
		CC.value( "P", &FC.config.yawPID[0] );
		CC.value( "I", &FC.config.yawPID[1] );
		CC.value( "D", &FC.config.yawPID[2] );

		// And load them into the controller
		FC.Yaw.setConstants( FC.config.yawPID );	
		
	}

	if( CC.check( "OFFSET" ) ){
		
		CC.value( "H", &FC.config.hoverOffset );
		CC.value( "A", &FC.config.motorOffset[0] );
		CC.value( "B", &FC.config.motorOffset[1] );
		CC.value( "C", &FC.config.motorOffset[2] );
		CC.value( "D", &FC.config.motorOffset[3] );
	
	}

}

void sendConfig() {

	CC.clearBuffer();

	CC.addToBuffer("CONFIG");

	// Add roll config
	CC.addToBuffer( FC.config.rollPID[0] );
	CC.addToBuffer( FC.config.rollPID[1] );
	CC.addToBuffer( FC.config.rollPID[2] );

	// Add pitch config
	CC.addToBuffer( FC.config.pitchPID[0] );
	CC.addToBuffer( FC.config.pitchPID[1] );
	CC.addToBuffer( FC.config.pitchPID[2] );
	
	// Add yaw config
	CC.addToBuffer( FC.config.yawPID[0] );
	CC.addToBuffer( FC.config.yawPID[1] );
	CC.addToBuffer( FC.config.yawPID[2] );

	// Add altitude config
	CC.addToBuffer( FC.config.altPID[0] );
	CC.addToBuffer( FC.config.altPID[1] );
	CC.addToBuffer( FC.config.altPID[2] );

	// Add also motor offsets
	CC.addToBuffer( FC.config.hoverOffset );
	CC.addToBuffer( FC.config.motorOffset[0] );
	CC.addToBuffer( FC.config.motorOffset[1]  );
	CC.addToBuffer( FC.config.motorOffset[2]  );
	CC.addToBuffer( FC.config.motorOffset[3]  );

	// Send everything
	CC.sendBuffer();
	
}


void sendTelemetry( ){

	CC.clearBuffer();

	CC.addToBuffer("TELEMETRY");

	// Add roll, pitch and yaw
	CC.addToBuffer( FC.angles[1] ); // Roll
	CC.addToBuffer( FC.angles[2] ); // Pitch
	CC.addToBuffer( FC.angles[0] ); // Yaw

	// Add altitude
	CC.addToBuffer( FC.altitude );
	
	// Add battery voltage
	CC.addToBuffer( FC.batteryVoltage() );

	// Add motor outputs for inspection
	CC.addToBuffer( FC.s1 );
	CC.addToBuffer( FC.s2 );
	CC.addToBuffer( FC.s3 );
	CC.addToBuffer( FC.s4 );

	CC.sendBuffer();

} 
