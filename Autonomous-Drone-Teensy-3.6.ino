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
volatile uint32_t countDmp = 0; 	// Number of received DMP interrupts. Used to determine when the DMP is stabilized
volatile bool newDmpData = false;	// True if new data from DMP hasn't been read yet.
volatile uint32_t lastDmpData = 0;

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

	// Enable interrupt timer at 400 Hz for throttle update.
	// Set priority to highest as to not be interrupted by other interrupts
	speedTimer.begin(interruptThrottle, 2500); 	
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

uint32_t testFreq = 0;
uint32_t timing = 0;

uint32_t junkTimer = 0;
uint32_t rpmTimer = 0;
uint32_t mmaTimer = 0;

uint32_t junkInterval = 2500; // 200 Hz, shifted 1250 microseconds
uint32_t rpmInterval = 2500;
uint32_t mmaInterval = 5000; // 200 Hz

uint8_t timeAvailable = 0;

void loop() {

	/* 
   * ----- Safety motor of ----- 
   * Turn motors off if last packet was received to long ago
   */
   
	if( millis() - lastAlive > 500 ) {
		FC.motorsOn = false;
	}

  // Variable used for precisely timing the jobs
  timing = micros() - lastDShot;

  /* 
   *  ----- RPM Timing -----
   *  It takes a little time for ESC telemetry to reach Teensy, and the RPM control system takes short time to process
   *  We put rpm control and telemetry read in back of time-frame, 300us is more than enough
   */
 
   
  if( timing >= 2200 && (micros() - rpmTimer >= rpmInterval ) ){
    rpmTimer = micros() + (2200 - timing); // Adjust the rpmTimer such that we always run the rpm-thingy at the same time. 

    // FUCKING TELEMETRY DOESNT WORK INSIDE HERE WHY
    // WHYYYYYY???
    // NOW IT DOES
    // WUP WUP - MASTER PROGRAMMAR WAS HERE
    FC.getTelemetry();
    FC.motorRPMControl();

    /* Serial.println( micros() - testFreq ); 
    testFreq = micros(); */
    
  }

   /* 
   *  ----- MMA Timing -----
   *  The reading of DMP angles takes quite a while (+1500us) and is therefore placed frist in the time-frame
   *  The DMP hardware interrupt is ditched as it fucks up timing. Instead af software check is in place inside readDMPAngles().
   */
   
  if( timing >= 10 && (micros() - mmaTimer >= mmaInterval)){
    mmaTimer = micros() + (10 - timing) ;  // Adjust the timer to always hit 10us after dshot..

    // Set to tell next "time-frame" is free for junk (wifi)
    timeAvailable = 1;

    // Read angles/data from DMP (FUCK THE HARDWARE INTERRUPT)
    FC.readDMPAngles();

    // Motor mixing algorithm and controller
    FC.motorMixing();
  }

  /*
   * ----- Junk timer ------
   * This takes care of lesser important tasks running at the 2500us shifted time-frame
   * WiFi communcation is placed here
   */
   
  if( timing >= 10 && timeAvailable == 2 && (micros() - junkTimer >= junkInterval)){
    junkTimer = micros() + (10 - timing) ;  // Adjust the timer to always hit 10us after dshot..
    timeAvailable = 0;

    uint32_t startTimer = micros();
  
    // Send telemetry over WiFI
    sendTelemetry();

    // Read commands from WiFi
    if( CC.listen() ){
      // Process command
      commandList();
    }
  }

  /* 
  // MMA takes 1600us + 200us headroom.
  if( newTlm && timing > 1800 ){

    Serial.println(micros()-testFreq);
    testFreq = micros();
    
    FC.getTelemetry();
    FC.motorRPMControl();
    newTlm = false;
    
  }
  
	// Listen for commands from WiFi
  if( mmaDone && newTlm == false && timing <= 200 ){
    
    sendTelemetry();
    
  	if( CC.listen() ){
  		// Process command
  		commandList();
  	}
  }*/

}

// Function called when new DMP data is ready
void dmpDataReady(){
	newDmpData = true;
	countDmp++;
  lastDmpData = micros();
}

// Function to update motor speeds
void interruptThrottle(){
  
  if( timeAvailable) timeAvailable++;
  
  FC.ESC1->requestTelemetry(); 
  FC.ESC2->requestTelemetry(); 
  FC.ESC3->requestTelemetry(); 
  FC.ESC4->requestTelemetry();
  
	FC.setMotorSpeeds();
  lastDShot = micros();
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

    Serial.println( CC.buffer );
  
		CC.value( "P", &FC.config.rollPID[0] );
		CC.value( "I", &FC.config.rollPID[1] );
		CC.value( "D", &FC.config.rollPID[2] );
		FC.Roll.setConstants( FC.config.rollPID );
	}

  if( CC.check( "ROLLRATE" ) ){

    Serial.println( CC.buffer );
    
    CC.value( "P", &FC.config.rollRatePID[0] );
    CC.value( "I", &FC.config.rollRatePID[1] );
    CC.value( "D", &FC.config.rollRatePID[2] );
    FC.RollRate.setConstants( FC.config.rollRatePID );
  }

	if( CC.check( "PITCH" ) ){

    Serial.println( CC.buffer );
		CC.value( "P", &FC.config.pitchPID[0] );
		CC.value( "I", &FC.config.pitchPID[1] );
		CC.value( "D", &FC.config.pitchPID[2] );
		FC.Pitch.setConstants( FC.config.pitchPID );
	}

  if( CC.check( "PITCHRATE" ) ){
    Serial.println( CC.buffer );
    CC.value( "P", &FC.config.pitchRatePID[0] );
    CC.value( "I", &FC.config.pitchRatePID[1] );
    CC.value( "D", &FC.config.pitchRatePID[2] );
    FC.PitchRate.setConstants( FC.config.pitchRatePID );
  }

	if( CC.check( "YAW" ) ){
    Serial.println( CC.buffer );
		CC.value( "P", &FC.config.yawPID[0] );
		CC.value( "I", &FC.config.yawPID[1] );
		CC.value( "D", &FC.config.yawPID[2] );
		FC.Yaw.setConstants( FC.config.yawPID );
	}

  if( CC.check( "YAWRATE" ) ){
    Serial.println( CC.buffer );
    CC.value( "P", &FC.config.yawRatePID[0] );
    CC.value( "I", &FC.config.yawRatePID[1] );
    CC.value( "D", &FC.config.yawRatePID[2] );
    FC.YawRate.setConstants( FC.config.yawRatePID );
  }

  if( CC.check( "ALTITUDE" ) ){
    Serial.println( CC.buffer );
    CC.value( "P", &FC.config.altPID[0] );
    CC.value( "I", &FC.config.altPID[1] );
    CC.value( "D", &FC.config.altPID[2] );
    FC.Altitude.setConstants( FC.config.altPID );
  }

	if( CC.check( "OFFSET" ) ){
    Serial.println( CC.buffer );
		CC.value( "H", &FC.config.hoverOffset );
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

  // Add roll rate config
  CC.addToBuffer( FC.config.rollRatePID[0] );
  CC.addToBuffer( FC.config.rollRatePID[1] );
  CC.addToBuffer( FC.config.rollRatePID[2] );

  // Add pitch rate config
  CC.addToBuffer( FC.config.pitchRatePID[0] );
  CC.addToBuffer( FC.config.pitchRatePID[1] );
  CC.addToBuffer( FC.config.pitchRatePID[2] );
  
  // Add yaw rate config
  CC.addToBuffer( FC.config.yawRatePID[0] );
  CC.addToBuffer( FC.config.yawRatePID[1] );
  CC.addToBuffer( FC.config.yawRatePID[2] );

	// Add altitude config
	CC.addToBuffer( FC.config.altPID[0] );
	CC.addToBuffer( FC.config.altPID[1] );
	CC.addToBuffer( FC.config.altPID[2] );

	// Add also motor offsets
	CC.addToBuffer( FC.config.hoverOffset );

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
	CC.addToBuffer( FC.height );
	
	// Add battery voltage
	CC.addToBuffer( FC.batteryVoltage() );

	// Add motor outputs for inspection
	CC.addToBuffer( FC.s1 );
	CC.addToBuffer( FC.s2 );
	CC.addToBuffer( FC.s3 );
	CC.addToBuffer( FC.s4 );

	CC.sendBuffer();

} 
