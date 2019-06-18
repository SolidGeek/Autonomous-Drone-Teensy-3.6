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

// Timing 
uint32_t timing = 0;
uint32_t junkTimer = 0;
uint32_t rpmTimer = 0;
uint32_t mmaTimer = 0;

uint32_t junkInterval = 5000; // 200 Hz, shifted 2500 microseconds
uint32_t rpmInterval = 2500;
uint32_t mmaInterval = 5000; // 200 Hz

uint8_t timeAvailable = 0;

void setup() {

	// Begin Serial for DEBUGGING
	Serial.begin(115200);

	// Setup wireless communication
	CC.setup();

	// Setup flight stablizer (IMU, altitude etc.)
	FC.setup();

	// Enable interrupt timer at 400 Hz for throttle update.
	// Set priority to highest as to not be interrupted by other interrupts
	speedTimer.begin(interruptThrottle, 2500); 	
	speedTimer.priority(0);

  uint32_t settlingTimer = millis();
	// Makes sure the DMP values have settled (2 seconds)
	while( (millis() - settlingTimer < 5000) || ( FC.angles[1] > 1.0 && FC.angles[1] < -1.0 && FC.angles[2] > 1.0 && FC.angles[2] < -1.0) ){
		
	  FC.readDMPAngles();
    delayMicroseconds(2500); // 200 Hz

    FC.anglesSettled = true;
    
	}
	// Set home (current real world yaw = drones zero reference)
	FC.setHome();
	
}

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
  if( timing > 2500 )
    timing = 0;
  /* 
   *  ----- RPM Timing -----
   *  It takes a little time for ESC telemetry to reach Teensy, and the RPM control system takes short time to process
   *  We put rpm control and telemetry read in back of time-frame, 300us is more than enough
   */

  if( timing >= 2200 && (micros() - rpmTimer >= rpmInterval ) ){
    rpmTimer = micros() + (2200 - timing); // Adjust the rpmTimer such that we always run the rpm-thingy at the same time. 

    FC.getTelemetry();
    FC.motorRPMControl();
     
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
    
    // Send telemetry over WiFI
    sendTelemetry();

    // Read commands from WiFi
    if( CC.listen() ){
      // Process command
      commandList();
    }
  }
}

// Function to update motor speeds
void interruptThrottle(){
  
  lastDShot = micros();
  
  if( timeAvailable == 1 ) timeAvailable++;
  
  FC.ESC1->requestTelemetry(); 
  FC.ESC2->requestTelemetry(); 
  FC.ESC3->requestTelemetry(); 
  FC.ESC4->requestTelemetry();
  
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
    FC.rpmStartup = false;
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

  static bool tlmType = false;
  
	CC.clearBuffer();

  if( tlmType == true ){
  	CC.addToBuffer("TELEMETRY");
  
  	// Add roll, pitch and yaw
  	CC.addToBuffer( FC.angles[1] ); // Roll
  	CC.addToBuffer( FC.angles[2] ); // Pitch
  	CC.addToBuffer( FC.angles[0] ); // Yaw
    
  	// Add altitude
  	CC.addToBuffer( FC.height );
  	
  	// Add battery voltage
  	CC.addToBuffer( FC.batteryVoltage() );
  
  }else{
    
    CC.addToBuffer("RPM");
    
    CC.addToBuffer( FC.rpm[0] );
    CC.addToBuffer( FC.rpm[1] );
    CC.addToBuffer( FC.rpm[2] );
    CC.addToBuffer( FC.rpm[3] ); 
    
  }

  tlmType = !tlmType;

	CC.sendBuffer();

} 
