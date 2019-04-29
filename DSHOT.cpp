#include "DSHOT.h"

DShot::DShot( uint8_t num ){
  this->setup( num );
}

void DShot::setup( uint8_t num ){

	/* Configuration of FlexTimer0 */
	FTM0_SC = 0; 								// First disable FTM0 such that we can initiate it
	FTM0_CNT = 0;								// Reset the counter
  FTM0_CNTIN = 0;                // Reset the counter
	FTM0_MOD = mod - 1; 				// Set the modulo (overflow) value that triggers the timer event
	FTM0_SC = FTM_SC_CLKS(1) | FTM_SC_PS(0); 	// Set clock source to 1 (System clock) and prescale to 0 (Divide by 1)


	/* Configuration of each channel */
  if( num == 1 ){
    
    FTM0_CnSC(OUT1_TRIG_CH) = FTM_CSC_CHIE | FTM_CSC_DMA | FTM_CSC_MSA | FTM_CSC_ELSA;
    FTM0_CnSC(OUT1_PWM_CH)  = FTM_CSC_MSB | FTM_CSC_ELSB;
    FTM0_CnV(OUT1_TRIG_CH)  = 0;
    FTM0_CnV(OUT1_PWM_CH)   = 0;
    
    dma.destination( FTM0_CnV(OUT1_PWM_CH) );                           // Feed the PWM lenght to the PWM output channel
    dma.triggerAtHardwareEvent( DMA_SOURCE_FTM0_CHn(OUT1_TRIG_CH) );    // Set source of DMA tranfer (the trigger channel)

    // Configure PWM output pin
    FTM_PINCFG(OUT1_PWM_PIN) = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;
    
  }else if( num == 2 ){
    
    FTM0_CnSC(OUT2_TRIG_CH) = FTM_CSC_CHIE | FTM_CSC_DMA | FTM_CSC_MSA | FTM_CSC_ELSA;
    FTM0_CnSC(OUT2_PWM_CH)  = FTM_CSC_MSB | FTM_CSC_ELSB;
    FTM0_CnV(OUT2_TRIG_CH)  = 0;
    FTM0_CnV(OUT2_PWM_CH)   = 0;
    
    dma.destination( FTM0_CnV(OUT2_PWM_CH) );                           // Feed the PWM lenght to the PWM output channel
    dma.triggerAtHardwareEvent( DMA_SOURCE_FTM0_CHn(OUT2_TRIG_CH) ); // Set source of DMA tranfer (the trigger channel)

    // Configure PWM output pin
    FTM_PINCFG(OUT2_PWM_PIN) = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;
    
  }else if( num == 3 ){
    
    FTM0_CnSC(OUT3_TRIG_CH) = FTM_CSC_CHIE | FTM_CSC_DMA | FTM_CSC_MSA | FTM_CSC_ELSA;
    FTM0_CnSC(OUT3_PWM_CH)  = FTM_CSC_MSB | FTM_CSC_ELSB;
    FTM0_CnV(OUT3_TRIG_CH)  = 0;
    FTM0_CnV(OUT3_PWM_CH)   = 0;
    
    dma.destination( FTM0_CnV(OUT3_PWM_CH) );                           // Feed the PWM lenght to the PWM output channel
    dma.triggerAtHardwareEvent( DMA_SOURCE_FTM0_CHn(OUT3_TRIG_CH) ); // Set source of DMA tranfer (the trigger channel)

    // Configure PWM output pin
    FTM_PINCFG(OUT3_PWM_PIN) = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;
    
  }else if( num == 4 ){
    
    FTM0_CnSC(OUT4_TRIG_CH) = FTM_CSC_CHIE | FTM_CSC_DMA | FTM_CSC_MSA | FTM_CSC_ELSA;
    FTM0_CnSC(OUT4_PWM_CH)  = FTM_CSC_MSB | FTM_CSC_ELSB;
    FTM0_CnV(OUT4_TRIG_CH)  = 0;
    FTM0_CnV(OUT4_PWM_CH)   = 0;
    
    dma.destination( FTM0_CnV(OUT4_PWM_CH) );                           // Feed the PWM lenght to the PWM output channel
    dma.triggerAtHardwareEvent( DMA_SOURCE_FTM0_CHn(OUT4_TRIG_CH) ); // Set source of DMA tranfer (the trigger channel)

    // Configure PWM output pin
    FTM_PINCFG(OUT4_PWM_PIN) = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;
    
  }

  // Configure pin-independent DMA configuration
  dma.sourceBuffer((uint8_t *)dshotBuffer, DSHOT_BUFFER_LENGTH);  // Define source buffer
  dma.transferSize(1);
  dma.transferCount(DSHOT_BUFFER_LENGTH);
  dma.disableOnCompletion();
  
}

void DShot::setTlmPort( Stream * port ){

  this->tlmPort = port;
  
}

uint8_t DShot::getChecksum( uint16_t value ) {

	uint8_t checksum = 0;

	for (uint8_t i = 0; i < 3; i++) {
		checksum ^=  value;
		value >>= 4;
	}

	checksum &= 0xf; // mask off the first nibble

	return checksum;

}

void DShot::fillBuffer( uint16_t value) {

	memset(dshotBuffer, 0, DSHOT_BUFFER_LENGTH);

	// Scan all the bits in the packet
	for (int i = 0; i < DSHOT_COMMAND_LENGTH; i++){ 
		if ( (bool)((1 << i)&value) )
		{
			dshotBuffer[15 - i] = (mod * DSHOT_1_TIMING) >> 8; // pack buffer MSB first
		}
		else
		{
			dshotBuffer[15 - i] = (mod * DSHOT_0_TIMING) >> 8; // pack buffer MSB first
		}

	}
}

void DShot::setThrottle( uint16_t value ) {
	this->write( value + 47);
}


void DShot::setDirection( bool dir ){

  uint8_t value = 0;

  if( dir )
    value = 7;
  else
    value = 8;

  // Send setting 10 times
  for( uint8_t i = 0; i < 10; i++){

    this->requestTelemetry();
    this->write( value );
    delayMicroseconds(500);
      
  }

  delay(100);

  // Save settings
  for( uint8_t i = 0; i < 10; i++){

    this->requestTelemetry();
    this->write( 12 );
    delayMicroseconds(500);
      
  }
   
}

void DShot::setNormal(){

  uint8_t value = 9;

  // Send setting 10 times
  for( uint8_t i = 0; i < 10; i++){

    this->requestTelemetry();
    this->write( value );
    delayMicroseconds(500);
      
  }

  delay(100);

  // Save settings
  for( uint8_t i = 0; i < 10; i++){

    this->requestTelemetry();
    this->write( 12 );
    delayMicroseconds(500);
      
  }
   
}

void DShot::write( uint16_t value ) {
	uint16_t packet = 0;
	uint8_t checksum = 0;

	if (value > 2047) {
		value = 2047;
	}

	// Append the telemetry bit 
	packet = (value << 1) | (uint8_t)this->requestTlm;
	this->requestTlm = false;

	checksum = this->getChecksum(packet);
	packet = (packet << 4) | checksum;

	this->fillBuffer( packet );

	// Write the package by enabling the DMA
	dma.enable();

}

void DShot::requestTelemetry(){
	this->requestTlm = true;
}
