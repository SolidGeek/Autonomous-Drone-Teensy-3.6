#ifndef _DSHOT_H_
#define _DSHOT_H_

#include <Arduino.h>
#include "DMAChannel.h"

#define DSHOT_COMMAND_LENGTH 16
#define DSHOT_BUFFER_LENGTH DSHOT_COMMAND_LENGTH+1

// Flextimer pins for PWM output
#define OUT1_PWM_PIN 22 // FTM0_CH0_PIN - PTC1 Alt(4)
#define OUT2_PWM_PIN 23 // FTM0_CH1_PIN - PTC2 Alt(4)
#define OUT3_PWM_PIN 6  // FTM0_CH4_PIN - PTD4 Alt(4)
#define OUT4_PWM_PIN 20 // FTM0_CH5_PIN - PTD5 Alt(4)

// Flextimer channels for PWMOUT and TRIGGER
#define OUT1_PWM_CH 0
#define OUT2_PWM_CH 1
#define OUT3_PWM_CH 4
#define OUT4_PWM_CH 5
#define OUT1_TRIG_CH 2
#define OUT2_TRIG_CH 3
#define OUT3_TRIG_CH 6
#define OUT4_TRIG_CH 7

// Macros
#define FTM_PINCFG(pin) FTM_PINCFG2(pin)
#define FTM_PINCFG2(pin) CORE_PIN ## pin ## _CONFIG
#define FTM0_CnSC(ch) FTM0_CnSC2(ch)
#define FTM0_CnSC2(ch) FTM0_C ## ch ## SC
#define FTM0_CnV(ch) FTM0_CnV2(ch)
#define FTM0_CnV2(ch) FTM0_C ## ch ## V
#define DMA_SOURCE_FTM0_CHn(ch) DMA_SOURCE_FTM0_CHn2(ch)
#define DMA_SOURCE_FTM0_CHn2(ch) DMAMUX_SOURCE_FTM0_CH ## ch

// Flextimer configuration values
#define FTM_CSC_DMA 0x01 			    // 0000 0001
#define FTM_CSC_PWM_EDGE_HI 0x28 	// 0010 1000
#define FTM_CSC_IRQ 0x40 			    // 0100 0000


// DShot Clock in hz
#define DSHOT_CLOCK 600000

// Bit timing as a % of clock rate
#define DSHOT_0_TIMING  (int)(0.38*255)
#define DSHOT_1_TIMING  (int)(0.75*255)

class DShot
{
	public:

		DShot( uint8_t num );

		void setup( uint8_t num );

		void setThrottle( uint16_t value);

    void setDirection( bool dir );
    void setNormal();

    void beep( uint8_t value );

		void requestTelemetry( void );

    void setTlmPort( Stream * port );
		
	private:

		DMAChannel dma;
		uint8_t dshotBuffer[DSHOT_BUFFER_LENGTH];
		uint32_t mod = (F_BUS + DSHOT_CLOCK / 2) / DSHOT_CLOCK;
    uint32_t trig = 254;
		bool requestTlm = false;

		uint8_t getChecksum(uint16_t value);

		void fillBuffer( uint16_t value);

		void write( uint16_t value);

    Stream * tlmPort = NULL;

};

#endif
