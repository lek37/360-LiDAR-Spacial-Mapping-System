//----------------------------------------------------------------
// COMPENG 2DX3 - Microprocessor System Project
// Final Project: 360 LiDAR Spatial Mapping System
// Student Number: 400558145
// Name: Mark Le
// ---------------------------------------------------------------
// ASSIGNED PARAMETERS
// Bus frequency: 26 MHz
// Measurement status: PN1
// UART Status: PF0
// Additional status: PF4 (for direction of spin: CW: off, CCW: on)
//----------------------------------------------------------------
// HARDWARE SETUP: 
// Stepper motor: IN1 goes to PH0, IN2 goes to PH1, IN3 goes to PH2, IN4 goes to PH3, - goes to GND, + goes to 5V on the MCU
// Button 1 start/stop: connect to PL0 GPIO Pin, with a physical pull-up resistor of 1kOhm, with V_CC = 3.3V
// Button 2 home button: connect to PM0 GPIO Pin, with a physical pull-up resistor of 1kOhm, with V_CC = 3.3V
// Can be done without using physical pull-up resistors. Enable it in the port initialization. 
// ToF sensor: V_IN goes to 3.3V, GND goes to GND on MCU, SDA goes to PB3, SCL goes to PB2. 
// Detail picture of the hardware setup can be found in the project documentation (the final report)
//-----------------------------------------------------------------

// INCLUDE HEADER FILES
#include <stdint.h>
#include <stdio.h>
#include "tm4c1294ncpdt.h"
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"

// SOME CONSTANTS
#define CW 0
#define CCW 1
#define TOF_ADDR 0x29
#define STEPS_PER_11P25_DEG 64

// PORT INITIALIZATION
void PortM_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;
	while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R11) == 0){}

	GPIO_PORTM_DIR_R &= ~0x03;     // PM1 input
	GPIO_PORTM_AFSEL_R &= ~0x03;
	GPIO_PORTM_DEN_R |= 0x03;
	GPIO_PORTM_AMSEL_R &= ~0x03;
	return;
}

void PortN_Init(void){
	//Use PortN pins (PN0-PN1) for output
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;				// activate clock for Port M
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R10) == 0){};	// allow time for clock to stabilize
	GPIO_PORTN_DIR_R |= 0x03;        								
  	GPIO_PORTN_AFSEL_R &= ~0x03;     								
  	GPIO_PORTN_DEN_R |= 0x03;        								
																									
  	GPIO_PORTN_AMSEL_R &= ~0x03;     							
	return;
}

void PortH_Init(void){
	// Use Port H pin PH0 - PH3 for output
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;				// activate clock for Port H
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};	// allow time for clock to stabilize
	GPIO_PORTH_DIR_R |= 0x0F;        								// configure Port H pins (PM0-PM3) as output
  	GPIO_PORTH_AFSEL_R &= ~0x0F;     								// disable alt funct on Port M pins (PH0-PH3)
  	GPIO_PORTH_DEN_R |= 0x0F;        								// enable digital I/O on Port M pins (PH0-PH3)
																									// configure Port H as GPIO
  	GPIO_PORTH_AMSEL_R &= ~0x0F;     								// disable analog functionality on Port H	pins (PH0-PH3)	
	return;
}

void PortJ_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;				// activate clock for Port J
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};	// allow time for clock to stabilize
	GPIO_PORTJ_DIR_R &= ~0x03;        								// configure Port M pins (PJ0-PJ1) as input
  	GPIO_PORTJ_AFSEL_R &= ~0x03;     								// disable alt funct on Port M pins (PJ0-PJ3)
  	GPIO_PORTJ_DEN_R |= 0x03;        								// enable digital I/O on Port M pins (PJ0-PJ3)
																									// configure Port M as GPIO
  	GPIO_PORTJ_AMSEL_R &= ~0x03;     								// disable analog functionality on Port M	pins (PJ0-PJ3)
	GPIO_PORTJ_PUR_R |= 0x03;					// enable pull up resistors
	return;
}

void PortF_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
	while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5) == 0){}

	GPIO_PORTF_DIR_R |= 0x13;      // PF4, PF1 and PF0 output
	GPIO_PORTF_AFSEL_R &= ~0x13;
	GPIO_PORTF_DEN_R |= 0x13;
	GPIO_PORTF_AMSEL_R &= ~0x13;
	return;
}

void PortL_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R10;
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R10) == 0){}
	GPIO_PORTL_DIR_R &= ~0x01;      // PL0 input for start/stop button
	GPIO_PORTL_AFSEL_R &= ~0x01;
	GPIO_PORTL_DEN_R |= 0x01;
	GPIO_PORTL_AMSEL_R &= ~0x01;
	return;
}

void PortK_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R9;
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R9) == 0){}
	GPIO_PORTK_DIR_R |= 0x01;       // PK0 output for bus speed test
	GPIO_PORTK_AFSEL_R &= ~0x01;
	GPIO_PORTK_DEN_R |= 0x01;
	GPIO_PORTK_AMSEL_R &= ~0x01;
	GPIO_PORTK_DATA_R &= ~0x01;     // start low
	return;
}

// I2C Initialization
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        						// 8) configure for 100 kbps clock
}      

// VL53L1X:
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  	GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  	GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  	//GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  	GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.

void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}

// STEPPER MOTOR SPIN & HALT
void cw_step(void){
  static uint8_t step = 0;
  uint8_t seq[4] = {0b00000011, 0b00000110, 0b00001100, 0b00001001};

  GPIO_PORTH_DATA_R = seq[step];
  step = (step + 1) % 4;

  SysTick_Wait10us(1000);   
}

void ccw_step(void){
	static uint8_t step = 0;
	uint8_t seq[4] = {0b00001001, 0b00001100, 0b00000110, 0b00000011};
	 
	GPIO_PORTH_DATA_R = seq[step];
	step = (step + 1) % 4;
	
	SysTick_Wait10us(1000);
}

void motor_off(void){
  GPIO_PORTH_DATA_R = 0x00;
}

// STATUS LED FLASH
// Flash LED D1, for indicating measurement status
void Flash_LED_D1(void){
    GPIO_PORTN_DATA_R |= 0b00000010;
    SysTick_Wait10ms(10);
	GPIO_PORTN_DATA_R &= ~0b00000010;
}

// Flash LED D4, for indicating UART tranmission, control by PF0
void Flash_LED_D4(void){
    GPIO_PORTF_DATA_R |= 0b00000001;
    SysTick_Wait10ms(10);
    GPIO_PORTF_DATA_R &= ~0b00000001;
}

// Toggle PK0 for a bus-speed check on the Analog Discovery 3.
void measure_bus(){
    while(1){
        GPIO_PORTK_DATA_R ^= 0b00000001; // toggle PK0
		SysTick_Wait(200000); // toggle every 200 ms. 
    }
}




// MAIN FUNCTION
int main(void){
	PLL_Init();
	SysTick_Init();
	PortL_Init();
	PortM_Init();
	PortN_Init();
	PortH_Init();
	PortJ_Init();
	PortF_Init();
	PortG_Init();
	PortK_Init();

	// I2C and UART initialization
	I2C_Init();
	UART_Init();

	// UNCOMMENT THIS LINE TO TEST THE BUS SPEED.
	//************************ 
	// measure_bus(); 
	//************************


	uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
	uint16_t wordData;
	uint16_t Distance;
	uint16_t SignalRate;
	uint16_t AmbientRate;
	uint16_t SpadNum; // comdfk
	uint8_t RangeStatus;
	uint8_t dataReady = 0;
	int status;
	uint16_t dev = TOF_ADDR;

	uint8_t motor_on = 0; // default: no spin. Press PL0 to start/stop
	uint8_t direction = CW; // default: spin clockwise

	uint8_t prev_b0;
	uint8_t prev_b2;

	uint32_t stepCount = 0; 
	// track step taken in a 360 degree spin 
	// (at 0 deg, stepCount = 0, at 180 deg, stepCount = 1024, at 360 deg, stepCount = 2048)

	int32_t position = 0; // track current position of the motor
	uint32_t depth = 0;
	uint32_t captureCount;
	uint32_t angleTimes100;

	UART_printf("Program Begins\r\n");

	while(sensorState == 0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
	}
	status = VL53L1X_ClearInterrupt(dev);
	status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);
	status = VL53L1X_StartRanging(dev);


	prev_b0 = GPIO_PORTL_DATA_R & 0x01;
	prev_b2 = GPIO_PORTM_DATA_R & 0x01;

	

	while(1){
		uint8_t b0 = GPIO_PORTL_DATA_R & 0x01;         // PL0
		uint8_t b2 = GPIO_PORTM_DATA_R & 0x01; 	// PM0

		// PL0 pressed: start/stop motor
		if((prev_b0 == 1) && (b0 == 0)){
			SysTick_Wait10us(2000);   // debounce
			if((GPIO_PORTL_DATA_R & 0x01) == 0){
				if(motor_on == 0){
					motor_on = 1;
					stepCount = 0;
				}else{
					motor_on = 0;
					motor_off();
				} 
			}
		}
		
		// PM0 pressed: return to home position, no more measurement taken. 
		if((prev_b2 == 1) && (b2 == 0)){
			SysTick_Wait10us(2000); 
			if((GPIO_PORTM_DATA_R & 0x01) == 0){
				motor_on = 0;
				GPIO_PORTF_DATA_R &= ~0b00010000;
				while(position > 0){
					ccw_step();
					position--;
				}
				
				while(position < 0){
					cw_step();
					position++;
				}
				
				motor_on = 0;
				direction = CW;
				stepCount = 0;
				motor_off();
			}
		}
		
		prev_b0 = b0;
		prev_b2 = b2;

		if(motor_on){
			if(direction == CW){
				cw_step();
				GPIO_PORTF_DATA_R &= ~0b00010000; 
				position++;
			}else{
				ccw_step(); // rotate counter clockwise
				GPIO_PORTF_DATA_R |= 0b00010000;
				position--; // go back to the user defined position. 
			}

			stepCount++; // tracking variable of the motor spin
			

			// take measurement and output using UART on Realterm for every 11.25 degree of spin
			if ((stepCount % STEPS_PER_11P25_DEG) == 0){
				while (dataReady == 0){
					status = VL53L1X_CheckForDataReady(dev, &dataReady);
					VL53L1_WaitMs(dev, 5);
				}
				dataReady = 0;

				status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
				status = VL53L1X_GetDistance(dev, &Distance);
				Flash_LED_D1();
				status = VL53L1X_GetAmbientRate(dev, &AmbientRate);
				status = VL53L1X_GetSpadNb(dev, &SpadNum);
				status = VL53L1X_ClearInterrupt(dev);

				captureCount = stepCount / STEPS_PER_11P25_DEG; // measurement index within one revolution
				if (direction == CW) {
					angleTimes100 = captureCount * 1125; // 11.25, 22.50, ..., 360.00
				} else {
					// Reverse sweep should report 360.00, 348.75, ..., 11.25 in the MATLAB output terminal
					angleTimes100 = 36000 - ((captureCount - 1) * 1125);
				}

				sprintf(printf_buffer,
								"%u.%02u,  %u,  %u ,  %u,  %u,  %u\r\n",
								(unsigned int)(angleTimes100 / 100), 
								(unsigned int)(angleTimes100 % 100),
								RangeStatus,
								Distance,
								depth,
								AmbientRate,
								SpadNum);
				Flash_LED_D4();				
				UART_printf(printf_buffer); 
			}
			
			
			if(stepCount >= 2048){
				depth = depth + 700; // take 10 steps, hallway is 7 meters long, measurement taken in mm. 
				stepCount = 0;
				if(direction == CW){
					direction = CCW;
					GPIO_PORTF_DATA_R |= 0b00010000;
				}else{
					direction = CW;
					GPIO_PORTF_DATA_R &= ~0b00010000;
				}
			}
		}else{
			motor_off();
			GPIO_PORTF_DATA_R &= ~0b00010000;
		}
	}
}