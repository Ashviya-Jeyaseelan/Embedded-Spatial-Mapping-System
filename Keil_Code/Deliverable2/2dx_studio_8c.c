// COMPENG 2DX3 Project Deliverable 2

// Written by Ashviya Jeyaseelan

// Student Number: 400450142
//-----------------------------
// 1st LSB: 2
// Assigned Bus Speed: 48 MHz
//-----------------------------
// 2nd LSB: 4
// Measurement Status LED: PF4
// Additional Status LED: PN1

#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"

#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up

#define DELAY										1				// stepper motor delay

void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
                                                                            // 6) configure PB2,3 as I2C

		GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    			//TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                								// activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    										// allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        				// make PG0 in (HiZ)
		GPIO_PORTG_AFSEL_R &= ~0x01;                                     				// disable alt funct on PG0
		GPIO_PORTG_DEN_R |= 0x01;                                        				// enable digital I/O on PG0
                                                                            // configure PG0 as GPIO
    GPIO_PORTG_AMSEL_R &= ~0x01;                                     				// disable analog functionality on PN0

    return;
}

//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        				// make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 				//PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            	// make PG0 input (HiZ)
    
}

// PortH Pins for Stepper Motor Output
void PortH_Init(void){	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;		              // Activate the clock for Port E
	while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R7) == 0){};	      // Allow time for clock to stabilize
	GPIO_PORTH_DIR_R = 0b00001111;														// Configure Port H pins (PH0-PH3) as output
	GPIO_PORTH_DEN_R = 0b00001111;                        		// Enable PH0-PH3 as digital I/O pins
	return;
}

// PortM Pins for Button Input
void PortM_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;									// Activate clock for Port M
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};				// Allow time for clock to stabilize
	GPIO_PORTM_DIR_R |= 0b00000000;        										// Configure Port M pins PM0 as input
  GPIO_PORTM_DEN_R |= 0b00000001;        										// Enable digital I/O on Port M pins PM0
	return;
}

// Port L for External LED (button debugging)
void PortL_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R10;                 // Activate the clock for Port L
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R10) == 0){};				// Allow time for clock to stabilize
		
	GPIO_PORTL_DIR_R=0b00000001;															// Enable PL0 outputs													
	GPIO_PORTL_DEN_R=0b00000001;															// Enable PL0 as digital pins
	return;
}

//Enable LED D1, D2. Remember D1 is connected to PN1 and D2 is connected to PN0
void PortN_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;                 // Activate the clock for Port N
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R12) == 0){};				// Allow time for clock to stabilize
		
	GPIO_PORTN_DIR_R=0b00000111;															// Enable PN0 and PN1 as outputs													
	GPIO_PORTN_DEN_R=0b00000111;															// Enable PN0 and PN1 as digital pins
	return;
}

//Enable LED D3, D4. Remember D3 is connected to PF4 and D4 is connected to PF0
void PortF_Init(void){
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;                 	// Activate the clock for Port F
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R5) == 0){};					// Allow time for clock to stabilize
		
	GPIO_PORTF_DIR_R=0b00010001;															// Enable PF0 and PF4 as outputs
	GPIO_PORTF_DEN_R=0b00010001;															// Enable PF0 and PF4 as digital pins
	return;
}

void rotateCW(){
	GPIO_PORTH_DATA_R = 0b00000011;
	SysTick_Wait10ms(DELAY);											
	GPIO_PORTH_DATA_R = 0b00000110;
	SysTick_Wait10ms(DELAY);
	GPIO_PORTH_DATA_R = 0b00001100;
	SysTick_Wait10ms(DELAY);
	GPIO_PORTH_DATA_R = 0b00001001;
	SysTick_Wait10ms(DELAY);
}

void rotateCCW(){
	GPIO_PORTH_DATA_R = 0b00001001;
	SysTick_Wait10ms(DELAY);										
	GPIO_PORTH_DATA_R = 0b00001100;
	SysTick_Wait10ms(DELAY);
	GPIO_PORTH_DATA_R = 0b00000110;
	SysTick_Wait10ms(DELAY);
	GPIO_PORTH_DATA_R = 0b00000011;
	SysTick_Wait10ms(DELAY);
}

void measurementStatus(){ // PF4 (D3)
	GPIO_PORTF_DATA_R ^= 0b00010000;
	SysTick_Wait10ms(DELAY);
	GPIO_PORTF_DATA_R ^= 0b00010000;
}

void additionalStatus(){ // PN1 (D1)
	GPIO_PORTN_DATA_R ^= 0b00000010;
	SysTick_Wait10ms(DELAY);
	GPIO_PORTN_DATA_R ^= 0b00000010;
}


//*********************************************************************************************************
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************
uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;

int main(void) {
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;

	//initialize
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	PortH_Init(); // Stepper motor
	PortM_Init(); // Push button
	PortL_Init(); // External LED
	PortF_Init(); // Onboard LED D3 - PF4 
	PortN_Init(); // Onboard LED D1 - PN1
	
                                                                                                                                                                                                                                                                   
	
	
	// Set LED ports to zero
	GPIO_PORTL_DATA_R = 0x00;
	GPIO_PORTF_DATA_R = 0x00;
	GPIO_PORTN_DATA_R = 0x00;
	
	// hello world!
	UART_printf("Program Begins\r\n");
	
	int mynumber = 1;

 //Those basic I2C read functions can be used to check your own I2C functions 
	status = VL53L1_RdWord(dev, 0x010F, &wordData); //for both model ID and type
	sprintf(printf_buffer,"(Model_ID, Module_Type): 0x%x\r\n", wordData);
	UART_printf(printf_buffer);
	
 /* Wait for device ToF booted */
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
	
	FlashAllLEDs();
	UART_printf("ToF Chip Booted!\r\n Please Wait...\r\n");
	
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
	
  /* Initialize the sensor with the default setting  */
	status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);
	
	
	for (int i = 0; i < 10; i++)
	{
		GPIO_PORTN_DATA_R ^= 0b100;
		additionalStatus();
		SysTick_Wait10ms(50);
		GPIO_PORTN_DATA_R ^= 0b100;
		SysTick_Wait10ms(50);
	}
	
	
  status = VL53L1X_StartRanging(dev);   // This function has to be called to enable the ranging
	
	//variables
	int debounce = 10;
	int error = 2; 
	float currentDegree = 0;
  
	while(1){
		
		if((GPIO_PORTM_DATA_R & 0b00000001) == 0b00000000){ // If button is pushed
			currentDegree = 0; // Reset degrees to zero for each scan
			SysTick_Wait10ms(debounce);
			
			for(int i = 0; i <= 512; i++) // 512 steps for 360 deg
			{
				rotateCW();
				
				// 11.25 degree rotations
				// 360 deg / 11.25 deg = 32
				// 512 steps / 32 = 16 steps
				// 16 steps for 11.25 deg
				
				if(i % 16 == 0 || i == 512)
				{ 
					measurementStatus(); // Measurement LED tracks every distance measurement taken
					error = 1; // Assumes there is invalid data
					while(error != 0)
					{
						
						//5 wait until the ToF sensor's data is ready
						while (dataReady == 0)
						{
							status = VL53L1X_CheckForDataReady(dev, &dataReady);
							VL53L1_WaitMs(dev, 5);
						}
						dataReady = 0;
					
						//7 read the data values from ToF sensor
						status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
						status = VL53L1X_GetDistance(dev, &Distance);							//7 The Measured Distance value
						status = VL53L1X_GetSignalRate(dev, &SignalRate);
						status = VL53L1X_GetAmbientRate(dev, &AmbientRate);
						status = VL53L1X_GetSpadNb(dev, &SpadNum);
						
						status = VL53L1X_ClearInterrupt(dev); /* 8 clear interrupt has to be called to enable next interrupt*/
					
						// print the resulted readings to UART
						sprintf(printf_buffer,"%u, %u, %u, %u, %u, %f\r\n", RangeStatus, Distance, SignalRate, AmbientRate, SpadNum, currentDegree);
						UART_printf(printf_buffer);
						SysTick_Wait10ms(50);
						
						error = RangeStatus; // If the data isn't zero (invalid), then continues going through while loop
					}
					currentDegree += 11.25;
				}
			}
			for(int i = 0; i <= 512; i++) // For detangling purposes
			{
					rotateCCW(); // Returns to original position
			}
		}
	}	
	VL53L1X_StopRanging(dev);	
}