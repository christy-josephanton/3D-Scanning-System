/*  josepc11 2DX3 Final Project

Christy Joseph-Anton
josepc11
400325365
On Baord LED: PF4
System Clock: 48 MHz
April 8, 2022
*/

#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "vl53l1x_api.h"
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "stepperMotor.h"
#include "testBusSpeed.h"

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

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
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
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}


// give clock to Port J and initalize as input GPIO
void PortJ_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;					// activate clock for Port J
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};	// allow time for clock to stabilize
  GPIO_PORTJ_DIR_R &= ~0x02;    										// make PJ1 in 
  GPIO_PORTJ_DEN_R |= 0x02;     										// enable digital I/O on PJ1
	
	GPIO_PORTJ_PCTL_R &= ~0x000000F0;	 								//  configure PJ1 as GPIO 
	GPIO_PORTJ_AMSEL_R &= ~0x02;											//  disable analog functionality on PJ1		
	GPIO_PORTJ_PUR_R |= 0x02;													//	enable weak pull up resistor
}

// Enable interrupts
void EnableInt(void)
{    __asm("    cpsie   i\n");
}

// Disable interrupts
void DisableInt(void)
{    __asm("    cpsid   i\n");
}

// Low power wait
void WaitForInt(void)
{    __asm("    wfi\n");
}

// global variable visible in Watch window of debugger
// increments at least once per button press
volatile unsigned long FallingEdges = 0;

// Interrupt initialization for GPIO Port J IRQ# 51
void PortJ_Interrupt_Init(void){
	
		FallingEdges = 0;             			// initialize counter

	
		GPIO_PORTJ_IS_R = 0;     						// (Step 1) PJ1 is edge-sensitive 
		GPIO_PORTJ_IBE_R = 0;    						//     			PJ1 is not both edges 
		GPIO_PORTJ_IEV_R = 0;    						//     			PJ1 falling edge event 
		GPIO_PORTJ_ICR_R = 0x02;      			// 					clear interrupt flag by setting proper bit in ICR register
		GPIO_PORTJ_IM_R = 0x02;      				// 					arm interrupt on PJ1 by setting proper bit in IM register
    
		NVIC_EN1_R = 0x00080000;           // (Step 2) enable interrupt 51 in NVIC
	
		NVIC_PRI12_R = 0xA0000000; 					// (Step 4) set interrupt priority 5

		EnableInt();           							// (Step 3) Enable Global Interrupt. lets go!
}

int state=0;

void GPIOJ_IRQHandler(void){

	if(state==1)
		state=0;
	else
		state=1;
	
	//SysTick_Wait10ms(5);
	
	GPIO_PORTJ_ICR_R = 0x02;      	// acknowledge flag by setting proper bit in ICR register
	//state=(1+state)%2;
}



//*********************************************************************************************************
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************
uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;

int main(void) {

	uint8_t sensorState=0;
  uint16_t wordData;
  uint16_t Distance;
  uint8_t RangeStatus;
  uint8_t dataReady;

	//initialize
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	Motor_Init();
	PortJ_Init();
	PortJ_Interrupt_Init();
	TestBusSpeed_Init();
	
	int input = 0;
	//wait for the right transmition initiation code
	while(1){
		input = UART_InChar();
		if (input == 'b')
			break;
	}
	
	// hello world!
	UART_printf("Program Begins\r\n");
	
	sprintf(printf_buffer,"josepc11 2DX3 Final Project");
	UART_printf(printf_buffer);


/* Those basic I2C read functions can be used to check your own I2C functions */
	status = VL53L1X_GetSensorId(dev, &wordData);

	sprintf(printf_buffer,"(Model_ID, Module_Type)=0x%x\r\n",wordData);
	UART_printf(printf_buffer);

	// Booting ToF chip
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
	FlashAllLEDs();
	UART_printf("ToF Chip Booted!\r\n Please Wait...\r\n");
	
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
	
  /* This function must to be called to initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);

  /* Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
 status = VL53L1X_SetDistanceMode(dev, 2); /* 1=short, 2=long */
//  status = VL53L1X_SetTimingBudgetInMs(dev, 100); /* in ms possible values [20, 50, 100, 200, 500] */
//  status = VL53L1X_SetInterMeasurementInMs(dev, 200); /* in ms, IM must be > = TB */

	 //enables ranging on ToF sensor
  status = VL53L1X_StartRanging(dev);  

	//10 slices, x values
	int x=0;
	while(x<10){
		
		//function that creates a 50% duty cycle on PM0 with a period of 20ms
		//uses SysTick's 10ms wait function to prove 48Mhz bus speed
		TestBusSpeed();
		
		//starts data collection if button is pressed
		if(state==1){
			
			SysTick_Wait10ms(1);
			//x value increments after button is pushed
			x++;
			//state is set 1 in case interrupt sets state to 0 unintentionally
			state=1;

			//runs for 32 readings in a 360 deg rotation	
			for(int i = 0; i < 32; i++) {
			
				//wait until the ToF sensor's data is ready
				while (dataReady == 0){
					status = VL53L1X_CheckForDataReady(dev, &dataReady);
							VL53L1_WaitMs(dev, 5);
				}
				dataReady = 0;
				
				//read the data values from ToF sensor
				status = VL53L1X_GetRangeStatus(dev, &RangeStatus);   //Range Status Value
				status = VL53L1X_GetDistance(dev, &Distance);					//The Measured Distance value

				//clear interrupt has to be called to enable next interrupt
				status = VL53L1X_ClearInterrupt(dev); 

				// print the distance readings to UART if range status is zero, if not, disregard data and redo it in next loop
				//if(RangeStatus==0){
				
				if(1){
					sprintf(printf_buffer,"%u\r\n", Distance);
					UART_printf(printf_buffer);
					//spins motor 11.25 deg in clockwise direction for next scan
					spin(64);
				}
				else
					i--;
				//if program is to be in idle state, infinite loop
				while(state==0){}
			}
			//reset motor after every 360 deg to avoid tangled wires
			resetMotor(2048);
		}
		//switches state to idle state for next x value scan
		state=0;
	
	}

  //stops ranging on ToF sensor
	VL53L1X_StopRanging(dev);
	
  while(1) {}
}
