#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "PLL.h"
#include "SysTick.h"
#include "stepperMotor.h"
#include "onboardLEDs.h"


//used for stepper motor
void PortL_Init(void){
	//Use PortL pins for output
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R10;				// activate clock for Port L
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R10) == 0){};	// allow time for clock to stabilize
	GPIO_PORTL_DIR_R |= 0xFF;        								// make Port L output
  GPIO_PORTL_AFSEL_R &= ~0xFF;     								// disable alt funct on Port L
  GPIO_PORTL_DEN_R |= 0xFF;        								// enable digital I/O on Port L
  GPIO_PORTL_AMSEL_R &= ~0xFF;     								// disable analog functionality on PN0		
	return;
}

//Turns on D3
void PortF4_Init(void){
SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5; //activate the clock for Port F
while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R5) == 0){};//allow time for clock to stabilize
GPIO_PORTF_DIR_R=0b00010000; //Make PF4 output, to turn on LED
GPIO_PORTF_DEN_R=0b00010000;
return;
}

void Motor_Init(void){
	PortL_Init();
	PortF4_Init();
}

void spin(int steps){
	int delay=140000;
	
	//list used to hold stepper motor port values
	
	int stepValues[4]={0b00001001,0b00000011,0b00000110,0b00001100};
	int counter=0; //keeps track of index of stepValues list
	int angle =1; //keeps track of angle of stepper motor
	
	//runs 2048 times, 4 steps, 8 times per rotor rotation; 32 steps
	//rotor and shaft have 1:64 ratio; 32x64=2048 steps for 1 motor shaft rotation
	for(int i=0; i<steps; i++){
		
		//if angle is factor of 45deg(or 256steps), turn on LED. 
		if(angle++==steps){
			//FlashLED3(1);
			GPIO_PORTF_DATA_R=GPIO_PORTF_DATA_R|0x10; //turn on LED, or everything with 0 except bit to be turned on
			//resets angle variable
			angle=1;
		}
		
		//outputs steps to GPIO Port L
		GPIO_PORTL_DATA_R = stepValues[counter++];
		SysTick_Wait(delay);

		//if the end of the list is reached, reset indexer
		if(counter==4)
			counter=0;
		//turn off LED if on, AND everything with 1  except bit to be turned off
		GPIO_PORTF_DATA_R=GPIO_PORTF_DATA_R&0b11101111;		
	}
	
GPIO_PORTL_DATA_R = 0b00000000; //turns off all coils so it doesnt heat up


}

void resetMotor(int steps){

	int delay=140000;
	
	//list used to hold stepper motor port values
	int stepValues[4]={0b00000011,0b00001001,0b00001100,0b00000110};
	int counter=0; //keeps track of index of stepValues list
	
	//runs 2048 times, 4 steps, 8 times per rotor rotation; 32 steps
	//rotor and shaft have 1:64 ratio; 32x64=2048 steps for 1 motor shaft rotation
	for(int i=0; i<steps; i++){
		
		//outputs steps to GPIO Port L
		GPIO_PORTL_DATA_R = stepValues[counter++];
		SysTick_Wait(delay);

		//if the end of the list is reached, reset indexer
		if(counter==4)
			counter=0;
		//turn off LED if on, AND everything with 1  except bit to be turned off
		GPIO_PORTF_DATA_R=GPIO_PORTF_DATA_R&0b11101111;		
	}
	
GPIO_PORTL_DATA_R = 0b00000000; //turns off all coils so it doesnt heat up



}
