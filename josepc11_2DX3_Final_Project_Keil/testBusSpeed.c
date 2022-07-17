#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "SysTick.h"

void PortM_Init(void){
	//Use PortN onboard LED	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;				// activate clock for Port N
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};	// allow time for clock to stabilize
	GPIO_PORTM_DIR_R |= 0x05;        								// make PN0 out (PN0 built-in LED1)
  GPIO_PORTM_AFSEL_R &= ~0x05;     								// disable alt funct on PN0
  GPIO_PORTM_DEN_R |= 0x05;        								// enable digital I/O on PN0
																									// configure PN1 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF0F)+0x00000000;
  GPIO_PORTM_AMSEL_R &= ~0x05;     								// disable analog functionality on PN0		
	
	return;
}

void TestBusSpeed_Init(){
	PortM_Init();
}
void TestBusSpeed(){
	GPIO_PORTM_DATA_R ^= 0b00000001;
	SysTick_Wait10ms(1);	
}
