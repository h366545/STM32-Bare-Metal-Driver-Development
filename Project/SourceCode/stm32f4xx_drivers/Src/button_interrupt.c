#include "stm32f407xx.h"
#include<string.h>

#define HIGH 			1
#define LOW 			0
#define BTNPRESSED 		LOW

void delay()
{
	// this will introduce ~200ms delay when system clock is 16MHz
	for(int i = 1; i <= 500000/2; i++);
}

int main(void)
{
	GPIO_Handle_t GPIOLed, GPIOBtn;

	memset(&GPIOLed,0,sizeof(GPIOLed));
	memset(&GPIOBtn,0,sizeof(GPIOBtn));

	//This is LED GPIO configuration
	GPIOLed.pGPIOx = GPIOD;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIOLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GPIOLed);


	//This is Button GPIO configuration
	GPIOBtn.pGPIOx = GPIOD;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GPIOBtn);


	//IRQ configurations
	GPIO_IRQInterrupt_Config(IRQ_NO_EXTI3, ENABLE);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI3, NVIC_IRQ_PRI15);

	while(1);

}


void EXTI3_IRQHandler(void)
{
	delay();  //200ms . wait till button de-bouncing gets over
	GPIO_IRQ_Handling(GPIO_PIN_NO_3); //clear the pending event from exti line which is set in the pending register
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_14);
}


