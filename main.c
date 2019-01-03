#include "stm32l1xx_ll_rcc.h"
#include "stm32l1xx_ll_gpio.h"
#include "stm32l1xx_ll_bus.h"
#include "stm32l1xx_ll_tim.h"
#include "stm32l152xe.h"


void TIM2_IRQHandler(void)
{
	LL_GPIO_TogglePin(GPIOA,LL_GPIO_PIN_5);

	LL_TIM_ClearFlag_UPDATE(TIM2);
}

void main (void)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct;
	LL_TIM_InitTypeDef TIM_InitStruct;

	/*GPIO INIT PA5*/
	GPIO_InitStruct.Pin 		= LL_GPIO_PIN_5;
	GPIO_InitStruct.Mode 		= LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate  = LL_GPIO_AF_0;
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*GPIO INIT PC13*/
	GPIO_InitStruct.Pin 		= LL_GPIO_PIN_13;
	GPIO_InitStruct.Mode 		= LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate  = LL_GPIO_AF_0;
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
	LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*TIMER INIT*/
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_ALL);
	TIM_InitStruct.Prescaler         = 1000;						//Divise CLK timer by 1000
	TIM_InitStruct.CounterMode       = LL_TIM_COUNTERMODE_UP;		//Count positively
	TIM_InitStruct.Autoreload        = 3200;						//Autoreload when timer reaches 3200
	TIM_InitStruct.ClockDivision     = LL_TIM_CLOCKDIVISION_DIV1;	//Timer CLock to CPU clock (32000000Hz)
	LL_TIM_Init(TIM2, &TIM_InitStruct); //Timer Init (IT all CPUCLK/Prescaler/Autoreload => 32Mhz/1000/3200=10Hz => ALL 0.1sec
	LL_TIM_EnableUpdateEvent(TIM2);		//Enable Update event
	LL_TIM_EnableIT_UPDATE(TIM2);		//Enable IT when timer update (autoreload)
	NVIC_EnableIRQ(TIM2_IRQn);			//Enable core Interrupt
	LL_TIM_EnableCounter(TIM2);			//Start Timer

	while(1)
	{
		if(LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_13)) {
			LL_TIM_DisableCounter(TIM2);
		} else {
			LL_TIM_EnableCounter(TIM2);
		}
	}
}
