/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Dec 13, 2023
 *      Author: Halimulati Sailike
 */

#include "stm32f407xx_gpio_driver.h"


/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 *********************************************************************/

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}else
	{
		if(pGPIOx == GPIOA)
			{
				GPIOA_PCLK_DI();
			}else if(pGPIOx == GPIOB)
			{
				GPIOB_PCLK_DI();
			}else if(pGPIOx == GPIOC)
			{
				GPIOC_PCLK_DI();
			}else if(pGPIOx == GPIOD)
			{
				GPIOD_PCLK_DI();
			}else if(pGPIOx == GPIOE)
			{
				GPIOE_PCLK_DI();
			}else if(pGPIOx == GPIOF)
			{
				GPIOF_PCLK_DI();
			}else if(pGPIOx == GPIOG)
			{
				GPIOG_PCLK_DI();
			}else if(pGPIOx == GPIOH)
			{
				GPIOH_PCLK_DI();
			}else if(pGPIOx == GPIOI)
			{
				GPIOI_PCLK_DI();
			}
	}
}


/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - This function initializes the given GPIO port and the given GPIO pin
 *
 * @param[in]         - base address of the GPIO handle structure
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 *********************************************************************/

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	uint32_t temp = 0;		// temporary register

	//1. Configure the mode of GPIO pin

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//1.0 non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));	//clearing
		pGPIOHandle->pGPIOx->MODER |= temp;		//setting

	}else
	{
		//1.1 interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			// Configure the FTSR (Falling Trigger Selection Register)
			EXTI->EXTI_FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clear the corresponding RTSR bit
			EXTI->EXTI_RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			// Configure the RTSR (Rising Trigger Selection Register)
			EXTI->EXTI_RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clear the corresponding RTSR bit
			EXTI->EXTI_FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			// Configure both FTSR and RTSR
			EXTI->EXTI_FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			EXTI->EXTI_RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}

		//1.2 Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->SYSCFG_EXTICR[temp1] = portcode << (temp2 * 4);

		//1.3 Enable the EXTI interrupt delivery using IMR(Interrupt Masking Register)
		EXTI->EXTI_IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}


	//2. Configure the speed

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));	//clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;	//setting

	//3. Configure the pull up pull down settings

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));	//clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;	//setting

	//4. Configure the output type

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	//clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;	//setting

	//5. Configure the alternate functionality

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		// Configure the alternate function register
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));	//clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));	//setting
	}

}


/*********************************************************************
 * @fn      		  - GPIO_Deinit
 *
 * @brief             - This function de-initializes the given GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 *********************************************************************/

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
		{
			GPIOA_REG_RESET();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_REG_RESET();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_REG_RESET();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_REG_RESET();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_REG_RESET();
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_REG_RESET();
		}else if (pGPIOx == GPIOG)
		{
			GPIOG_REG_RESET();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_REG_RESET();
		}else if (pGPIOx == GPIOI)
		{
			GPIOI_REG_RESET();
		}
}


/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             - This function reads data from a Pin of the given GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - Pin number
 * @param[in]         -
 *
 * @return            -  value reads from the corresponding Pin number
 *
 * @Note              -  none
 *********************************************************************/

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;

	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);

	return value;
}


/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             - This function reads data from the given GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  value reads from the corresponding GPIO port
 *
 * @Note              -  none
 *********************************************************************/

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
		uint16_t value;

		value = (uint16_t)pGPIOx->IDR;

		return value;
}


/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             - This function writes data to the Pin of the given GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - Pin number
 * @param[in]         -	Write value
 *
 * @return            -  none
 *
 * @Note              -  none
 *********************************************************************/

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		// Write 1 to the output data register at the bit field corresponding to the Pin number
		pGPIOx->ODR |= (1 << PinNumber);
	}else
	{	// Write 0
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}


/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             - This function writes data to the given GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - Write value
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 *********************************************************************/

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}


/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             - This function toggles data of a Pin of the given GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - Pin number
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 *********************************************************************/

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}


/*********************************************************************
 * @fn      		  - GPIO_IRQInterrupt_Config
 *
 * @brief             - This function configures enabling and disabling
 * 						of an interrupt according to IRQ numbers
 * @param[in]         - IRQ numbers
 * @param[in]         - IRQ priority
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            -  none
 *
 * @Note              -  none
 *********************************************************************/

void GPIO_IRQInterrupt_Config(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			// Program ISER0 register
			*NVIC_ISER0_BASEADDR |= (1 << IRQNumber);

		}else if(IRQNumber > 32 && IRQNumber <= 63)
		{
			// Program ISER1 register
			*NVIC_ISER0_BASEADDR |= (1 << (IRQNumber % 32));

		}else if(IRQNumber > 63 && IRQNumber <= 95)
		{
			// Program ISER2 register
			*NVIC_ISER0_BASEADDR |= (1 << (IRQNumber % 64));
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			// Program ICER0 register
			*NVIC_ICER0_BASEADDR |= (1 << IRQNumber);

		}else if(IRQNumber > 32 && IRQNumber <= 63)
		{
			// Program ICER1 register
			*NVIC_ICER0_BASEADDR |= (1 << (IRQNumber % 32));

		}else if(IRQNumber > 63 && IRQNumber <= 95)
		{
			// Program ICER2 register
			*NVIC_ICER0_BASEADDR |= (1 << (IRQNumber % 32));
		}
	}
}


/*********************************************************************
 * @fn      		  - GPIO_IRQPriority_Config
 *
 * @brief             - This function configures the interrupt priority
 *						according to IRQ numbers
 * @param[in]         - IRQ numbers
 * @param[in]         - IRQ priority
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 *********************************************************************/

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. Find out the IPR register
	uint8_t iprx = IRQNumber / 4;
	//2. Find out the corresponding section(bit field) to be programmed
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}


/*********************************************************************
 * @fn      		  - GPIO_IRQ_Handling
 *
 * @brief             - This function implements the ISR(Interrupt Service Routine)
 *
 * @param[in]         - pin number
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 *********************************************************************/

void GPIO_IRQ_Handling(uint8_t PinNumber)
{
	// Clear the EXTI pending register corresponding to the pin number
	if(EXTI->EXTI_PR & (1 << PinNumber))
	{
		// Clear
		EXTI->EXTI_PR |= (1 << PinNumber);
	}
}
