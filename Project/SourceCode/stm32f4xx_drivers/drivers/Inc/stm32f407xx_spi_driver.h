/*
 * stm32f4xx_spi_drivers.c
 *
 *  Created on: Dec 13, 2023
 *      Author: Halimulati Sailike
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_


#include "stm32f407xx.h"


/*
 * Configuration structure for SPIx peripheral
 */

typedef struct
{
	uint8_t SPI_DeviceMode;			/* possible values from @SPI_DEVICE_MODE */
	uint8_t SPI_BusConfig;			/* possible values from @SPI_BUS_CONFIG */
	uint8_t SPI_SclkSpeed;			/* possible values from @SPI_SCLK_SPEED */
	uint8_t SPI_DFF;				/* possible values from @SPI_DFF */
	uint8_t SPI_CPOL;				/* possible values from @SPI_CPOL */
	uint8_t SPI_CPHA;				/* possible values from @SPI_CPHA */
	uint8_t SPI_SSM;				/* possible values from @SPI_SSM */
}SPI_Config_t;


/*
 * Handle structure for SPIx peripheral
 */

typedef struct
{
	SPI_RegDef_t 	*pSPIx;				/*This holds the base address of SPIx(x:0,1,2) peripherals*/
	SPI_Config_t 	SPI_Config;			/*This holds SPI configuration settings*/
	uint8_t 		*pTxBuffer; 		/*To store the application Tx buffer address*/
	uint8_t 		*pRxBuffer;			/*To store the application Rx buffer address*/
	uint32_t 		TxLen;				/*To store Tx len*/
	uint32_t 		RxLen;				/*To store Tx len*/
	uint8_t 		TxState;			/*To store Tx state*/
	uint8_t 		RxState;			/*To store Rx state*/
}SPI_Handle_t;


/*
 * SPI application states
 */

#define SPI_READY 					0
#define SPI_BUSY_IN_RX 				1
#define SPI_BUSY_IN_TX 				2


/*
 * Possible SPI Application events
 */
#define SPI_EVENT_TX_CMPLT   1
#define SPI_EVENT_RX_CMPLT   2
#define SPI_EVENT_OVR_ERR    3
#define SPI_EVENT_CRC_ERR    4


/*
 * @SPI_DEVICE_MODE
 * SPI device mode (master or slave)
 */

#define SPI_DEVICE_MODE_MASTER			1
#define SPI_DEVICE_MODE_SLAVE			0


/*
 * @SPI_BUS_CONFIG
 * SPI bus configuration (full-duplex, half-duplex, simplex)
 */

#define SPI_BUS_CONFIG_FULL_DUPLEX			1
#define SPI_BUS_CONFIG_HALF_DUPLEX			2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		3


/*
 * @SPI_SCLK_SPEED
 * SPI serial clock speed (fPCLK / pre-scaler)
 */

#define SPI_SCLK_SPEED_DIV2			0
#define SPI_SCLK_SPEED_DIV4			1
#define SPI_SCLK_SPEED_DIV8			2
#define SPI_SCLK_SPEED_DIV16		3
#define SPI_SCLK_SPEED_DIV32		4
#define SPI_SCLK_SPEED_DIV64		5
#define SPI_SCLK_SPEED_DIV128		6
#define SPI_SCLK_SPEED_DIV256		7



/*
 * @SPI_DFF
 * SPI data frame format (by default: 8)
 */

#define SPI_DFF_8BITS		0
#define SPI_DFF_16BITS		1


/*
 * @SPI_CPOL
 * SPI clock polarity (by default: 0)
 */

#define SPI_CPOL_LOW		0
#define SPI_CPOL_HIGH		1


/*
 * @SPI_CPHA
 * SPI clock phase (by default: 0)
 */

#define SPI_CPHA_LOW		0
#define SPI_CPHA_HIGH		1


/*
 * @SPI_SSM
 * SPI software slave management (by default: SW)
 */

#define SPI_SSM_EN		1
#define SPI_SSM_DI		0


/*
 * SPI related status flags definitions
 */

#define SPI_TXE_FLAG		(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG		(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG		(1 << SPI_SR_BSY)



/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/


/*
 * Peripheral Clock Setup
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);


/*
 * Init and De_Init
 */

void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);


/*
 * Data Send and Receive
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);


/*
 * IRQ Configuration and ISR handling
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);


/*
 * Other Peripheral Control APIs
 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDI);

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDI);

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDI);

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);

void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle);

void SPI_CloseReception(SPI_Handle_t *pSPIHandle);


/*
 * Application callback
 */

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv);



#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
