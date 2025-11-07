/*
 * FreeRTOS+IO V1.0.1 (C) 2012 Real Time Engineers ltd.
 *
 * FreeRTOS+IO is an add-on component to FreeRTOS.  It is not, in itself, part 
 * of the FreeRTOS kernel.  FreeRTOS+IO is licensed separately from FreeRTOS, 
 * and uses a different license to FreeRTOS.  FreeRTOS+IO uses a dual license
 * model, information on which is provided below:
 *
 * - Open source licensing -
 * FreeRTOS+IO is a free download and may be used, modified and distributed
 * without charge provided the user adheres to version two of the GNU General
 * Public license (GPL) and does not remove the copyright notice or this text.
 * The GPL V2 text is available on the gnu.org web site, and on the following
 * URL: http://www.FreeRTOS.org/gpl-2.0.txt
 *
 * - Commercial licensing -
 * Businesses and individuals who wish to incorporate FreeRTOS+IO into
 * proprietary software for redistribution in any form must first obtain a low
 * cost commercial license - and in-so-doing support the maintenance, support
 * and further development of the FreeRTOS+IO product.  Commercial licenses can
 * be obtained from http://shop.freertos.org and do not require any source files
 * to be changed.
 *
 * FreeRTOS+IO is distributed in the hope that it will be useful.  You cannot
 * use FreeRTOS+IO unless you agree that you use the software 'as is'.
 * FreeRTOS+IO is provided WITHOUT ANY WARRANTY; without even the implied
 * warranties of NON-INFRINGEMENT, MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE. Real Time Engineers Ltd. disclaims all conditions and terms, be they
 * implied, expressed, or statutory.
 *
 * 1 tab == 4 spaces!
 *
 * http://www.FreeRTOS.org
 * http://www.FreeRTOS.org/FreeRTOS-Plus
 *
 */

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* IO library includes. */
#include "FreeRTOS_IO.h"
#include "IOUtils_Common.h"
#include "FreeRTOS_uart.h"


typedef struct rtos_objects_t_ {
	xQueueHandle qUART;
	// Declares a semaphore structure for the UART
	xSemaphoreHandle sUART;
	// Declares a mutex structure for the UART
	xSemaphoreHandle mUARTTx;
	uint8_t data_rx;
}rtos_objects_t;

rtos_objects_t rtos_objects[boardNUM_UARTS];

UART_HandleTypeDef hlpuart1;

/* The bits in the FIFOLVL register that represent the Tx Fifo level. */
#define uartTX_FIFO_LEVEL_MASK		( 0xf00UL )

/* The TEMT bit in the line status register. */
#define uartTX_BUSY_MASK			( 1UL << 6UL )

/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*/

/* Stores the transfer control structures that are currently in use by the
supported UART ports. */
//static Transfer_Control_t *pxTxTransferControlStructs[ boardNUM_UARTS ] = { NULL };
//static Transfer_Control_t *pxRxTransferControlStructs[ boardNUM_UARTS ] = { NULL };

/* Stores the IRQ numbers of the supported UART ports. */
//static const IRQn_Type xIRQ[] = { UART0_IRQn, UART1_IRQn, UART2_IRQn, UART3_IRQn };

/*-----------------------------------------------------------*/

portBASE_TYPE init_UART_rtos_objects(uint8_t cPeripheralNumber){
		cPeripheralNumber--;
		rtos_objects[cPeripheralNumber].sUART = xSemaphoreCreateBinary();

		if( rtos_objects[cPeripheralNumber].sUART == NULL )
		{
			/* There was insufficient FreeRTOS heap available for the semaphore to
			be created successfully. */
			return pdFAIL;
		}
		else
		{
			rtos_objects[cPeripheralNumber].mUARTTx = xSemaphoreCreateMutex();
			if( rtos_objects[cPeripheralNumber].mUARTTx == NULL )
			{
				/* There was insufficient FreeRTOS heap available for the semaphore to
				be created successfully. */
				vSemaphoreDelete( rtos_objects[cPeripheralNumber].sUART);
				return pdFAIL;
			}else
			{
				rtos_objects[cPeripheralNumber].qUART = xQueueCreate(128, sizeof(char));
				if( rtos_objects[cPeripheralNumber].qUART == NULL )
				{
					/* There was insufficient FreeRTOS heap available for the queue to
					be created successfully. */
					vSemaphoreDelete( rtos_objects[cPeripheralNumber].sUART);
					vSemaphoreDelete( rtos_objects[cPeripheralNumber].mUARTTx);
					return pdFAIL;
				}
			}
		}
		return pdPASS;
}

void boardCONFIGURE_UART_PINS(const uint8_t cPeripheralNumber){
	  GPIO_InitTypeDef GPIO_InitStruct = {0};
	  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
	  if (cPeripheralNumber == 1)
	  {
			/** Initializes the peripherals clocks
			*/
		  	LPUART1_CLOCK_INIT();
			GPIO_InitStruct.Pin = LPUART1_TX_Pin|LPUART1_RX_Pin;
			GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
			GPIO_InitStruct.Pull = GPIO_NOPULL;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			GPIO_InitStruct.Alternate = LPUART1_PINS_ALTERNATE_FUNCTIONS;
			HAL_GPIO_Init(LPUART1_TX_GPIO_Port, &GPIO_InitStruct);

			/* LPUART1 interrupt Init */
			HAL_NVIC_SetPriority(LPUART1_IRQ_NUMBER, 5, 0);
			HAL_NVIC_EnableIRQ(LPUART1_IRQ_NUMBER);
	  }
}


portBASE_TYPE FreeRTOS_UART_open( Peripheral_Control_t * const pxPeripheralControl )
{
portBASE_TYPE xReturn;
//uint32_t UART_BASE_ADDR = (uint32_t)diGET_PERIPHERAL_BASE_ADDRESS( pxPeripheralControl );
const uint8_t cPeripheralNumber = diGET_PERIPHERAL_NUMBER( pxPeripheralControl );
	pxPeripheralControl->read = FreeRTOS_UART_read;
	pxPeripheralControl->write = FreeRTOS_UART_write;
	pxPeripheralControl->ioctl = FreeRTOS_UART_ioctl;

	/* Sanity check the peripheral number. */
	if( cPeripheralNumber < boardNUM_UARTS )
	{
		if (init_UART_rtos_objects(cPeripheralNumber) == pdFAIL){
			return pdFAIL;
		}

		/* Setup the pins for the UART being used. */
		boardCONFIGURE_UART_PINS(cPeripheralNumber);
		UART_HandleTypeDef *pxUART = (UART_HandleTypeDef *) diGET_PERIPHERAL_BASE_ADDRESS( pxPeripheralControl );

		pxUART->Instance = LPUART1_BASE_ADDRESS;
		pxUART->Init.BaudRate = boardDEFAULT_UART_BAUD;
		pxUART->Init.WordLength = UART_WORDLENGTH_8B;
		pxUART->Init.StopBits = UART_STOPBITS_1;
		pxUART->Init.Parity = UART_PARITY_NONE;
		pxUART->Init.Mode = UART_MODE_TX_RX;
		pxUART->Init.HwFlowCtl = UART_HWCONTROL_NONE;
		pxUART->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
		pxUART->Init.ClockPrescaler = UART_PRESCALER_DIV1;
		pxUART->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
		if (HAL_UART_Init(pxUART) != HAL_OK)
		{
			return pdFAIL;
		}
		if (HAL_UARTEx_SetTxFifoThreshold(pxUART, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
		{
			return pdFAIL;
		}
		if (HAL_UARTEx_SetRxFifoThreshold(pxUART, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
		{
			return pdFAIL;
		}
		if (HAL_UARTEx_DisableFifoMode(pxUART) != HAL_OK)
		{
			return pdFAIL;
		}

		if (HAL_UART_Receive_IT(pxUART, &rtos_objects[cPeripheralNumber-1].data_rx, 1) != HAL_OK){
			return pdFAIL;
		}

		xReturn = pdPASS;
	}
	else
	{
		xReturn = pdFAIL;
	}

	return xReturn;
}
/*-----------------------------------------------------------*/

extern UART_HandleTypeDef hlpuart1;
size_t FreeRTOS_UART_write( Peripheral_Descriptor_t const pxPeripheral, const void *pvBuffer, const size_t xBytes )
{
	UART_HandleTypeDef *pxUART = (UART_HandleTypeDef *) diGET_PERIPHERAL_BASE_ADDRESS( ( ( Peripheral_Control_t * const ) pxPeripheral ) );
	uint8_t cPeripheralNumber = diGET_PERIPHERAL_NUMBER( ( Peripheral_Control_t * const )pxPeripheral );
	cPeripheralNumber--;

	uint32_t len = 0;
	if (!xBytes){
		len = strlen(pvBuffer);
	}else{
		len = xBytes;
	}

	if (rtos_objects[cPeripheralNumber].mUARTTx != NULL)
	{
		if (xSemaphoreTake(rtos_objects[cPeripheralNumber].mUARTTx, 100) == pdTRUE){
			if (HAL_UART_Transmit_IT(pxUART, pvBuffer, len) == HAL_OK){
				xSemaphoreTake(rtos_objects[cPeripheralNumber].sUART, portMAX_DELAY);
			}
			xSemaphoreGive(rtos_objects[cPeripheralNumber].mUARTTx);
		}
	}

	return pdPASS;
}
/*-----------------------------------------------------------*/

size_t FreeRTOS_UART_read( Peripheral_Descriptor_t const pxPeripheral, void * const pvBuffer, const size_t xBytes )
{
	uint8_t cPeripheralNumber = diGET_PERIPHERAL_NUMBER( ( Peripheral_Control_t * const )pxPeripheral );
	cPeripheralNumber--;

	return xQueueReceive(rtos_objects[cPeripheralNumber].qUART, ( uint8_t * ) pvBuffer, ( size_t ) portMAX_DELAY);

	return pdPASS;
}

portBASE_TYPE FreeRTOS_UART_ioctl( Peripheral_Descriptor_t pxPeripheral, uint32_t ulRequest, void *pvValue )
{
	portBASE_TYPE xReturn = pdPASS;
	UART_HandleTypeDef *pxUART = (UART_HandleTypeDef *) diGET_PERIPHERAL_BASE_ADDRESS( ( ( Peripheral_Control_t * const ) pxPeripheral ) );
	uint8_t cPeripheralNumber = diGET_PERIPHERAL_NUMBER( ( Peripheral_Control_t * const )pxPeripheral );
	cPeripheralNumber--;
	switch( ulRequest )
	{
		case ioctlSET_SPEED:
			HAL_UART_DeInit(pxUART);
			pxUART->Init.BaudRate = *(uint32_t *)pvValue;
			if (HAL_UART_Init(pxUART) != HAL_OK){
				xReturn = pdFAIL;
			}
			if (HAL_UART_Receive_IT(pxUART, &rtos_objects[cPeripheralNumber].data_rx, 1) != HAL_OK){
				return pdFAIL;
			}
			break;
		case ioctlSET_UART_NUMBER_OF_STOP_BITS:
			HAL_UART_DeInit(pxUART);
			pxUART->Init.StopBits = *(uint32_t *)pvValue;
			if (HAL_UART_Init(pxUART) != HAL_OK){
				xReturn = pdFAIL;
			}
			if (HAL_UART_Receive_IT(pxUART, &rtos_objects[cPeripheralNumber].data_rx, 1) != HAL_OK){
				return pdFAIL;
			}
			break;
		case ioctlSET_UART_PARITY_MODE:
			HAL_UART_DeInit(pxUART);
			pxUART->Init.Parity = *(uint32_t *)pvValue;
			if (HAL_UART_Init(pxUART) != HAL_OK){
				xReturn = pdFAIL;
			}
			if (HAL_UART_Receive_IT(pxUART, &rtos_objects[cPeripheralNumber].data_rx, 1) != HAL_OK){
				return pdFAIL;
			}
			break;
		default:
			xReturn = pdFAIL;
			break;
	}

	return xReturn;
}

/*-----------------------------------------------------------*/
void LPUART1_IRQHandler(void)
{
  HAL_UART_IRQHandler(&hlpuart1);
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(rtos_objects[0].sUART, &pxHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
	xQueueSendToBackFromISR(rtos_objects[0].qUART, huart->pRxBuffPtr, &pxHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}
