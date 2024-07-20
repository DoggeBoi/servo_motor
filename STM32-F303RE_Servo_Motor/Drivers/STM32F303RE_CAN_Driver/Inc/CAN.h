
#ifndef CAN_INC_CAN_H
#define CAN_INC_CAN_H

#include "stm32f3xx_hal.h"
#include "cmsis_os.h"

/*   Define message priorities   */
#define PRIORITY_CRITICAL		0
#define PRIORITY_HIGH			1
#define PRIORITY_MODERATE		2
#define PRIORITY_LOW			3

typedef struct {

	/*   CAN bus handle   */
	CAN_HandleTypeDef *canHandle;

	/*   Device IDs   */
	uint8_t canDeviceID;
	uint8_t canMasterID;

	/*   CAN bus transmission mailbox   */
	uint32_t canTxMailbox[3];

	/*   freeRTOS task IDs   */
	osThreadId taskFIFO0;
	osThreadId taskFIFO1;

} CANBUS;

/*   Initialisation   */
uint8_t CAN_Init(CANBUS *canbus, CAN_HandleTypeDef *canHandle, uint8_t canDeviceID, uint8_t canMasterID);


/*   Activate CAN callbacks   */
void CAN_ActivateCallback(CANBUS *canbus);


/*   Sets filter parameters   */
uint8_t CAN_SetFilter(CANBUS *canbus);


/*   Add data frame to Tx mailbox   */
void CAN_SendDataFrame(CANBUS *canbus, uint8_t *TxData, uint8_t dataLenght, uint8_t priority, uint8_t operationId);


/*   Unpack frame from Rx mailbox*/
void CAN_GetFrame(CANBUS *canbus, uint32_t RxFifo, uint8_t *RxData, uint8_t *senderId, uint8_t *operationId, uint8_t *priority);


#endif /* CAN_INC_CAN_H */


