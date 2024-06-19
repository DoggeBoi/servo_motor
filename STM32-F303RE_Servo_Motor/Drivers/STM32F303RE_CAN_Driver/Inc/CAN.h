
#ifndef CAN_INC_CAN_H
#define CAN_INC_CAN_H

#include "stm32f3xx_hal.h"

/*   Define message priorities   */
#define PRIORITY_CRITICAL		0
#define PRIORITY_HIGH			1
#define PRIORITY_MODERATE		2
#define PRIORITY_LOW			3

/*   Define ID of can master node   */
#define CAN_MASTER_ID           0x00000

typedef struct {

	CAN_HandleTypeDef *canHandle;

	uint8_t canDeviceID;
	uint8_t canMasterID;

	uint32_t canTxMailbox;

} CANBUS;


/*   Initialisation   */
void CAN_Init(CANBUS *canbus, CAN_HandleTypeDef *canHandle, uint8_t canDeviceID);


/*   Sets filter parameters   */
void CAN_SetFilter(CANBUS *canbus);


/*   Add data frame to Tx Mailbox   */
void CAN_SendDataFrame(CANBUS *canbus, uint8_t *TxData, uint8_t dataLenght, uint8_t priority, uint8_t operation);


/*   Send startup confirmation to master device   */
void CAN_SendStartUpFrame(CANBUS *canbus);


/*   Get Rx mailbox frame data and header   */
void CAN_GetFrame(CANBUS *canbus, uint8_t RxMailbox, CAN_RxHeaderTypeDef *RxHeader, uint8_t *RxBuf);


#endif /* CAN_INC_CAN_H */

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! IF NOT HAL OK ERROR HANDELER?


