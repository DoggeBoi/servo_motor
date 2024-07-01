
#include "CAN.h"

uint8_t CAN_Init(CANBUS *canbus, CAN_HandleTypeDef *canHandle, uint8_t canDeviceID) {

	/*   Initialise status variable   */
	uint8_t status = 0;

	/*   Initialise CAN parameters   */
	canbus->canHandle 	= canHandle;
	canbus->canDeviceID = canDeviceID;
	canbus->canMasterID = CAN_MASTER_ID;

	/*   Initialise CAN filter parameters   */
	status += CAN_SetFilter(canbus);

	/*   Initialise CAN   */
	status += HAL_CAN_Start(canbus->canHandle);

	return status;

}

/*   Activate CAN callbacks   */
void CAN_ActivateCallback(CANBUS *canbus) {

	/*   Activate callback functions   */
	HAL_CAN_ActivateNotification(canbus->canHandle, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(canbus->canHandle, CAN_IT_RX_FIFO1_MSG_PENDING);

}


uint8_t CAN_SetFilter(CANBUS *canbus) {

	/*   Initialise status variable   */
	uint8_t status 			= 0;

	/*   Initialise and calculate CAN id parameters   */
	uint16_t FIFO0_ID_HIGH 	= ( canbus->canDeviceID >> 5 )  | (0 << 14);		// Set can ID bits, MSB 0 for high priority messages
	uint16_t FIFO0_ID_LOW	= ( canbus->canDeviceID << 11 ) | (1 << 2);			// IDE bit must be 1


    uint16_t FIFO1_ID_HIGH 	= ( canbus->canDeviceID << 5 )  | (1 << 14);		// Set can ID bits, MSB 1 for low priority messages
    uint16_t FIFO1_ID_LOW	= ( canbus->canDeviceID << 11 ) | (1 << 2);			// IDE bit must be 1

	/*	CAN FIFO Filter configuration declaration	*/
	CAN_FilterTypeDef CAN_Filterconfig_FIFO_0;									// Critical to high priority mailbox
	CAN_FilterTypeDef CAN_Filterconfig_FIFO_1;									// Moderate to low priority mailbox

	/*	CAN FIFO_0 filter specific configuration   */
	CAN_Filterconfig_FIFO_0.FilterIdHigh 			= 	FIFO0_ID_HIGH;			// Bit 0 must be 0 (High priority), bit 1 don't care (Both high priority), bit 2-9 don't care (sender id), bit 10-17 must be id, bit 18-29 don't care ( misc )
	CAN_Filterconfig_FIFO_0.FilterIdLow 			= 	FIFO0_ID_LOW;
	CAN_Filterconfig_FIFO_0.FilterMaskIdHigh 		= 	0b0100000000000111;
	CAN_Filterconfig_FIFO_0.FilterMaskIdLow			= 	0b1111100000000100;		// IDE bit must be correct

	/*	CAN FIFO_0 filter standard configuration   */
	CAN_Filterconfig_FIFO_0.FilterFIFOAssignment 	=	CAN_FILTER_FIFO0;
	CAN_Filterconfig_FIFO_0.FilterBank 				= 	0;						// Selects used filter bank as 0 for FIFO0
	CAN_Filterconfig_FIFO_0.FilterMode 				= 	CAN_FILTERMODE_IDMASK;
	CAN_Filterconfig_FIFO_0.FilterScale 			= 	CAN_FILTERSCALE_32BIT;	// Use 32 bit, only need single identifier mask
	CAN_Filterconfig_FIFO_0.FilterActivation 		= 	CAN_FILTER_ENABLE;
	CAN_Filterconfig_FIFO_0.SlaveStartFilterBank 	= 	0;						// Unimportant, SMT32F3 has only one CAN BUS

	/*	CAN FIFO_1 filter specific configuration   */
	CAN_Filterconfig_FIFO_1.FilterIdHigh 			= 	FIFO1_ID_HIGH;			// Bit 0 must be 1 (Low priority), bit 1 don't care (Both high priority), bit 2-9 don't care (sender id), bit 10-17 must be id, bit 18-29 don't care ( misc )
	CAN_Filterconfig_FIFO_1.FilterIdLow 			= 	FIFO1_ID_LOW;
	CAN_Filterconfig_FIFO_1.FilterMaskIdHigh 		= 	0b0100000000000111;
	CAN_Filterconfig_FIFO_1.FilterMaskIdLow 		= 	0b1111100000000100;		// IDE bit must be correct

	/*	CAN FIFO_1 filter standard configuration   */
	CAN_Filterconfig_FIFO_1.FilterFIFOAssignment 	= 	CAN_FILTER_FIFO1;
	CAN_Filterconfig_FIFO_1.FilterBank 				= 	1;						// Selects used filter bank as 1 for FIFO1
	CAN_Filterconfig_FIFO_1.FilterMode 				= 	CAN_FILTERMODE_IDMASK;
	CAN_Filterconfig_FIFO_1.FilterScale 			= 	CAN_FILTERSCALE_32BIT;	// Use 32 bit, only need single identifier mask
	CAN_Filterconfig_FIFO_1.FilterActivation 		= 	CAN_FILTER_ENABLE;
	CAN_Filterconfig_FIFO_1.SlaveStartFilterBank 	= 	0;						// Unimportant, SMT32F3 has only one CAN BUS

	/*   Activate CAN filters   */
	status += HAL_CAN_ConfigFilter(canbus->canHandle, &CAN_Filterconfig_FIFO_0);
	status += HAL_CAN_ConfigFilter(canbus->canHandle, &CAN_Filterconfig_FIFO_1);

	return status;

}

/*   Add data frame to Tx Mailbox   */
void CAN_SendDataFrame(CANBUS *canbus, uint8_t *TxData, uint8_t dataLenght, uint8_t priority, uint8_t operationId) {


	/*   Initialise header struct   */
	CAN_TxHeaderTypeDef TxHeader;

	/*   Standard settings   */
	TxHeader.IDE 					= 	CAN_ID_EXT;					// Use 29-bit identifier
	TxHeader.RTR 					= 	CAN_RTR_DATA;				// Send/request data
	TxHeader.TransmitGlobalTime 	=	DISABLE;					// Time-stamp disable.

	/*   Specific setting   */
	TxHeader.DLC 					= 	dataLenght;					// Set data-lenght bit

	/*   Set CAN id header   */
	TxHeader.StdId 				    = priority << 8 ;     			// Set priority bits
	TxHeader.StdId 				   |= canbus->canDeviceID;     		// Set sender id bits

	TxHeader.ExtId 					= canbus->canMasterID << 8;
	TxHeader.ExtId 				   |= operationId;

	/*   Add can frame to transmission mailbox   */
	HAL_CAN_AddTxMessage(canbus->canHandle, &TxHeader, TxData, &canbus->canTxMailbox);

}

/*   Unpack frame from Rx mailbox*/
void CAN_GetFrame(CANBUS *canbus, uint32_t RxFifo, uint8_t *RxData, uint8_t *senderId, uint8_t *operationId, uint8_t *priority) {

	/*   Initialise header struct and data buffer   */
	CAN_RxHeaderTypeDef 	RxHeader;

	/*   Get frame*/
	HAL_CAN_GetRxMessage(canbus->canHandle, RxFifo, &RxHeader, RxData);

	/*   Extract specific data from extended identifier   */
	*senderId 		= RxHeader.StdId & 0xFF;

	*operationId	= RxHeader.ExtId & 0xFF;

	*priority		= RxHeader.ExtId >> 8;

}

