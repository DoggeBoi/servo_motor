
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

	/*   Activate callback functions   */
	status += HAL_CAN_ActivateNotification(canbus->canHandle, CAN_IT_RX_FIFO0_MSG_PENDING);
	status += HAL_CAN_ActivateNotification(canbus->canHandle, CAN_IT_RX_FIFO1_MSG_PENDING);

	return status;

}

uint8_t CAN_SetFilter(CANBUS *canbus) {

	/*   Initialise status variable   */
	uint8_t status = 0;

	/*   Initialise and calculate CAN id parameters   */
	uint16_t FIFO0_ID = ( canbus->canDeviceID << 4 ) | (0 << 10);						// Set can ID bits, MSB 0 for high priority messages
    uint16_t FIFO1_ID =	( canbus->canDeviceID << 4 ) | (1 << 10);						// Set can ID bits, MSB 1 for low priority messages

	/*	CAN FIFO Filter configuration declaration	*/
	CAN_FilterTypeDef CAN_Filterconfig_FIFO_0;									// Critical to high priority mailbox
	CAN_FilterTypeDef CAN_Filterconfig_FIFO_1;									// Moderate to low priority mailbox

	/*	CAN FIFO_0 filter specific configuration   */							// Shift 5 since identifier 11 bits, high register 16 bits
	CAN_Filterconfig_FIFO_0.FilterIdHigh 			= 	FIFO0_ID << 5;			// Bit 0 must be 0 (High priority), bit 1 don't care (Both high priority),bit 2-6 must be 00001 (Device id 1), bit 7-10 don't care (Operation)
	CAN_Filterconfig_FIFO_0.FilterIdLow 			= 	0x0000;					// Not used with 11-bit identifier
	CAN_Filterconfig_FIFO_0.FilterMaskIdHigh 		= 	0x05F0 << 5;			// Masked to only check, MSB priority bit and 5 bit identifier
	CAN_Filterconfig_FIFO_0.FilterMaskIdLow			= 	0x0000;					// Not used with 11-bit identifier

	/*	CAN FIFO_0 filter standard configuration   */
	CAN_Filterconfig_FIFO_0.FilterFIFOAssignment 	=	CAN_FILTER_FIFO0;
	CAN_Filterconfig_FIFO_0.FilterBank 				= 	0;						// Selects used filter bank as 0 for FIFO0
	CAN_Filterconfig_FIFO_0.FilterMode 				= 	CAN_FILTERMODE_IDMASK;
	CAN_Filterconfig_FIFO_0.FilterScale 			= 	CAN_FILTERSCALE_32BIT;	// Use 32 bit, only need single identifier mask
	CAN_Filterconfig_FIFO_0.FilterActivation 		= 	CAN_FILTER_ENABLE;
	CAN_Filterconfig_FIFO_0.SlaveStartFilterBank 	= 	0;						// Unimportant, SMT32F3 has only one CAN BUS

	/*	CAN FIFO_1 filter specific configuration   */
	CAN_Filterconfig_FIFO_1.FilterIdHigh 			= 	FIFO1_ID << 5;			// Bit 0 must be 1 (High priority), bit 1 don't care (Both high priority), bit 2-6 must be 00001 (Device id 1), bit 7-10 don't care (Operation)
	CAN_Filterconfig_FIFO_1.FilterIdLow 			= 	0x0000;					// Not used with 11-bit identifier
	CAN_Filterconfig_FIFO_1.FilterMaskIdHigh 		= 	0x05F0 << 5;			// Masked to only check, MSB priority bit and 5 bit identifier
	CAN_Filterconfig_FIFO_1.FilterMaskIdLow 		= 	0x0000;					// Not used with 11-bit identifier

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
void CAN_SendDataFrame(CANBUS *canbus, uint8_t *TxData, uint8_t dataLenght, uint8_t priority, uint8_t operation) {


	/*   Initialise header struct   */
	CAN_TxHeaderTypeDef TxHeader;

	/*   Standard settings   */
	TxHeader.ExtId 					= 	0;					// Set extended id, not used, 0
	TxHeader.IDE 					= 	CAN_ID_STD;			// Use 11/29-bit identifier
	TxHeader.RTR 					= 	CAN_RTR_DATA;		// Send/request data
	TxHeader.TransmitGlobalTime 	=	DISABLE;			// Time-stamp disable.

	/*   Specific setting   */
	TxHeader.DLC 					= dataLenght;						// Set data-lenght bit

	/*   Set CAN transmission header   */
	TxHeader.StdId 				    = ( priority << 9 );     			// Set priority bits
	TxHeader.StdId 				   |= ( canbus->canMasterID << 4 );     // Set recipient id bits
	TxHeader.StdId 				   |= ( operation );     				// Set operation id bits

	/*   Add can frame to transmission mailbox   */
	HAL_CAN_AddTxMessage(canbus->canHandle, &TxHeader, TxData, &canbus->canTxMailbox);

}

void CAN_GetFrame(CANBUS *canbus, uint8_t RxMailbox, CAN_RxHeaderTypeDef *RxHeader, uint8_t *RxBuf) {

	HAL_CAN_GetRxMessage(canbus->canHandle, RxMailbox, RxHeader, RxBuf);
	//Motor_do_thing
	//Check fifo fill level, if 0 reacitvate notification and lower flag

}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {

	HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	//RTOS fifo task unsuspend

}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {

	HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
	//RTOS fifo task unsuspend

}


/*   Send startup confirmation to master device   */
void CAN_SendStartUpFrame(CANBUS *canbus) {
// send remote frame requseting statrup data parameters!!!

}
