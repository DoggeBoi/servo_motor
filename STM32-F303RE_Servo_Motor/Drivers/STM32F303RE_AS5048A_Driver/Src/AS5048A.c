
#include "AS5048A.h"

/*   SPI parity calculator   */
uint8_t CalcParityEven(uint16_t data) {

	/*   Initialise return variable   */
	uint8_t parity_bit = 0;

	/*   Calculate parity   */
	while (data){
		parity_bit ^= ( data & 1 );
		data >>= 1;
	}

	return parity_bit;

}

uint8_t CheckParityEven(uint16_t data){

	/*   Initialise count variable   */
	uint8_t count = 0;

	/*   Count number of ones   */
	while ( data ) {
		count += data & 1;
		data >>= 1;
	}

	return ( count & 1 ); 			// Return 0 if even count

}


/*   Initialisation   */
uint8_t AS5048A_Init(AS5048A *encoder, SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *csPort, uint16_t csPin, uint8_t invert) {

	/*   Store parameters in struct   */
	encoder->spiHandle 	= 	spiHandle;
	encoder->csPort		=	csPort;
	encoder->csPin		=	csPin;

	/*   Angle data inversion variable   */
	encoder->inverted 	= 	invert;

	/*   Chip select default inactive high   */
	HAL_GPIO_WritePin(encoder->csPort, encoder->csPin, GPIO_PIN_SET);

	/*	Encoder startup   */
	HAL_Delay(10);

	/*   SPI connection check   */
	return AS5048A_ConnectionCheck(encoder);

}


void AS5048A_ReadRegister(AS5048A *encoder, uint16_t regAddr, uint16_t * data) {

	/*   Initialise SPI buffers   */
	uint16_t TxBuf[] = {AS5048A_NOP};
	uint16_t RxBuf[] = {AS5048A_NOP};

	/*   Set command transmission package   */
	TxBuf[0]  = ( regAddr | ( 0x0001 << 14 ) );					// Sets Command Package address and read bit
	TxBuf[0] |= ( CalcParityEven( TxBuf[0] ) << 15 );			// Sets Command Package parity bit

	/*   Transmit command package   */
	AS5048A_Transmit(encoder, TxBuf);

	/*   Receive read package   */
	AS5048A_Receive(encoder, RxBuf);

	/*   Save package data*/
	*data = RxBuf[0];

}


void AS5048A_WriteRegister(AS5048A *encoder, uint16_t regAddr, uint16_t data) {

	/*   Initialise SPI buffers   */
	uint16_t TxBuf[] = {AS5048A_NOP};
	uint16_t RxBuf[] = {AS5048A_NOP};

	/*   Set read transmission package   */
	TxBuf[0] = ( regAddr | CalcParityEven( regAddr ) << 15 );	// Sets Command Package parity bit

	/*   Transmit command package   */
	AS5048A_Transmit(encoder, TxBuf);

	/*   Set data transmission package   */
	TxBuf[0] = ( data | CalcParityEven( data ) << 15 );			// Sets Command Package parity bit

	/*   Transmit data package   */
	AS5048A_Transmit(encoder, TxBuf);

	/*   Receive new register read package   */
	AS5048A_Receive(encoder, RxBuf);							// Do noting with return

}


void AS5048A_ClearErrorFlags(AS5048A *encoder) {

	/*   Initialise dummy buffer   */
	uint16_t TxBuf[1] = {AS5048A_NOP};

	/*   Read error register   */
	AS5048A_ReadRegister(encoder, AS5048A_CEFR, TxBuf);			// Error register cleared by reading, do nothing with old EF register data.

}


void AS5048A_SetZeroPosition(AS5048A *encoder, uint16_t zeroPos) {

	/*   Calculate high/low register split   */
	uint16_t highRegisterData = ( zeroPos & 0x3FC0 ) >> 6;
	uint16_t lowRegisterData  = ( zeroPos & 0x003F );

	/*   Write position data to registers   */
	AS5048A_WriteRegister(encoder, AS5048A_ZPHR, highRegisterData);
	AS5048A_WriteRegister(encoder, AS5048A_ZPLR, lowRegisterData);

}


void AS5048A_ReadAngle(AS5048A *encoder){

	/*   Initialise raw data variable   */
	uint16_t registerData;

	/*   Read angle register   */
	AS5048A_ReadRegister(encoder, AS5048A_ANGR, &registerData);

	/*   Save only data bits and invert   */
	registerData &= 0x3FFF;
	encoder->angle = ( encoder->inverted == 0 ) ? registerData : ( 16383 - registerData );

}


void AS5048A_Transmit(AS5048A *encoder, uint16_t *TxBuf){

	/*   Transmit and pulse chip select   */
	HAL_GPIO_WritePin(encoder->csPort, encoder->csPin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(encoder->spiHandle, (uint8_t*)TxBuf, 1, AS5048A_MAX_DELAY);

	HAL_GPIO_WritePin(encoder->csPort, encoder->csPin, GPIO_PIN_SET);

}


void AS5048A_Receive(AS5048A *encoder, uint16_t *RxBuf){

	/*   Transmit and pulse chip select   */
	HAL_GPIO_WritePin(encoder->csPort, encoder->csPin, GPIO_PIN_RESET);

	HAL_SPI_Receive(encoder->spiHandle, (uint8_t*)RxBuf, 1, AS5048A_MAX_DELAY);

	HAL_GPIO_WritePin(encoder->csPort, encoder->csPin, GPIO_PIN_SET);


}

uint8_t AS5048A_ConnectionCheck(AS5048A *encoder){

	/*   Initialise status variable   */
	uint8_t status = 0;

	/*   Initialise raw data variable   */
	uint16_t registerData;

	/*   Clear error flag register   */
	AS5048A_ClearErrorFlags(encoder);

	/*   Read AGCR register   */
	AS5048A_ReadRegister(encoder, AS5048A_AGCR, &registerData);

	/*   Check even parity bit   */
	status += CheckParityEven(registerData);

	/*   Check error flag bit   */
	status +=  ( ( registerData >> 14 ) & 1 );			// Get bit 14 value

	/*   Check OCF bit   */
	status += !( ( registerData >> 8 ) & 1 );			// Get bit 8 value, invert

	/*   Check COF bit   */
	status +=  ( ( registerData >> 9 ) & 1 );			// Get bit 9 value, invert

	if ( status != 0 ) {
		return SPI_COM_FAIL;
	}

	return SPI_COM_OK;

}
