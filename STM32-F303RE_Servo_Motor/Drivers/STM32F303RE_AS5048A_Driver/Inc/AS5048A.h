
#ifndef STM32F303RE_AS5048A_DRIVER_INC_AS5048A_H
#define STM32F303RE_AS5048A_DRIVER_INC_AS5048A_H

#include "stm32f3xx_hal.h"

/*	Register defines*/
#define	AS5048A_NOP				0x0000		//	Dummy Command
#define	AS5048A_CEFR			0x0001		//	Clear error flag, errors cleared by access
//#define	AS5048A_PCR			0x0003		//	Program control, can burn settings, no touchy!
#define	AS5048A_ZPHR			0x0016		//  Zero Position value MSB(s)
#define	AS5048A_ZPLR			0x0017		//	Zero Position value 6 LSB(s)
#define	AS5048A_AGCR			0x3FFD		//	Diagnostics + Automatic Gain Control
#define	AS5048A_MAGR			0x3FFE		//	Magnitude output value of the CORDIC
#define	AS5048A_ANGR			0x3FFF		//	Angle output value including zero position correction

/*   SPI connections check return values   */
#define SPI_COM_OK 				0
#define SPI_COM_FAIL 			1

/*   Max SPI timeout define*/
#define	AS5048A_MAX_DELAY		50

typedef struct {

	/*	SPI	 */
	SPI_HandleTypeDef *spiHandle;
	GPIO_TypeDef 	  *csPort;
	uint16_t 		   csPin;

	/*   Angle data inversion variable   */
	uint8_t inverted;

	/*   raw angle data   */
	uint16_t angle;

} AS5048A;


/*   Initialisation   */
uint8_t AS5048A_Init(AS5048A *encoder, SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *csPort, uint16_t csPin, uint8_t invert);


/*   SPI transmit / receive and chip select control function   */
void AS5048A_Transmit(AS5048A *encoder, uint16_t *TxBuf);
void AS5048A_Receive(AS5048A *encoder, uint16_t *RxBuf);


/*   Lower level register functions   */
void AS5048A_ReadRegister(AS5048A *encoder, uint16_t regAddr, uint16_t *data);
void AS5048A_WriteRegister(AS5048A *encoder, uint16_t regAddr, uint16_t data);


/*   Clear error flags   */
void AS5048A_ClearErrorFlags(AS5048A *encoder);


/*	Angle read function   */
void AS5048A_ReadAngle(AS5048A *encoder);


/*   Set home position   */
void AS5048A_SetZeroPosition(AS5048A *encoder, uint16_t zeroPos);


/*   SPI parity calculator   */
uint8_t CalcParityEven(uint16_t data);


/*   Check parity calculator   */
uint8_t CheckParityEven(uint16_t data);


/*   Check connection   */
uint8_t AS5048A_ConnectionCheck(AS5048A *encoder);


#endif /* STM32F303RE_AS5048A_DRIVER_INC_AS5048A_H */
