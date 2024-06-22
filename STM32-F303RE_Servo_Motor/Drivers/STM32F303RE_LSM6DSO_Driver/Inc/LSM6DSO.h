
#ifndef STM32F303RE_LSM6DSO_DRIVER_INC_LSM6DSO_H
#define STM32F303RE_LSM6DSO_DRIVER_INC_LSM6DSO_H


#include "stm32f3xx_hal.h"

/*	Register defines*/
#define LSM6DSO_INT1_CTRL 	0x0D
#define LSM6DSO_INT2_CTRL 	0x0E
#define LSM6DSO_CTRL1_XL  	0x10
#define LSM6DSO_CTRL2_G	  	0x11
#define LSM6DSO_STATUS_REG	0x1E

#define LSM6DSO_OUTX_L_G    0x22
#define LSM6DSO_OUTX_H_G    0x23
#define LSM6DSO_OUTY_L_G    0x24
#define LSM6DSO_OUTY_H_G    0x25
#define LSM6DSO_OUTZ_L_G    0x26
#define LSM6DSO_OUTZ_H_G    0x27

#define LSM6DSO_OUTX_L_A    0x28
#define LSM6DSO_OUTX_H_A    0x29
#define LSM6DSO_OUTY_L_A    0x2A
#define LSM6DSO_OUTY_H_A    0x2B
#define LSM6DSO_OUTZ_L_A    0x2C
#define LSM6DSO_OUTZ_H_A    0x2D


/*   Max SPI timeout define   */
#define LSM6DSO_MAX_DELAY 3


/*   Base parameters   */
#define LSM6DSO_FILTER_COEFFICIENT 51 	// 0.2

typedef struct {

	/*	SPI	 */
	SPI_HandleTypeDef  *spiHandle;
	GPIO_TypeDef 	   *csPort;
	uint16_t 		   	csPin;

	/*   Accelerometer values   */
	float 				acclerationX;
	float 				acclerationY;
	float 				acclerationZ;

	/*   Gyroscope values   */
	float 				angularVelocityX;
	float 				angularVelocityY;
	float 				angularVelocityZ;

	/*   Complementary filter parameters   */
	uint8_t 			extSampleTime;			// in ms
	uint8_t				extFilterCoefficient;	// 0 - 255, 0 - 0.1

	/*   Complementary filter internal output values   */
	float				intAngleX;
	float				intAngleY;

	/*   Output values   */
	int16_t				extAngleX;
	int16_t				extAngleY;

} LSM6DSO;


/*   Initialisation   */
uint8_t LSM6DSO_Init(LSM6DSO *imu, SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *csPort, uint16_t csPin);

/*   SPI transmit / receive and chip select control function   */
void LSM6DSO_TransmitReceive(LSM6DSO *imu, uint8_t *TxBuf, uint8_t *RxBuf, uint8_t lenght);

/*   Lower level register functions   */
void LSM6DSO_ReadRegister(LSM6DSO *imu, uint8_t regAddr, uint8_t *data);
void LSM6DSO_WriteRegister(LSM6DSO *imu, uint8_t regAddr, uint8_t data);

/*   Read IMU data   */
void LSM6DSO_ReadSensors(LSM6DSO *imu);
void LSM6DSO_ReadAccelerometer(LSM6DSO *imu);
void LSM6DSO_ReadGyroscope(LSM6DSO *imu);

/*   Complementary filter   */
void LSM6DSO_EstimateOrientation(LSM6DSO *imu);



#endif /* STM32F303RE_LSM6DSO_DRIVER_INC_LSM6DSO_H */
