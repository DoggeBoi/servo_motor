
#include "LSM6DSO.h"
#include <math.h>


uint8_t LSM6DSO_Init(LSM6DSO *imu, SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *csPort, uint16_t csPin) {

	/*   Store parameters in struct   */
	imu->spiHandle 				= 	spiHandle;
	imu->csPort					=	csPort;
	imu->csPin					=	csPin;

	/*   Set base parameters   */
	imu->extFilterCoefficient 	= LSM6DSO_FILTER_COEFFICIENT;

	/*   Initialise previous calculation terms   */
	imu->intAngleX = 0;
	imu->intAngleY = 0;


	/*   Chip select default inactive high   */
	HAL_GPIO_WritePin(imu->csPort, imu->csPin, GPIO_PIN_SET);

	/*	IMU startup   */
	HAL_Delay(20);

	/*   Activate accelerometer   */
	LSM6DSO_WriteRegister(imu, LSM6DSO_CTRL1_XL, 0x60);		// Accelerometer 416Hz, +-2g, no secondary LPF filtering

	/*   Activate gyroscope   */
	LSM6DSO_WriteRegister(imu, LSM6DSO_CTRL2_G, 0x60);		// Gyroscope 416Hz +-250 dps, no secondary LPF filtering


	return 1; //TEMP
}


void LSM6DSO_TransmitReceive(LSM6DSO *imu, uint8_t *TxBuf, uint8_t *RxBuf, uint8_t lenght) {

	/*   Transmit and pulse chip select   */
	HAL_GPIO_WritePin(imu->csPort, imu->csPin, GPIO_PIN_RESET);

	HAL_SPI_TransmitReceive(imu->spiHandle, TxBuf, RxBuf, lenght, LSM6DSO_MAX_DELAY);

	HAL_GPIO_WritePin(imu->csPort, imu->csPin, GPIO_PIN_SET);

}


void LSM6DSO_ReadRegister(LSM6DSO *imu, uint8_t regAddr, uint8_t *data) {

	/*   Initialise SPI buffers   */
	uint8_t TxBuf[2] = {0};
	uint8_t RxBuf[2] = {0};

	/*   Set read bit and register address   */
	TxBuf[0]  = ( regAddr | ( 1 << 7 ) );			// R/W bit 1

	/*   Transmit and receive SPI package   */
	LSM6DSO_TransmitReceive(imu, TxBuf, RxBuf, 2);

	/*   Save read data to data variable   */
	*data = RxBuf[1];

}


void LSM6DSO_WriteRegister(LSM6DSO *imu, uint8_t regAddr, uint8_t data) {

	/*   Initialise SPI buffers   */
	uint8_t TxBuf[2] = {0};
	uint8_t RxBuf[2] = {0};

	/*   Set write bit and register address   */
	TxBuf[0]  = ( regAddr | ( 0 << 7 ) );			// R/W bit 0

	/*   Set write data   */
	TxBuf[1]  = data;

	/*   Transmit and receive SPI package   */
	LSM6DSO_TransmitReceive(imu, TxBuf, RxBuf, 2);

}


void LSM6DSO_ReadAccelerometer(LSM6DSO *imu) {

	/*   Initialise raw data registers   */
	int16_t registerX;
	int16_t registerY;
	int16_t registerZ;

	/*   Conversion factor calculation ( m/s^2 )   */
	float convertionFactor  = ( 9.81f * 2.0f / ( 2 << 15 ) );		// For +-2g

	/*   Read accelerometer data   */
	LSM6DSO_ReadRegister(imu, LSM6DSO_OUTX_L_A, ( (uint8_t *)&registerX ) );
	LSM6DSO_ReadRegister(imu, LSM6DSO_OUTX_H_A, ( (uint8_t *)&registerX + 1 ) );

	LSM6DSO_ReadRegister(imu, LSM6DSO_OUTY_L_A, ( (uint8_t *)&registerY ) );
	LSM6DSO_ReadRegister(imu, LSM6DSO_OUTY_H_A, ( (uint8_t *)&registerY + 1 ) );

	LSM6DSO_ReadRegister(imu, LSM6DSO_OUTY_L_A, ( (uint8_t *)&registerZ ) );
	LSM6DSO_ReadRegister(imu, LSM6DSO_OUTY_H_A, ( (uint8_t *)&registerZ + 1 ) );

	/*   Update values in struct   */
	imu->acclerationX = registerX * convertionFactor;
	imu->acclerationY = registerY * convertionFactor;
	imu->acclerationZ = registerZ * convertionFactor;

}


void LSM6DSO_ReadGyroscope(LSM6DSO *imu) {

	/*   Initialise raw data registers   */
	int16_t registerX;
	int16_t registerY;
	int16_t registerZ;

	/*   Conversion factor calculation ( radians/s )   */
	float convertionFactor  = ( 3.141592f / 180.0f );			// For +-250dps

	/*   Read gyroscope data   */
	LSM6DSO_ReadRegister(imu, LSM6DSO_OUTX_L_G, ( (uint8_t *)&registerX ) );
	LSM6DSO_ReadRegister(imu, LSM6DSO_OUTX_H_G, ( (uint8_t *)&registerX + 1 ) );

	LSM6DSO_ReadRegister(imu, LSM6DSO_OUTY_L_G, ( (uint8_t *)&registerY ) );
	LSM6DSO_ReadRegister(imu, LSM6DSO_OUTY_H_G, ( (uint8_t *)&registerY + 1 ) );

	LSM6DSO_ReadRegister(imu, LSM6DSO_OUTY_L_G, ( (uint8_t *)&registerZ ) );
	LSM6DSO_ReadRegister(imu, LSM6DSO_OUTY_H_G, ( (uint8_t *)&registerZ + 1 ) );

	/*   Update values in struct   */
	imu->angularVelocityX = registerX * convertionFactor;
	imu->angularVelocityY = registerY * convertionFactor;
	imu->angularVelocityZ = registerZ * convertionFactor;

}


void LSM6DSO_ReadSensors(LSM6DSO *imu) {

	/*   Initialise raw data variable   */
	uint8_t registerData;

	/*   Read status register   */
	LSM6DSO_ReadRegister(imu, LSM6DSO_STATUS_REG, &registerData);

	/*   If data ready   */
	if ( registerData & 0x1 ) {

		LSM6DSO_ReadAccelerometer(imu);
		LSM6DSO_ReadGyroscope(imu);

	}

}


/*   Complementary filter   */
void LSM6DSO_EstimateOrientation(LSM6DSO *imu) {

	/*   Conversion factor calculation ( int16_t )   */
	float convertionFactor  = ( 32768.0f / 3.141592f );							// For +- 1 radian

	/*   Calculate internal filter value from external    */
	float alpha			= imu->extFilterCoefficient / 2550.0f;					// Convert Filter-coeffiectnt to alpha
	float time			= imu->extSampleTime 		/ 1000.0f;					// Convert ms to s

	/*   Inertial frame of reference angle estimation   */
	float accelAngleX 	= atanf( imu->acclerationY 	/ imu->acclerationZ );		// Roll calculation
	float accelAngleY 	= asinf( imu->acclerationX 	/ 9.81f );					// Pitch calculation

	/*   Calculate Euler-rates   */
	float gyroAngleX	= imu->angularVelocityX + sinf(imu->intAngleX) * tanf(imu->intAngleY) * imu->angularVelocityY + sinf(imu->intAngleX) * tanf(imu->intAngleY) * imu->angularVelocityZ;
	float gyroAngleY 	= imu->angularVelocityY * cosf(imu->intAngleX) - sinf(imu->intAngleX) * imu->angularVelocityZ;

	/*   Complementary filter calculation   */
	imu->intAngleX		= accelAngleX * alpha + ( 1 - alpha ) * ( imu->intAngleX + time * gyroAngleX );
	imu->intAngleY 		= accelAngleY * alpha + ( 1 - alpha ) * ( imu->intAngleY + time * gyroAngleY );

	/*   Convert radians to int16   */
	imu->extAngleX = imu->intAngleX * convertionFactor;
	imu->extAngleY = imu->intAngleY * convertionFactor;

}
