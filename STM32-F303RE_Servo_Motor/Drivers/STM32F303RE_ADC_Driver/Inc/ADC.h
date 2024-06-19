
#ifndef STM32F303RE_ADC_DRIVER_INC_ADC_H
#define STM32F303RE_ADC_DRIVER_INC_ADC_H

#include "stm32f3xx_hal.h"

/*	Maximum ADC polling conversion timeout   */
#define ADC_MAX_DELAY 1							// ms

/*   Sensor specific conversion function   */
typedef int16_t (*ADC_Conversion_Func)(uint16_t);

typedef struct {

	/*   ADC handle   */
	ADC_HandleTypeDef 	*adcHandle;

	/*   Raw 12-bit ADC data   */
	uint16_t			rawData;

	/*   Converted data sensor specific unit   */
	int16_t				converData;

	/*   Sensor specific conversion function   */
	ADC_Conversion_Func conversionFunction;

} ADC;

/*   Sensor Initialisation   */
void adcSensorInit(ADC *sensor, ADC_Conversion_Func conversionFunction, ADC_HandleTypeDef *adcHandle);

/*   Sensor ADC polling update   */
void adcSensorUpdate(ADC *sensor);

/*   Sensor range check   */
uint8_t adcSensorRangeCheck(ADC *sensor, uint8_t *flag, uint16_t maxValue, uint16_t minValue);

/*   Sensor specific conversion functions   */
int16_t rawToIntTemp(uint16_t rawData);

int16_t rawToMotorTemp(uint16_t rawData);

int16_t rawToVoltage(uint16_t rawData);

int16_t rawToCurrent(uint16_t rawData);

#endif /* STM32F303RE_ADC_DRIVER_INC_ADC_H */
