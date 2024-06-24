
#ifndef STM32F303RE_H_BRIDGE_DRIVER_INC_HBRIDGE_H
#define STM32F303RE_H_BRIDGE_DRIVER_INC_HBRIDGE_H

#include "stm32f3xx_hal.h"

typedef struct {

	/*   Timer handle and channel   */
	TIM_HandleTypeDef  *timHandle;
	uint8_t				timChannel;

	/*   H-brdige switch a   */
	GPIO_TypeDef 	   *hswAPort;
	uint16_t 		   	hswAPin;

	/*   H-brdige switch B   */
	GPIO_TypeDef 	   *hswBPort;
	uint16_t 		   	hswBPin;

	/*   Drive parameters   */
	uint8_t 			direction;							// 0-10243
	uint16_t 			dutyCycle;							// 0 = positive

	/*   Friction compensation value   */
	uint8_t 			frictionCompensation;				// Adds value to PWM to avoid startup fiction

} MOTOR;

/*   Motor initialisation   */
uint8_t pwmInit(MOTOR *motor, TIM_HandleTypeDef *timHandle, uint8_t timChannel, GPIO_TypeDef *hswAPort, uint16_t hswAPin, GPIO_TypeDef *hswBPort, uint16_t hswBPin, uint8_t frictionCompensation);

/*   PWM decomposition into magnitude and direction   */
void motorPwmDecomp(MOTOR *motor, int16_t input, uint8_t enable, uint8_t direction);

/*   Update PWM and H-brdige switches   */
void motorUpdate(MOTOR *motor);


#endif /* STM32F303RE_H_BRIDGE_DRIVER_INC_HBRIDGE_H */
