
#include "HBRIDGE.h"
#include <stdlib.h>
#include <stdio.h>

/*   Motor initialisation   */
void pwmInit(MOTOR *motor, TIM_HandleTypeDef *timHandle, uint8_t timChannel, GPIO_TypeDef *hswAPort, uint16_t hswAPin, GPIO_TypeDef *hswBPort, uint16_t hswBPin, uint8_t frictionCompensation){

	/*   Timer handle   */
	motor->timHandle 		= timHandle;

	/*   H-brdige switch A   */
	motor->hswAPort 		= hswAPort;
	motor->hswAPin 			= hswAPin;

	/*   H-brdige switch B   */
	motor->hswBPort 		= hswBPort;
	motor->hswBPin 			= hswBPin;

	/*   Drive parameters   */
	motor->direction 		= 0;							// 0-10243
	motor->dutyCycle 		= 0;							// 0 = positive

	/*   Friction compensation value   */
	motor->frictionCompensation = frictionCompensation;


	/*   Start PWM timer count   */
	HAL_TIM_PWM_Start(motor->timHandle, motor->timChannel);

}


/*   Update PWM and H-brdige switches   */
void motorUpdate(MOTOR *motor) {

	/*   Set H-Bridge switches   */
	if ( motor->direction == 0 ) {
		HAL_GPIO_WritePin(motor->hswAPort, motor->hswAPin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(motor->hswBPort, motor->hswBPin, GPIO_PIN_SET);
	}
	else {
		HAL_GPIO_WritePin(motor->hswAPort, motor->hswAPin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(motor->hswBPort, motor->hswBPin, GPIO_PIN_RESET);
	}

	if ( motor->dutyCycle == 0 ) {
			HAL_GPIO_WritePin(motor->hswAPort, motor->hswAPin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(motor->hswBPort, motor->hswBPin, GPIO_PIN_RESET);
		}

	/*   Set PWM compare value   */
	__HAL_TIM_SET_COMPARE(motor->timHandle, motor->timChannel, motor->dutyCycle);

}

/*   PWM decomposition into magnitude and direction   */
void motorPwmDecomp(MOTOR *motor, int16_t input, uint8_t enable, uint8_t direction) {

	int16_t power = ( input * enable * ( ( direction == 0 ) ? 1 : -1 ) );
	int16_t duty  = abs(power) + motor->frictionCompensation;

	/*   Clamp limit duty cycle   */
	if ( duty > 1023 ) duty = 1023;

	/*   Update duty cycle   */
	motor->dutyCycle = duty;

	/*   Update direction   */
	if( power >= 0 )   motor->direction = 0;
	else		  	   motor->direction = 1;

}
