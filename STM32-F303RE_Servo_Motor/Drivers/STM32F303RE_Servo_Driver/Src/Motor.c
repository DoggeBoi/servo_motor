
#include "Motor.h"
#include "AS5048A.h"
#include "ADC.h"
#include "HBRIDGE.h"
#include <math.h>
#include <stdlib.h>



/*   Servo value initialisation   */
void servoInit(SERVO_CONTROL *servo) {

	/*   Initialise ID and version   */
	servo->id 								= ID;
	servo->firmwareVer 						= FIRMWARE_VERSION;

	/*   Disable torque   */
	servo->torqueEnable 					= SERVO_TORQUE_DISABLE;

	/*   Initialise motion variables   */
	servo->motionDirection				 	= SERVO_DIRECTION_CCW;
	servo->motionThreshold 					= SERVO_MOTION_THRESHOLD;
	servo->profile.followingThreshold 		= SERVO_FOLLOWING_THRESHOLD;

	/*   Initialise hardware error status   */
	servo->hardwareError.rangeVoltage		= 0;
	servo->hardwareError.overTempMCU		= 0;
	servo->hardwareError.overTempMotor		= 0;
	servo->hardwareError.encoderMalfunc		= 0;
	servo->hardwareError.overCurrent		= 0;
	servo->hardwareError.errorSPI			= 0;

	/*   Initialise  operating limits   */
	servo->maxMotorTemp 					= SERVO_MAX_MOTOR_TEMP;
	servo->maxIntTemp 						= SERVO_MAX_INTERNAL_TEMP;
	servo->maxVoltage 						= SERVO_MAX_VOLTAGE;
	servo->minVoltage 						= SERVO_MIN_VOLTAGE;
//	servo->maxCurrent 						= SERVO_MAX_CURRENT;
//	servo->maxTorque 						= SERVO_MAX_TORQUE;

	/*   Initialise motion profile limits   */
	servo->maxPosition 						= SERVO_MAX_POSITION;
	servo->minPosition						= SERVO_MIN_POSITION;

	servo->profile.maxAcceleration 			= SERVO_MAX_ACCELERATION;
	servo->profile.maxVelocity 	   			= SERVO_MAX_VELOCITY;

	/*   Initialise previous values for PID   */
	servo->prevVelocity 					= 0;
	servo->PID.prevError 					= 0;

	/*   Initialise motion profile state   */
	servo->profile.trajectoryStatus 		= SERVO_PROFILE_COMPLETE;
	servo->profile.trajectoryGoalPosition 	= 0;
	servo->profile.trajectoryVelocity 		= 0;

	/*   Static friction compensation value   */
	servo->friction 						= SERVO_STATIC_FRICTION;
}

/*   Encoder initialisation   */
void encoderInit(SERVO_CONTROL *servo, SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *csPort, uint16_t csPin) {

	AS5048A_Init(&servo->encoder, spiHandle, csPort, csPin, servo->motionDirection);

}


/*   Motor H-Bridge initialisation   */
void motorInit(SERVO_CONTROL *servo, TIM_HandleTypeDef *timHandle, uint8_t timChannel, GPIO_TypeDef *hswAPort, uint16_t hswAPin, GPIO_TypeDef *hswBPort, uint16_t hswBPin) {

	pwmInit(&servo->motor, timHandle, timChannel, hswAPort, hswAPin, hswBPort, hswBPin, servo->friction);

}


/*   ADC sensors initialisations   */
void sensorsInit(SERVO_CONTROL *servo, ADC_HandleTypeDef *adcHandle0, ADC_HandleTypeDef *adcHandle1, ADC_HandleTypeDef *adcHandle2, ADC_HandleTypeDef *adcHandle3) {

	adcSensorInit(&servo->intTemp, 			rawToIntTemp, 	adcHandle0);

	adcSensorInit(&servo->motorTemp, 		rawToMotorTemp, adcHandle1);

	adcSensorInit(&servo->batteryVoltage, 	rawToVoltage, 	adcHandle2);

	adcSensorInit(&servo->motorCurrent,	 	rawToCurrent, 	adcHandle3);

}


/*   Sensor Data check    */
void sensorsCheck(SERVO_CONTROL *servo) {

	/*   Initialise status variable   */
	uint8_t status = 0;

	/*   Check each sensor value   */
	adcSensorRangeCheck(&servo->intTemp, &servo->hardwareError.overTempMCU, SERVO_MAX_INTERNAL_TEMP, 0);

	adcSensorRangeCheck(&servo->motorTemp, &servo->hardwareError.overTempMCU, SERVO_MAX_MOTOR_TEMP, 0);

	adcSensorRangeCheck(&servo->batteryVoltage, &servo->hardwareError.overTempMCU, SERVO_MAX_VOLTAGE, SERVO_MIN_VOLTAGE);

	adcSensorRangeCheck(&servo->motorCurrent, &servo->hardwareError.overTempMCU, SERVO_MAX_CURRENT, 0);

	/*   If range error has occurred call error handler   */
	if ( status != 0 ) errorHandeler(servo);

}


/*   PID value initialisation   */
void pidInit(SERVO_CONTROL *servo, float Kp, float Kd, float Ki, float lpfConstant) {

	/*   Initialise PID sample period value   */
	servo->PID.samplePeriod   	= 	SERVO_PID_INTERVAL;

	/*   Initialise PID output limits   */
	servo->PID.outputMax      	=  	SERVO_PID_MAX;
	servo->PID.outputMin      	=  	SERVO_PID_MIN;

	/*   Initialise PID gains   */
	servo->PID.Kp 			  	= 	Kp;
	servo->PID.Kd 			  	= 	Kd;
	servo->PID.Ki 			  	= 	Ki;

	/*   Initialise low pass filter parameter   */
	servo->PID.lpfConstant 	  	= 	lpfConstant;

	/*   Initialise previous values    */
	servo->PID.prevInput 	  	= 	0;

	/*   Initialise PID terms    */
	servo->PID.differentaitor 	= 	0;
	servo->PID.integrator	  	= 	0;

}


/*   PID calculate and update   */
void pidUpdate(SERVO_CONTROL *servo) {

	/*   Update PID input   */
	servo->PID.Input 			= 	servo->encoder.angle;

	/*   Calculate error   */
	servo->PID.Error      		= 	servo->PID.setPoint - servo->PID.Input;

	/*   Calculate PID proportional term   */
	servo->PID.proportinal 		= 	( servo->PID.Kp / 1000.0f ) * servo->PID.Error;


	/*   Calculate PID integrator term   */				// Disable change if torque is off.
	servo->PID.integrator 		= 	servo->PID.integrator + servo->torqueEnable * 0.5f * ( servo->PID.Ki / 1000.0f ) * servo->PID.samplePeriod * 0.001f *
								  ( servo->PID.Error + servo->PID.prevError );

	/*   Calculate PID dynamic anti whind-up limits   */
	if ( servo->PID.outputMax > servo->PID.proportinal )  				servo->PID.integratorLimitMax 	= servo->PID.outputMax - servo->PID.proportinal;
	else 																servo->PID.integratorLimitMax 	= 0.0f;

	if ( servo->PID.outputMin < servo->PID.proportinal )  				servo->PID.integratorLimitMin 	= servo->PID.outputMax - servo->PID.proportinal;
	else 																servo->PID.integratorLimitMin 	= 0.0f;

    /*   Apply calculated clamping limits   */
	if ( servo->PID.integrator > servo->PID.integratorLimitMax ) 		servo->PID.integrator 			= servo->PID.integratorLimitMax;
	else if ( servo->PID.integrator < servo->PID.integratorLimitMin ) 	servo->PID.integrator 			= servo->PID.integratorLimitMin;

	/*   Calculate velocity   */
	servo->velocity 	  		= ( ( 2.0f * ( servo->PID.Input - servo->PID.prevInput ) ) +
			  	  	  	  	  	    ( 2.0f * ( servo->PID.lpfConstant / 1000.0f )  - servo->PID.samplePeriod * 0.001f ) * servo->velocity ) /
			  	  	  	  	  	    ( 2.0f * ( servo->PID.lpfConstant / 1000.0f ) + servo->PID.samplePeriod * 0.001f );

	/*   Calculate acceleration   */
	servo->acceleration 	  	= ( ( 2.0f * ( servo->velocity - servo->prevVelocity ) ) +
  	  	  	    					( 2.0f * ( servo->PID.lpfConstant / 1000.0f )  - servo->PID.samplePeriod * 0.001f ) * servo->acceleration ) /
  	  	  	    					( 2.0f * ( servo->PID.lpfConstant / 1000.0f ) + servo->PID.samplePeriod * 0.001f );

	/*   Calculate PID differentaitor term   */					// Differentaitor is wrong way for some reason.... Negating w -100000.0f
	servo->PID.differentaitor 	= 	( ( 2.0f * ( servo->PID.Kd / -100000.0f )  * ( servo->PID.Input - servo->PID.prevInput ) ) +
									  ( 2.0f * ( servo->PID.lpfConstant / 1000.0f )  - servo->PID.samplePeriod * 0.001f ) * servo->PID.differentaitor ) /
									  ( 2.0f * ( servo->PID.lpfConstant / 1000.0f ) + servo->PID.samplePeriod * 0.001f );

	/*   Calculate PID output   */
	servo->PID.output = (int16_t) ( servo->PID.proportinal + servo->PID.integrator + servo->PID.differentaitor);

	/*   Apply clamping limiter to PID output   */
	if 		(servo->PID.output < servo->PID.outputMin) 	servo->PID.outputLimited = servo->PID.outputMin;

	else if (servo->PID.output > servo->PID.outputMax) 	servo->PID.outputLimited = servo->PID.outputMax;

	else 												servo->PID.outputLimited = servo->PID.output;

	/*   Update previous variables for use in next PID cycle*/
    servo->PID.prevInput 				= servo->PID.Input;
    servo->prevVelocity					= servo->velocity;
    servo->PID.prevError				= servo->PID.Error;

    /*   Update inMotion variable   */
    servo->inMotion = ( abs( servo->velocity ) >= servo->motionThreshold );			// 1 if motion is above threshold

}


/*   Update motor duty cycle value   */
void motorUpdatePWM(SERVO_CONTROL *servo) {

	/*   PWM decomposition into magnitude and direction   */
	motorPwmDecomp(&servo->motor, servo->PID.outputLimited, servo->torqueEnable, servo->motionDirection);

	/*   Update PWM and H-brdige switches   */
	motorUpdate(&servo->motor);

}


/*   Encoder sensor update   */
void encoderUpdate(SERVO_CONTROL *servo) {

	AS5048A_ReadAngle(&servo->encoder);

}


/*   ADC sensors read   */
void sensorsUpdate(SERVO_CONTROL *servo) {

	adcSensorUpdate(&servo->intTemp);
	adcSensorUpdate(&servo->motorTemp);
	adcSensorUpdate(&servo->batteryVoltage);
	adcSensorUpdate(&servo->motorCurrent);

}


/*   Motion profile calculation    */
void motionPorfile(SERVO_CONTROL *servo, uint8_t timeStep) {

	/*   Motion profile calculation phase duration calculation   */
	if ( ( servo->goalPosition != servo->profile.trajectoryGoalPosition ) || (servo->profile.trajectorySplit == SERVO_PROFILE_SPLIT_2 ) ) {

		/*   Reset split status if goal has changed or if in SERVO_PROFILE_SPLIT_2 state   */
		servo->profile.trajectorySplit = ( servo->goalPosition == servo->profile.trajectoryGoalPosition ) * (servo->profile.trajectorySplit != SERVO_PROFILE_SPLIT_2 );

		/*   Motion profile parameter calculation   */
		servo->profile.trajectoryStartPosition 				= 	servo->profile.trajectorySetPoint;
		servo->profile.trajectoryGoalPosition				=   servo->goalPosition;
		servo->profile.trajectoryDistance 					= 	servo->profile.trajectoryGoalPosition - servo->profile.trajectoryStartPosition;
		servo->profile.trajectoryDirection 					= ( servo->profile.trajectoryDistance >= 0 ) ? 1 : -1;
		servo->profile.trajectoryStartVelocity				=   servo->profile.trajectoryVelocity;

		/*   Trajectory time reset   */
		servo->profile.trajectoryTime						= 0;

		/*   Motion profile acceleration phase parameter calculation   */
		servo->profile.trajectoryAccelerationTime 			= (float) abs( servo->profile.trajectoryDirection * servo->profile.maxVelocity - servo->profile.trajectoryStartVelocity )  / (float) servo->profile.maxAcceleration;
		servo->profile.trajectoryAccelerationDistance 		= ( servo->profile.trajectoryDirection * servo->profile.trajectoryStartVelocity * servo->profile.trajectoryAccelerationTime ) + 0.5f * servo->profile.maxAcceleration * servo->profile.trajectoryAccelerationTime * servo->profile.trajectoryAccelerationTime;

		/*   Motion profile deceleration phase parameter calculation   */
		servo->profile.trajectoryDecelerationTime			= (float) servo->profile.maxVelocity  / (float) servo->profile.maxAcceleration;
		servo->profile.trajectoryDecelerationDistance		= 0.5f * servo->profile.maxAcceleration * servo->profile.trajectoryDecelerationTime * servo->profile.trajectoryDecelerationTime;

		/*   Motion profile constant-velocity phase parameter calculation   */
		servo->profile.trajectoryVelocityDistance 			= abs(servo->profile.trajectoryDistance ) - ( servo->profile.trajectoryAccelerationDistance + servo->profile.trajectoryDecelerationDistance );
		servo->profile.trajectoryVelocityTime 				= servo->profile.trajectoryVelocityDistance / servo->profile.maxVelocity;

		/*   If distance too short to reach max velocity and start velocity is 0   */
		if ( servo->profile.trajectoryVelocityDistance < 0 && servo->profile.trajectoryStartVelocity == 0 ) {		// check if calculated veloity is actually zero by the end and there sint some roudning error like 0.0000001

			servo->profile.trajectoryAccelerationTime 		= sqrt( (float) abs( servo->profile.trajectoryDistance ) / (float) servo->profile.maxAcceleration );
			servo->profile.trajectoryAccelerationDistance 	= 0.5f * servo->profile.maxAcceleration * servo->profile.trajectoryAccelerationTime * servo->profile.trajectoryAccelerationTime;

			servo->profile.trajectoryDecelerationTime		= servo->profile.trajectoryAccelerationTime;

			servo->profile.trajectoryVelocityTime			= 0;
		    servo->profile.trajectoryVelocityDistance		= 0;

		}

		/*   If distance too short to reach max velocity and start velocity is not 0   */
		if ( servo->profile.trajectoryVelocityDistance < 0 && servo->profile.trajectoryStartVelocity != 0 ) {

			servo->profile.trajectoryAccelerationTime		= 0;
			servo->profile.trajectoryAccelerationDistance 	= 0;

			servo->profile.trajectoryVelocityTime			= 0;
			servo->profile.trajectoryVelocityDistance		= 0;

			/*   Decelerate to zero   */
			servo->profile.trajectoryDecelerationTime		= (float) abs(servo->profile.trajectoryStartVelocity) / (float) servo->profile.maxAcceleration;

			/*   Set direction to that of the deceleration of SERVO_PROFILE_SPLIT_1 and not full move   */
			servo->profile.trajectoryDirection = ( servo->profile.trajectoryStartVelocity >= 0 ) ? 1 : -1;

			/*   Update profile split status to deceleration phase   */
			servo->profile.trajectorySplit = SERVO_PROFILE_SPLIT_1;

		}

		/*   Motion profile status update   */
		servo->profile.trajectoryStatus 					= SERVO_PROFILE_ONGOING;

	}


	/*   Convert time reference to seconds   */
	float time = servo->profile.trajectoryTime * 0.001f;


	/*   Acceleration phase   */
	if (time <= servo->profile.trajectoryAccelerationTime ){

		servo->profile.trajectorySetPoint =
		servo->profile.trajectoryStartPosition
	 +  servo->profile.trajectoryDirection
	 *( servo->profile.trajectoryStartVelocity * time
	 *  servo->profile.trajectoryDirection + 0.5f
	 *  servo->profile.maxAcceleration * time * time );

		servo->profile.trajectoryVelocity =
		servo->profile.trajectoryDirection
	 *( servo->profile.trajectoryStartVelocity
	 *  servo->profile.trajectoryDirection
	 +  servo->profile.maxAcceleration * time );

	}

	/*   Constant velocity phase   */
	else if (time <= ( servo->profile.trajectoryAccelerationTime + servo->profile.trajectoryVelocityTime ) ) {

		float t = time - servo->profile.trajectoryAccelerationTime;

		servo->profile.trajectorySetPoint =
		servo->profile.trajectoryStartPosition
	+   servo->profile.trajectoryDirection
	* ( servo->profile.trajectoryAccelerationDistance
	+   servo->profile.maxVelocity * t);

		servo->profile.trajectoryVelocity =
	    servo->profile.trajectoryDirection
	 *  servo->profile.maxVelocity;

	}

	/*   Deceleration phase   */
	else if (time <= ( servo->profile.trajectoryAccelerationTime + servo->profile.trajectoryVelocityTime + servo->profile.trajectoryDecelerationTime ) ) {

		float t = time - servo->profile.trajectoryAccelerationTime - servo->profile.trajectoryVelocityTime;

		  servo->profile.trajectorySetPoint =
		  servo->profile.trajectoryStartPosition
	+     servo->profile.trajectoryDirection
	*   ( servo->profile.trajectoryAccelerationDistance
	+     servo->profile.trajectoryVelocityDistance
	+ ( ( servo->profile.maxAcceleration
	*     servo->profile.trajectoryAccelerationTime
	+     servo->profile.trajectoryStartVelocity
	*     servo->profile.trajectoryDirection ) * t )
	-   ( servo->profile.maxAcceleration * 0.5f * t * t ) );

		  servo->profile.trajectoryVelocity =
	      servo->profile.trajectoryDirection
	* ( ( servo->profile.maxAcceleration
    * 	  servo->profile.trajectoryAccelerationTime
	+     servo->profile.trajectoryStartVelocity * servo->profile.trajectoryDirection)
	-     servo->profile.maxAcceleration * t );

	}

	/*   Hold phase   */
	else {

		/*   Update status and velocity when complete   */
		servo->profile.trajectoryVelocity 				= 0;
		servo->profile.trajectoryStatus 				= SERVO_PROFILE_COMPLETE;

		/*   Update trajectory split status to SERVO_PROFILE_SPLIT_2   */
		if( servo->profile.trajectorySplit == SERVO_PROFILE_SPLIT_1 ) servo->profile.trajectorySplit = SERVO_PROFILE_SPLIT_2;

	}


	/*   Update trajectoryVelocity and setPoint within limits   */
	if ( servo->profile.trajectorySetPoint > servo->minPosition && servo->profile.trajectorySetPoint < servo->maxPosition ) { 	      // Within range

		servo->PID.setPoint = servo->profile.trajectorySetPoint;

	}

	else if ( servo->profile.trajectorySetPoint < servo->minPosition ) {

		servo->PID.setPoint = servo->minPosition;

	}

	else {

		servo->PID.setPoint = servo->maxPosition;

	}


	/*   Increment trajectory time   */
	servo->profile.trajectoryTime += 4 * !!servo->profile.trajectoryStatus;				// Increment by time step (ms) if trajectory is not yet complete.

	/*   Update trajectoryFollowing variable   */
	servo->profile.trajectoryFollowing = ( abs( servo->PID.setPoint - servo->encoder.angle ) <= servo->profile.followingThreshold );

}








