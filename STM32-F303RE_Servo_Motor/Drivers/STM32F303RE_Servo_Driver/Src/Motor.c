
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
	servo->motionDirection				 	= SERVO_DIRECTION_CW;
	servo->motionThreshold 					= SERVO_MOTION_THRESHOLD;
	servo->profile.followingThreshold 		= SERVO_FOLLOWING_THRESHOLD;

	/*   Initialise hardware error status   */
	servo->hardwareError.rangeVoltage		= SERVO_OK;
	servo->hardwareError.overTempMCU		= SERVO_OK;
	servo->hardwareError.overTempMotor		= SERVO_OK;
	servo->hardwareError.overCurrent		= SERVO_OK;

	servo->hardwareError.errorInit			= SERVO_OK;
	servo->hardwareError.errorEncoder		= SERVO_OK;
	servo->hardwareError.errorPWM			= SERVO_OK;
	servo->hardwareError.errorADC			= SERVO_OK;
	servo->hardwareError.errorIMU			= SERVO_OK;

	/*   Initialise  operating limits   */
	servo->maxMotorTemp 					= SERVO_MAX_MOTOR_TEMP;
	servo->maxIntTemp 						= SERVO_MAX_INTERNAL_TEMP;
	servo->maxVoltage 						= SERVO_MAX_VOLTAGE;
	servo->minVoltage 						= SERVO_MIN_VOLTAGE;
	servo->maxCurrent 						= SERVO_MAX_CURRENT;

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

	/*   Operation function array   */
	servo->OperationFunctions[0] 	= (void*)writeSingle;
	servo->OperationFunctions[1] 	= (void*)readSingle;
	servo->OperationFunctions[2] 	= (void*)torqueOnCommand;
	servo->OperationFunctions[3] 	= (void*)torqueOffCommand;
	servo->OperationFunctions[4] 	= (void*)startupCommand;
	servo->OperationFunctions[5] 	= (void*)shutdownCommand;
	servo->OperationFunctions[6] 	= (void*)writeConfig1;
	servo->OperationFunctions[7] 	= (void*)writeConfig2;
	servo->OperationFunctions[8] 	= (void*)writeConfig3;
	servo->OperationFunctions[9] 	= (void*)writeConfig4;
	servo->OperationFunctions[10] 	= (void*)writeConfig5;
	servo->OperationFunctions[11] 	= (void*)readconfig1;
	servo->OperationFunctions[12] 	= (void*)readconfig2;
	servo->OperationFunctions[13] 	= (void*)readconfig3;
	servo->OperationFunctions[14] 	= (void*)readconfig4;
	servo->OperationFunctions[15] 	= (void*)readconfig5;


	/*   Variable addressable list   */
	void *variablesCopyFrom[47] = {
			&servo->PID.Kp,
			&servo->PID.Ki,
			&servo->PID.Kd,
			&servo->PID.setPoint,
			&servo->PID.Error,
			&servo->PID.lpfConstant,
			&servo->PID.extIntegrator,
			&servo->PID.extDifferentaitor,
			&servo->PID.extProprotinal,
			&servo->PID.outputLimited,

			&servo->profile.maxAcceleration,
			&servo->profile.maxVelocity,
			&servo->profile.trajectoryStatus,
			&servo->profile.trajectoryFollowing,
			&servo->profile.followingThreshold,

			&servo->id,
			&servo->firmwareVer,
			&servo->goalPosition,
			&servo->torqueEnable,
			&servo->motionDirection,
			&servo->maxMotorTemp,
			&servo->maxIntTemp,
			&servo->maxVoltage,
			&servo->minVoltage,
			&servo->maxCurrent,
			&servo->maxPosition,
			&servo->minPosition,
			&servo->velocity,
			&servo->acceleration,
			&servo->inMotion,
			&servo->motionThreshold,

			&servo->encoder.angle,

			&servo->intTemp.converData,
			&servo->intTemp.extIIRFilterCoefficient,

			&servo->motorTemp.converData,
			&servo->motorTemp.extIIRFilterCoefficient,

			&servo->batteryVoltage.converData,
			&servo->batteryVoltage.extIIRFilterCoefficient,

			&servo->motorCurrent.converData,
			&servo->motorCurrent.extIIRFilterCoefficient,

			&servo->motor.dutyCycle,
			&servo->motor.direction,
			&servo->motor.frictionCompensation,

			&servo->imu.extIIRFilterCoefficient,
			&servo->imu.extCFilterCoefficient,
			&servo->imu.extAngleX,
			&servo->imu.extAngleY

	};

	uint8_t readWritePrivlagesCopyFrom[47] = {

			1, // Kp
			1, // Ki
			1, // Kd
			0, // setPoint
			0, // Error
			1, // lpfConstant
			0, // extIntegrator
			0, // extDifferentaitor
			0, // extProprotinal
			0, // outputLimited

			1, // maxAcceleration
			1, // maxVelocity
			0, // trajectoryStatus
			0, // trajectoryFollowing
			1, // followingThreshold

			0, // id
			0, // firmwareVer
			1, // goalPosition
			0, // torqueEnable, Changed via torque on command
			0, // motionDirection, Changed via torque on command
			1, // maxMotorTemp
			1, // maxIntTemp
			1, // maxVoltage
			1, // minVoltage
			1, // maxCurrent
			1, // maxPosition
			1, // minPosition
			0, // velocity
			0, // acceleration
			0, // inMotion
			1, // motionThreshold

			0, // angle

			0, // converData
			1, // extIIRFilterCoefficient

			0, // converData
			1, // extIIRFilterCoefficient

			0, // converData
			1, // extIIRFilterCoefficient

			0, // converData
			1, // extIIRFilterCoefficient

			0, // dutyCycle
			0, // direction
			1, // frictionCompensation

			1, // extIIRFilterCoefficient
			1, // extCFilterCoefficient
			0, // extAngleX
			0, // extAngleY

	};

	uint8_t variablesSizeCopyFrom[47] = {

			1, // Kp
			1, // Ki
			1, // Kd
			1, // setPoint
			1, // Error
			1, // lpfConstant
			1, // extIntegrator
			1, // extDifferentaitor
			1, // extProprotinal
			1, // outputLimited

			1, // maxAcceleration
			1, // maxVelocity
			0, // trajectoryStatus
			0, // trajectoryFollowing
			1, // followingThreshold

			0, // id
			0, // firmwareVer
			1, // goalPosition
			0, // torqueEnable, Changed via torque on command
			0, // motionDirection, Changed via torque on command
			1, // maxMotorTemp
			1, // maxIntTemp
			1, // maxVoltage
			1, // minVoltage
			1, // maxCurrent
			1, // maxPosition
			1, // minPosition
			1, // velocity
			1, // acceleration
			0, // inMotion
			1, // motionThreshold

			1, // angle

			1, // converData
			0, // extIIRFilterCoefficient

			1, // converData
			0, // extIIRFilterCoefficient

			1, // converData
			0, // extIIRFilterCoefficient

			1, // converData
			0, // extIIRFilterCoefficient

			1, // dutyCycle
			0, // direction
			0, // frictionCompensation

			0, // extIIRFilterCoefficient
			0, // extCFilterCoefficient
			1, // extAngleX
			1, // extAngleY

	};

	/*   Copy data into struct   */
	for (int i = 0; i < 47; ++i ) {

			servo->variables[i] 			= variablesCopyFrom[i];

			servo->readWritePrivlages[i]	= readWritePrivlagesCopyFrom[i];

			servo->variablesSize[i] 		= variablesSizeCopyFrom[i];

		}

}

/*   Encoder initialisation   */
void encoderInit(SERVO_CONTROL *servo, SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *csPort, uint16_t csPin) {

	/*   Initialise status variable   */
	uint8_t status = 0;

	status += AS5048A_Init(&servo->encoder, spiHandle, csPort, csPin, servo->motionDirection);

	/*   Check status   */
	if ( status != 0 ) {

		servo->hardwareError.errorInit			= SERVO_ERROR;
		servo->hardwareError.errorEncoder		= SERVO_ERROR;
		errorHandeler(servo);

	}

}

/*   CAN-bus initialisation    */
void canInit(SERVO_CONTROL *servo, CAN_HandleTypeDef *canHandle, uint8_t canDeviceID) {

	/*   Initialise status variable   */
	uint8_t status = 0;

	status += CAN_Init(&servo->can, canHandle, canDeviceID);

	/*   Check status   */
	if ( status != 0 ) {

			servo->hardwareError.errorInit			= SERVO_ERROR;
			servo->hardwareError.errorCAN			= SERVO_ERROR;
			errorHandeler(servo);

		}

}

/*   Motor H-Bridge initialisation   */
void motorInit(SERVO_CONTROL *servo, TIM_HandleTypeDef *timHandle, uint8_t timChannel, GPIO_TypeDef *hswAPort, uint16_t hswAPin, GPIO_TypeDef *hswBPort, uint16_t hswBPin) {

	/*   Initialise status variable   */
	uint8_t status = 0;

	pwmInit(&servo->motor, timHandle, timChannel, hswAPort, hswAPin, hswBPort, hswBPin, SERVO_STATIC_FRICTION);

	/*   Check status   */
	if ( status != 0 ) {

			servo->hardwareError.errorInit			= SERVO_ERROR;
			servo->hardwareError.errorPWM			= SERVO_ERROR;
			errorHandeler(servo);

		}

}


/*   ADC sensors initialisations   */
void sensorsInit(SERVO_CONTROL *servo, ADC_HandleTypeDef *adcHandle0, ADC_HandleTypeDef *adcHandle1, ADC_HandleTypeDef *adcHandle2, ADC_HandleTypeDef *adcHandle3, OPAMP_HandleTypeDef *hopamp) {

	/*   Initialise status variable   */
	uint8_t status = 0;

	/*   Initialise sensor ADCs   */
	status += adcSensorInit(&servo->intTemp, 			rawToIntTemp, 	adcHandle0);

	status += adcSensorInit(&servo->motorCurrent,	 	rawToCurrent, 	adcHandle1);

	status += adcSensorInit(&servo->batteryVoltage, 	rawToVoltage, 	adcHandle2);

	status += adcSensorInit(&servo->motorTemp, 		rawToMotorTemp, adcHandle3);

	/*   Start current sensing opamp   */
	status += HAL_OPAMP_Start(hopamp);

	/*   Check status   */
	if ( status != 0 ) {

			servo->hardwareError.errorInit			= SERVO_ERROR;
			servo->hardwareError.errorPWM			= SERVO_ERROR;
			errorHandeler(servo);

		}

}


/*   Sensor Data check    */
void sensorsCheck(SERVO_CONTROL *servo) {

	/*   Initialise status variable   */
	uint8_t status = 0;

	/*   Check each sensor value   */
	status += adcSensorRangeCheck(&servo->intTemp, 			&servo->hardwareError.overTempMCU, SERVO_MAX_INTERNAL_TEMP, 0);

	status += adcSensorRangeCheck(&servo->motorTemp, 		&servo->hardwareError.overTempMCU, SERVO_MAX_MOTOR_TEMP, 0);

	status += adcSensorRangeCheck(&servo->batteryVoltage, 	&servo->hardwareError.overTempMCU, SERVO_MAX_VOLTAGE, SERVO_MIN_VOLTAGE);

	status += adcSensorRangeCheck(&servo->motorCurrent, 	&servo->hardwareError.overTempMCU, SERVO_MAX_CURRENT, 0);

	/*   If range error has occurred call error handler   */
	if ( status != 0 ) {

		errorHandeler(servo);

	}

}


/*   PID value initialisation   */
void pidInit(SERVO_CONTROL *servo) {

	/*   Initialise PID sample period value   */
	servo->PID.samplePeriod   	= 	SERVO_PID_INTERVAL;

	/*   Initialise PID output limits   */
	servo->PID.outputMax      	=  	SERVO_PID_MAX;
	servo->PID.outputMin      	=  	SERVO_PID_MIN;

	/*   Initialise PID gains   */
	servo->PID.Kp 			  	= 	SERVO_PID_KP;
	servo->PID.Kd 			  	= 	SERVO_PID_KD;
	servo->PID.Ki 			  	= 	SERVO_PID_KI;

	/*   Initialise low pass filter parameter   */
	servo->PID.lpfConstant 	  	= 	SERVO_PID_LPF;

	/*   Initialise previous values    */
	servo->PID.prevInput 	  	= 	0;

	/*   Initialise PID terms    */
	servo->PID.differentaitor 	= 	0;
	servo->PID.integrator	  	= 	0;

}


/*   PID calculate and update   */
void pidUpdate(SERVO_CONTROL *servo) {


	/*   Calculate internal values from external   */
	float time 		= servo->PID.samplePeriod 	/ 1000.0f;
	float filter 	= servo->PID.lpfConstant 	/ 1000.0f;


	/*   Update PID input   */
	servo->PID.Input 			= 	servo->encoder.angle;

	/*   Calculate error   */
	servo->PID.Error      		= 	servo->PID.setPoint - servo->PID.Input;

	/*   Calculate PID proportional term   */
	servo->PID.proportinal 		= 	( servo->PID.Kp / 1000.0f ) * servo->PID.Error;


	/*   Calculate PID integrator term   */				// Disable change if torque is off.
	servo->PID.integrator 		= 	servo->PID.integrator + servo->torqueEnable * 0.5f * ( servo->PID.Ki / 1000.0f ) * time *
								  ( servo->PID.Error + servo->PID.prevError );

	/*   Calculate PID dynamic anti whind-up limits   */
	if ( servo->PID.outputMax > servo->PID.proportinal )  				servo->PID.integratorLimitMax 	= ( servo->PID.outputMax - servo->PID.proportinal );
	else 																servo->PID.integratorLimitMax 	= 0.0f;

	if ( servo->PID.outputMin < servo->PID.proportinal )  				servo->PID.integratorLimitMin 	= ( servo->PID.outputMin - servo->PID.proportinal );
	else 																servo->PID.integratorLimitMin 	= 0.0f;

    /*   Apply calculated clamping limits   */
	if 		( servo->PID.integrator > servo->PID.integratorLimitMax )   servo->PID.integrator 			= servo->PID.integratorLimitMax;
	else if ( servo->PID.integrator < servo->PID.integratorLimitMin ) 	servo->PID.integrator 			= servo->PID.integratorLimitMin;

	/*   Calculate velocity   */
	servo->velocity 	  		= ( ( 2.0f * ( servo->PID.Input - servo->PID.prevInput ) ) +
			  	  	  	  	  	    ( 2.0f * filter - time ) * servo->velocity ) /
			  	  	  	  	  	    ( 2.0f * filter + time );

	/*   Calculate acceleration   */
	servo->acceleration 	  	= ( ( 2.0f * ( servo->velocity   - servo->prevVelocity ) ) +
  	  	  	    					( 2.0f * filter - time ) * servo->acceleration ) /
  	  	  	    					( 2.0f * filter + time );

	/*   Calculate PID differentaitor term   */
	servo->PID.differentaitor 	= 	( ( 2.0f * ( servo->PID.Kd / 100000.0f )  * ( servo->PID.Error - servo->PID.prevError ) ) +
									  ( 2.0f * filter - time ) * servo->PID.differentaitor ) /
									  ( 2.0f * filter + time );

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

    /*   Update external PID values   */
    servo->PID.extProprotinal			= servo->PID.proportinal;
	servo->PID.extDifferentaitor		= servo->PID.differentaitor;
	servo->PID.extIntegrator			= servo->PID.integrator;

    /*   Update inMotion variable   */
    servo->inMotion = ( ( abs( servo->velocity ) >= servo->motionThreshold ) || ( abs( servo->acceleration ) >= servo->motionThreshold ) );			// 1 if velocity or acceleration is above threshold, only 0 if stationary and not changing velocity.

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
void motionPorfile(SERVO_CONTROL *servo) {

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
	servo->profile.trajectoryTime += servo->PID.samplePeriod * !!servo->profile.trajectoryStatus;				// Increment by time step (ms) if trajectory is not yet complete.

	/*   Update trajectoryFollowing variable   */
	servo->profile.trajectoryFollowing = ( abs( servo->PID.setPoint - servo->encoder.angle ) <= servo->profile.followingThreshold );

}


/*   IMU initialisations   */
void imuInit(SERVO_CONTROL *servo, SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *csPort, uint16_t csPin) {

	/*   Initialise status variable   */
	uint8_t status = 0;

	status += LSM6DSO_Init(&servo->imu, spiHandle, csPort, csPin, servo->PID.samplePeriod);

	/*   Check status   */
	if ( status != 0 ) {

			servo->hardwareError.errorInit			= SERVO_ERROR;
			servo->hardwareError.errorIMU			= SERVO_ERROR;
			errorHandeler(servo);

		}

}


/*   IMU update angle   */
void imuUpdateAngle(SERVO_CONTROL *servo) {

	/*   Read and update sensor data   */
	LSM6DSO_ReadSensors(&servo->imu);

	/*   Calculate angles from complementary filter   */
	LSM6DSO_EstimateOrientation(&servo->imu);

}

/*   Motor power control functions   */
void torqueDisable(SERVO_CONTROL *servo) {

	servo->torqueEnable = SERVO_TORQUE_DISABLE;

}


void torqueEnable(SERVO_CONTROL *servo, uint8_t direction) {

	servo->motionDirection = direction;

	encoderUpdate	(servo);
	servo->goalPosition = servo->encoder.angle;
	servo->torqueEnable = SERVO_TORQUE_ENABLE;

}


/*   General error handler   */
void errorHandeler(SERVO_CONTROL *servo) {

//STUFF GOES HERE

}


/*   Higher level CAN-bus functions   */
void processCanMessages(SERVO_CONTROL *servo, uint32_t RxFifo) {

	/*   Initialise message data variables   */
	uint8_t 			RxBuf[8];
	uint8_t 			operationId;
	uint8_t				senderId;
	uint8_t				priority;

	/*   While new messages are available   */
	while (  HAL_CAN_GetRxFifoFillLevel(servo->can.canHandle, RxFifo) > 0 ) {

		/*   Get message   */
		CAN_GetFrame(&servo->can, RxFifo, RxBuf, &senderId, &operationId, &priority);

		/*   Check if operation id is within range and operation is defined    */
		if ( ( operationId < 15 ) && ( servo->OperationFunctions[operationId] != NULL ) ) {

			/*   Call operation function   */
			OperationFucntionPointer function = ( OperationFucntionPointer ) servo->OperationFunctions[operationId];

			function(servo, RxBuf, servo->variables, servo->readWritePrivlages, servo->variablesSize, operationId, priority);

		}

	}

}


/*   Operation functions   */
void writeSingle(SERVO_CONTROL *servo, uint8_t *RxBuf, void** variables, uint8_t* readWritePrivlages, uint8_t* variablesSize, uint8_t operationId, uint8_t priority) {

	/*   Initialise address variable   */
	uint8_t adress = RxBuf[0];

	/*   If address is inside range and has write privilege   */
	if ( ( adress <= 46 )  && ( readWritePrivlages[adress] == 1 ) ) {

			/*   If variable is 16-bit number   */
			if ( variablesSize[adress] == 1 ) {

				*(uint16_t*)variables[adress] = ( ( RxBuf[1] << 8 ) | RxBuf[2] );

			}

			/*   If variable is 8-bit number   */
			else {

				*(uint8_t*)variables[adress] = RxBuf[2];

			}

	}

	/*   Send back read message   */
	readSingle(servo, RxBuf, variables, readWritePrivlages, variablesSize, operationId, priority);		// Value wont have changed if above if statement  in false;

}


void readSingle(SERVO_CONTROL *servo, uint8_t *RxBuf, void** variables, uint8_t* readWritePrivlages, uint8_t* variablesSize, uint8_t operationId, uint8_t priority) {

	/*   Initialise address variable   */
	uint8_t adress = RxBuf[0];


	/*   If address is inside range   */
	if ( adress <= 46 ) {

		/*   If variable is 16-bit number   */
		if ( variablesSize[adress] == 1 ) {

			/*   Initialise transmission buffer   */
			uint8_t TxBuf[3];

			/*   Set read address byte   */
			TxBuf[0] = adress;

			uint16_t readData =  *(uint16_t*)variables[adress];

			/*   Set data bytes in array   */
			TxBuf[1] = ( readData >> 8 ) & 0xFF;
			TxBuf[2] =   readData        & 0xFF;

			CAN_SendDataFrame(&servo->can, TxBuf, 3, priority, operationId);

		}

		/*   If variable is 8-bit number   */
		else {

			/*   Initialise transmission buffer   */
			uint8_t TxBuf[2];

			/*   Set read address byte   */
			TxBuf[0] = adress;

			uint8_t readData =  *(uint8_t*)variables[adress];

			/*   Set data bytes in array   */
			TxBuf[1] = readData;

			/*   Send response frame   */
			CAN_SendDataFrame(&servo->can, TxBuf, 2, priority, operationId);

		}

	}

}


void torqueOnCommand(SERVO_CONTROL *servo, uint8_t *RxBuf, void** variables, uint8_t* readWritePrivlages, uint8_t* variablesSize, uint8_t operationId, uint8_t priority) {

	/*   Servo  command    */
	torqueEnable(servo, RxBuf[0] );		// First byte is direction

	/*   Initialise transmission buffer   */
	uint8_t TxBuf[2];

	/*   Set read torqueEnable byte   */
	TxBuf[0] = *(uint8_t*)variables[18];

	/*   Set read motionDirection byte   */
	TxBuf[1] = *(uint8_t*)variables[19];

	/*   Send response frame   */
	CAN_SendDataFrame(&servo->can, TxBuf, 2, priority, operationId);

}

void torqueOffCommand(SERVO_CONTROL *servo, uint8_t *RxBuf, void** variables, uint8_t* readWritePrivlages, uint8_t* variablesSize, uint8_t operationId, uint8_t priority) {

	/*   Servo  command    */
	torqueDisable(servo);

	/*   Initialise transmission buffer   */
	uint8_t TxBuf[1];

	/*   Set read torqueEnable byte   */
	TxBuf[0] = *(uint8_t*)variables[18];

	/*   Send response frame   */
	CAN_SendDataFrame(&servo->can, TxBuf, 1, priority, operationId);

}


void startupCommand		(SERVO_CONTROL *servo, uint8_t *RxBuf, void** variables, uint8_t* readWritePrivlages, uint8_t* variablesSize, uint8_t operationId, uint8_t priority) {

	//TO do

}


void shutdownCommand	(SERVO_CONTROL *servo, uint8_t *RxBuf, void** variables, uint8_t* readWritePrivlages, uint8_t* variablesSize, uint8_t operationId, uint8_t priority) {

	//TO do

}


void writeConfig1		(SERVO_CONTROL *servo, uint8_t *RxBuf, void** variables, uint8_t* readWritePrivlages, uint8_t* variablesSize, uint8_t operationId, uint8_t priority) {

	/*   Write values    */
	*(uint16_t*)variables[0] = ( ( RxBuf[0] << 8 ) | RxBuf[1] );	// Kp
	*(uint16_t*)variables[1] = ( ( RxBuf[2] << 8 ) | RxBuf[3] );	// Ki
	*(uint16_t*)variables[2] = ( ( RxBuf[4] << 8 ) | RxBuf[5] );	// Kd
	*(uint16_t*)variables[5] = ( ( RxBuf[6] << 8 ) | RxBuf[7] );	// Lpf

}

void writeConfig2		(SERVO_CONTROL *servo, uint8_t *RxBuf, void** variables, uint8_t* readWritePrivlages, uint8_t* variablesSize, uint8_t operationId, uint8_t priority) {

	/*   Write values    */
	*(uint16_t*)variables[10] = ( ( RxBuf[0] << 8 ) | RxBuf[1] );	// maxAcceleration
	*(uint16_t*)variables[11] = ( ( RxBuf[2] << 8 ) | RxBuf[3] );	// maxVelocity
	*(uint16_t*)variables[14] = ( ( RxBuf[4] << 8 ) | RxBuf[5] );	// followingThreshold
	*(uint16_t*)variables[30] = ( ( RxBuf[6] << 8 ) | RxBuf[7] );	// motionThreshold

}

void writeConfig3		(SERVO_CONTROL *servo, uint8_t *RxBuf, void** variables, uint8_t* readWritePrivlages, uint8_t* variablesSize, uint8_t operationId, uint8_t priority) {

	/*   Write values    */
	*(uint16_t*)variables[20] = ( ( RxBuf[0] << 8 ) | RxBuf[1] );	// maxMotorTemp
	*(uint16_t*)variables[21] = ( ( RxBuf[2] << 8 ) | RxBuf[3] );	// maxIntTemp
	*(uint16_t*)variables[22] = ( ( RxBuf[4] << 8 ) | RxBuf[5] );	// maxVoltage
	*(uint16_t*)variables[23] = ( ( RxBuf[6] << 8 ) | RxBuf[7] );	// minVoltage

}

void writeConfig4		(SERVO_CONTROL *servo, uint8_t *RxBuf, void** variables, uint8_t* readWritePrivlages, uint8_t* variablesSize, uint8_t operationId, uint8_t priority) {

	/*   Write values    */
	*(uint16_t*)variables[24] = ( ( RxBuf[0] << 8 ) | RxBuf[1] );	// maxCurrent
	*(uint16_t*)variables[25] = ( ( RxBuf[2] << 8 ) | RxBuf[3] );	// maxPosition
	*(uint16_t*)variables[26] = ( ( RxBuf[4] << 8 ) | RxBuf[5] );	// minPosition

}

void writeConfig5		(SERVO_CONTROL *servo, uint8_t *RxBuf, void** variables, uint8_t* readWritePrivlages, uint8_t* variablesSize, uint8_t operationId, uint8_t priority) {

	/*   Write values    */
	*(uint8_t*)variables[33] = RxBuf[0];	// ADC - int temp 			- extIIRFilterCoefficient
	*(uint8_t*)variables[35] = RxBuf[1];	// ADC - motor temp 		- extIIRFilterCoefficien
	*(uint8_t*)variables[37] = RxBuf[2];	// ADC - battery voltage 	- extIIRFilterCoefficient
	*(uint8_t*)variables[39] = RxBuf[3];	// ADC - current motor 		- extIIRFilterCoefficient
	*(uint8_t*)variables[42] = RxBuf[4];	// frictionCompensation
	*(uint8_t*)variables[43] = RxBuf[5];	// extIIRFilterCoefficient
	*(uint8_t*)variables[44] = RxBuf[6];	// extCFilterCoefficient

}


void readconfig1			(SERVO_CONTROL *servo, uint8_t *RxBuf, void** variables, uint8_t* readWritePrivlages, uint8_t* variablesSize, uint8_t operationId, uint8_t priority) {

	/*   Initialise transmission buffer   */
	uint16_t TxBuf[4];

	/*   Read values   */
	TxBuf[0] =  *(uint16_t*)variables[0];
	TxBuf[1] =  *(uint16_t*)variables[1];
	TxBuf[2] =  *(uint16_t*)variables[2];
	TxBuf[3] =  *(uint16_t*)variables[5];

	/*   Send back response frame to confirm change   */
	CAN_SendDataFrame(&servo->can, (uint8_t*)TxBuf, 8, priority, 11);

}

void readconfig2			(SERVO_CONTROL *servo, uint8_t *RxBuf, void** variables, uint8_t* readWritePrivlages, uint8_t* variablesSize, uint8_t operationId, uint8_t priority) {

	/*   Initialise transmission buffer   */
	uint16_t TxBuf[3];

	/*   Read values   */
	TxBuf[0] =  *(uint16_t*)variables[10];
	TxBuf[1] =  *(uint16_t*)variables[11];
	TxBuf[2] =  *(uint16_t*)variables[14];
	TxBuf[3] =  *(uint16_t*)variables[30];

	/*   Send back response frame to confirm change   */
	CAN_SendDataFrame(&servo->can, (uint8_t*)TxBuf, 6, priority, 12);


}

void readconfig3			(SERVO_CONTROL *servo, uint8_t *RxBuf, void** variables, uint8_t* readWritePrivlages, uint8_t* variablesSize, uint8_t operationId, uint8_t priority) {

	/*   Initialise transmission buffer   */
	uint16_t TxBuf[4];

	/*   Read values   */
	TxBuf[0] =  *(uint16_t*)variables[20];
	TxBuf[1] =  *(uint16_t*)variables[21];
	TxBuf[2] =  *(uint16_t*)variables[22];
	TxBuf[3] =  *(uint16_t*)variables[23];

	/*   Send back response frame to confirm change   */
	CAN_SendDataFrame(&servo->can, (uint8_t*)TxBuf, 8, priority, 13);

}

void readconfig4			(SERVO_CONTROL *servo, uint8_t *RxBuf, void** variables, uint8_t* readWritePrivlages, uint8_t* variablesSize, uint8_t operationId, uint8_t priority) {

	/*   Initialise transmission buffer   */
	uint16_t TxBuf[4];

	/*   Read values   */
	TxBuf[0] =  *(uint16_t*)variables[24];
	TxBuf[1] =  *(uint16_t*)variables[25];
	TxBuf[2] =  *(uint16_t*)variables[26];

	/*   Send back response frame to confirm change   */
	CAN_SendDataFrame(&servo->can, (uint8_t*)TxBuf, 8, priority, 14);

}

void readconfig5			(SERVO_CONTROL *servo, uint8_t *RxBuf, void** variables, uint8_t* readWritePrivlages, uint8_t* variablesSize, uint8_t operationId, uint8_t priority) {

	/*   Initialise transmission buffer   */
	uint8_t TxBuf[7];

	/*   Read values   */
	TxBuf[0] =  *(uint8_t*)variables[33];
	TxBuf[1] =  *(uint8_t*)variables[35];
	TxBuf[2] =  *(uint8_t*)variables[37];
	TxBuf[3] =  *(uint8_t*)variables[39];
	TxBuf[4] =  *(uint8_t*)variables[42];
	TxBuf[5] =  *(uint8_t*)variables[43];
	TxBuf[6] =  *(uint8_t*)variables[44];

	/*   Send back response frame to confirm change   */
	CAN_SendDataFrame(&servo->can, TxBuf, 7, priority, 15);

}


void writeStandard		(SERVO_CONTROL *servo, uint8_t *RxBuf, void** variables, uint8_t* readWritePrivlages, uint8_t* variablesSize, uint8_t operationId, uint8_t priority) {



}





