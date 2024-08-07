
#ifndef STM32F303RE_SERVO_DRIVER_INC_MOTOR_H
#define STM32F303RE_SERVO_DRIVER_INC_MOTOR_H

#include "stm32f3xx_hal.h"
#include "AS5048A.h"
#include "ADC.h"
#include "HBRIDGE.h"
#include "CAN.h"
#include "LSM6DSO.h"

/*   Default values   */

/*   Device information   */
#define ID							1						// 8 bit Device ID
#define MASTER_ID					0
#define FIRMWARE_VERSION			1

/*   Motion direction   */
#define SERVO_DIRECTION_CCW			0
#define SERVO_DIRECTION_CW	    	1

/*   Torque enable   */
#define SERVO_TORQUE_ENABLE  		1
#define SERVO_TORQUE_DISABLE 		0

/*   Motion profile ongoing   */
#define	SERVO_PROFILE_ONGOING   	1
#define	SERVO_PROFILE_COMPLETE  	0

/*   Motion profile split   */
#define SERVO_PROFILE_SPLIT_2		2
#define SERVO_PROFILE_SPLIT_1		1
#define	SERVO_PROFILE_RESET 		0

/*   Operating limits   */
#define SERVO_MAX_VOLTAGE			16000					// 16 V
#define SERVO_MIN_VOLTAGE			8000					// 8 V
#define SERVO_MAX_MOTOR_TEMP		8000					// 80 °C
#define SERVO_MAX_INTERNAL_TEMP		8000					// 80 °C
#define SERVO_MAX_CURRENT			3000					// 3 A
#define SERVO_PID_MAX				1023
#define SERVO_PID_MIN		   	   -1023
#define SERVO_MAX_ACCELERATION  	10000
#define SERVO_MAX_VELOCITY  		6000
#define SERVO_MAX_POSITION			16000					// Absolute position limits to avoid encoder loop-back
#define SERVO_MIN_POSITION			383

/*   Motion threshold velocity   */
#define SERVO_MOTION_THRESHOLD		100

/*   Profile following threshold deviation   */
#define SERVO_FOLLOWING_THRESHOLD	300

/*   Base PID loop sample interval and filter constant  */
#define SERVO_PID_INTERVAL			4						// 4 ms
#define SERVO_PID_LPF				100						// 0.1

/*   Data output period   */
#define SERVO_OUTPUT_INTERVAL		50

/*   Base PID GAINS   */
#define SERVO_PID_KP				3000					// 3.0
#define SERVO_PID_KD				3000					// 0.03
#define SERVO_PID_KI				250						// 0.25

/*   Static friction compensation value   */
#define SERVO_STATIC_FRICTION		55

/*   Error value   */
#define SERVO_ERROR 1
#define SERVO_OK	0

/*   Hardware error status   */
typedef struct {

	uint8_t 	rangeVoltage;						// Voltage not within range of maxVoltage and minVoltage
	uint8_t 	overTempMCU;						// Internal MCU temperature exceeding maxTempInt
	uint8_t 	overTempMotor;						// Motor temperature exceeding maxTempMotor
	uint8_t 	overCurrent;						// Current exceeding maxCurrent

	uint8_t 	errorInit;							// Failure of one of init functions
	uint8_t 	errorEncoder;						// Error with encoder communication
	uint8_t 	errorPWM;							// Error with motor drive
	uint8_t 	errorADC;							// Error with ADC calibration
	uint8_t 	errorIMU;							// Error with IMU communication
	uint8_t 	errorCAN;							// Error with CAN bus communication

} HW_ERR;

/*   PID controller settings and status   */
typedef struct {

	/*   PID gains   */
	int16_t  	Kp;
	int16_t  	Ki;
	int16_t  	Kd;				   				 	// Tracking Anti-windup gain

	/*   PID values   */
	float 		integrator;
	float 		differentaitor;
	float 		proportinal;

	/*   PID values external   */
	int16_t		extIntegrator;
	int16_t		extDifferentaitor;
	int16_t		extProprotinal;

	/*   PID  state   */
	uint16_t 	setPoint;
	int16_t  	Error;
	uint16_t 	Input;
	uint16_t 	prevInput;
	int16_t	 	prevError;

	/*   PID low-pass filter constant   */
	uint16_t 	lpfConstant;								//in frequency period?

	/*   PID output clamp filter limits   */
	int16_t 	outputMax;
	int16_t 	outputMin;

	/*   PID integrator anti whind-up dynamic clamping terms   */
	float integratorLimitMax;
	float integratorLimitMin;

	/*   PID sample period   */
	uint16_t 	samplePeriod;             				// mS

	/*   PID output   */
	int16_t 	output;

	/*   PID limited output   */
	int16_t 	outputLimited; 							// +- 1023

}PID_CTRL;

/*   Motion profile settings and status   */
typedef struct {

	/*   Motion profile acceleration phase parameters   */
	float 		trajectoryAccelerationTime;
	float 		trajectoryAccelerationDistance;

	/*   Motion profile constant-velocity phase parameters   */
	float 		trajectoryVelocityTime;
	float 		trajectoryVelocityDistance;

	/*   Motion profile deceleration phase parameters   */
	float 		trajectoryDecelerationTime;
	float 		trajectoryDecelerationDistance;

	/*   Motion profile parameters   */
	int16_t		trajectoryDistance;
	int8_t 		trajectoryDirection;						// 1 if positive -1 if negative
	int16_t		trajectoryVelocity;
	uint16_t    trajectoryGoalPosition;
	int32_t 	trajectoryStartPosition;
	int32_t 	trajectorySetPoint;
	int16_t     trajectoryStartVelocity;


	/*   Motion profile limits   */
	uint16_t  	maxAcceleration;							// Maximum acceleration
	uint16_t  	maxVelocity;								// Maximum velocity

	/*   Motion profile status   */
	uint16_t	trajectoryTime;
	uint8_t  	trajectoryStatus;
	uint8_t		trajectorySplit;							// Curve has been split for ease of calculation
	uint8_t  	trajectoryFollowing;						// Motor following profile trajectory
	uint16_t 	followingThreshold;							// Threshold deviation to be considered following profile

} PROFILE;

typedef struct {

	/*   Version and ID   */
	uint8_t  	id;										// 8 bit CAN motor identifier
	uint8_t		masterId;							    // 8 bit CAN master identifier
	uint8_t  	firmwareVer;							// Firmware version

	/*   Position and drive status   */
//	uint16_t 	homePosition;							// Base position offset relative to homed zero position.
	uint16_t 	goalPosition;							// 0-2^14
	uint8_t  	torqueEnable;
	uint8_t  	motionDirection;						// Motion direction 0 is CW 1 is CCW

	/*   Operating limits   */
	uint16_t 	maxMotorTemp;							// Centi°C
	uint16_t 	maxIntTemp;								// Centi°C
	uint16_t 	maxVoltage;								// mV
	uint16_t 	minVoltage;								// mV
	uint16_t 	maxCurrent;								// mA
//	uint16_t 	maxTorque;								// mNm
	uint16_t 	maxPosition;					    	// 0-2^14
	uint16_t 	minPosition;					    	// 0-2^14

	/*   Error status   */
	HW_ERR  	hardwareError;

	/*   PID parameters   */
	PID_CTRL  	PID;

	/*   Rotary encoder   */
	AS5048A 	encoder;

	/*   ADC Sensors   */
	ADC 		intTemp;
	ADC 		motorTemp;
	ADC 		batteryVoltage;
	ADC 		motorCurrent;

	/*   H-bridge motor drive   */
	MOTOR 		motor;

	/*   Motion profile parameters   */
	PROFILE 	profile;

	/*   CAN bus   */
	CANBUS 		can;

	/*   Inertial measurement unit   */
	LSM6DSO 	imu;

	/*   Motion status  */
//	int16_t 	load;										// Switches signs in CW vs CCW
	int16_t   	velocity;
	int16_t	  	prevVelocity;
	int16_t   	acceleration;
	uint8_t  	inMotion;									// Currently moving
	uint16_t 	motionThreshold;							// Threshold velocity to be considered moving

	/*   Operation function array   */
	void* 		OperationFunctions	[22];

	/*   Variable addressable list   */
	void*		variables			[47];
	uint8_t 	readWritePrivlages	[47];					// 0 is read only
	uint8_t 	variablesSize		[47];					// 0 8 bit, 1 is 16 bit

} SERVO_CONTROL;

/*   Operation function pointer   */
typedef void (*OperationFucntionPointer)(SERVO_CONTROL*, uint8_t*, uint8_t, uint8_t);


/*   Servo value initialisation   */
void servoInit(SERVO_CONTROL *servo);


/*   PID value initialisation   */
void pidInit(SERVO_CONTROL *servo);


/*   Initialise can bus */
void canInit(SERVO_CONTROL *servo, CAN_HandleTypeDef *canHandle);


/*   PID calculate and update   */
void pidUpdate(SERVO_CONTROL *servo);


/*   Update motor duty cycle value   */
void motorUpdatePWM(SERVO_CONTROL *servo);


/*   Encoder initialisation   */
void encoderInit(SERVO_CONTROL *servo, SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *csPort, uint16_t csPin);


/*   Encoder sensor update   */
void encoderUpdate(SERVO_CONTROL *servo);


/*   Motor H-Bridge initialisation   */
void motorInit(SERVO_CONTROL *servo, TIM_HandleTypeDef *timHandle, uint8_t timChannel, GPIO_TypeDef *hswAPort, uint16_t hswAPin, GPIO_TypeDef *hswBPort, uint16_t hswBPin);


/*   ADC sensors initialisation   */
void sensorsInit(SERVO_CONTROL *servo, ADC_HandleTypeDef *adcHandle0, ADC_HandleTypeDef *adcHandle1, ADC_HandleTypeDef *adcHandle2, ADC_HandleTypeDef *adcHandle3, OPAMP_HandleTypeDef *hopamp);


/*   IMU initialisation   */
void imuInit(SERVO_CONTROL *servo, SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *csPort, uint16_t csPin);


/*   IMU angle update and cacluation   */
void imuUpdateAngle(SERVO_CONTROL *servo);


/*   ADC sensors read   */
void sensorsUpdate(SERVO_CONTROL *servo);


/*   Sensor Data check    */
void sensorsCheck(SERVO_CONTROL *servo);


/*   Motion profile calculation    */
void motionPorfile(SERVO_CONTROL *servo);


/*   General error handler   */
void errorHandeler(SERVO_CONTROL *servo);


/*   Motor power control functions   */
void torqueDisable(SERVO_CONTROL *servo);

void torqueEnable(SERVO_CONTROL *servo, uint8_t direction);


/*   Status led control function   */
void ledSet(uint8_t Red, uint8_t Green, uint8_t Blue);


/*   Higher level CAN-bus functions   */
void processCanMessages(SERVO_CONTROL *servo, uint32_t RxFifo);

/*   Configuration data split and join function  */
void configSplit(SERVO_CONTROL *servo, uint8_t *RxBuf, uint8_t *addressList, uint8_t listLenght);

void configJoin(SERVO_CONTROL *servo, uint8_t *TxBuf, uint8_t *addressList, uint8_t listLenght);

/*   Operation functions   */
void writeSingle		(SERVO_CONTROL *servo, uint8_t *RxBuf, uint8_t operationId, uint8_t priority);
void readSingle			(SERVO_CONTROL *servo, uint8_t *RxBuf, uint8_t operationId, uint8_t priority);
void torqueOnCommand	(SERVO_CONTROL *servo, uint8_t *RxBuf, uint8_t operationId, uint8_t priority);
void torqueOffCommand	(SERVO_CONTROL *servo, uint8_t *RxBuf, uint8_t operationId, uint8_t priority);
void startupCommand		(SERVO_CONTROL *servo, uint8_t *RxBuf, uint8_t operationId, uint8_t priority);
void shutdownCommand	(SERVO_CONTROL *servo, uint8_t *RxBuf, uint8_t operationId, uint8_t priority);
void writeConfig1		(SERVO_CONTROL *servo, uint8_t *RxBuf, uint8_t operationId, uint8_t priority);
void writeConfig2		(SERVO_CONTROL *servo, uint8_t *RxBuf, uint8_t operationId, uint8_t priority);
void writeConfig3		(SERVO_CONTROL *servo, uint8_t *RxBuf, uint8_t operationId, uint8_t priority);
void writeConfig4		(SERVO_CONTROL *servo, uint8_t *RxBuf, uint8_t operationId, uint8_t priority);
void writeConfig5		(SERVO_CONTROL *servo, uint8_t *RxBuf, uint8_t operationId, uint8_t priority);
void readconfig1		(SERVO_CONTROL *servo, uint8_t *RxBuf, uint8_t operationId, uint8_t priority);
void readconfig2		(SERVO_CONTROL *servo, uint8_t *RxBuf, uint8_t operationId, uint8_t priority);
void readconfig3		(SERVO_CONTROL *servo, uint8_t *RxBuf, uint8_t operationId, uint8_t priority);
void readconfig4		(SERVO_CONTROL *servo, uint8_t *RxBuf, uint8_t operationId, uint8_t priority);
void readconfig5		(SERVO_CONTROL *servo, uint8_t *RxBuf, uint8_t operationId, uint8_t priority);
void writeInStandard	(SERVO_CONTROL *servo, uint8_t *RxBuf, uint8_t operationId, uint8_t priority);
void readInStandard		(SERVO_CONTROL *servo, uint8_t *RxBuf, uint8_t operationId, uint8_t priority);
void readStandard1		(SERVO_CONTROL *servo, uint8_t *RxBuf, uint8_t operationId, uint8_t priority);
void readStandard2		(SERVO_CONTROL *servo, uint8_t *RxBuf, uint8_t operationId, uint8_t priority);
void readStandard3		(SERVO_CONTROL *servo, uint8_t *RxBuf, uint8_t operationId, uint8_t priority);
void readStandard4		(SERVO_CONTROL *servo, uint8_t *RxBuf, uint8_t operationId, uint8_t priority);


#endif /* STM32F303RE_SERVO_DRIVER_INC_MOTOR_H */
