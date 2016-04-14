/*
	Andrew Goodman
	March 12, 2016

	Implement the robot class
*/
#include "Robot.h"
#include <NewPing.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include "RF24.h"

// Left ultrsonic sensor
#define LEFT_TRIGGER_PIN 53
#define LEFT_ECHO_PIN 52
#define LEFT_MAX_DISTANCE 300

// Right ultrsonic sensor
#define RIGHT_TRIGGER_PIN 50
#define RIGHT_ECHO_PIN 51
#define RIGHT_MAX_DISTANCE 300

// Left sensor
NewPing leftPingSensor(LEFT_TRIGGER_PIN,LEFT_ECHO_PIN,LEFT_MAX_DISTANCE);
// Right sensor
NewPing rightPingSensor(RIGHT_TRIGGER_PIN,RIGHT_ECHO_PIN,RIGHT_MAX_DISTANCE);

// Motor objects
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *LEFT_MOTOR = AFMS.getMotor(4);
Adafruit_DCMotor *RIGHT_MOTOR = AFMS.getMotor(1);

/*
	Robot constructor

	@param - stopLED
	The pin number that controls the PWM of the stop LED
	@param - goLED
	The pin number that controls the PWM of the go LED
*/
Robot::Robot(int stopLED,int goLED)
{	
	// Assign LED pin numbers
	STOP_LED = stopLED;
	GO_LED = goLED;

	// Set initial value for infrared proximity sensor
	irDistSensor = 0;

	// Set LEDs to output voltage
	pinMode(STOP_LED,OUTPUT);
	pinMode(GO_LED,OUTPUT);

	// Test LEDs
	digitalWrite(STOP_LED,LOW);
	digitalWrite(GO_LED,LOW);
}

void Robot::startMotors()
{
	// Start Adafruit motor shield
	AFMS.begin();

	// Set initial speed to 0
	currentSpeed = 0;

	// Set initial values for the motors
	LEFT_MOTOR->setSpeed(currentSpeed);
	RIGHT_MOTOR->setSpeed(currentSpeed);
	LEFT_MOTOR->run(FORWARD);
	RIGHT_MOTOR->run(FORWARD);
	LEFT_MOTOR->run(RELEASE);
	RIGHT_MOTOR->run(RELEASE);	
}

/*
	Check for obstacles

	return values:
	0 - No obstacles
	1 - obstacle detected on left ping sensor
	2 - obstacle detected on right ping sensor
	3 - obstacle detected on IR distance sensor
*/
int Robot::checkObstacles()
{
	// Send ping signal in microseconds (uS) from both left and right
	unsigned int right_uS = rightPingSensor.ping_median();
	unsigned int left_uS = leftPingSensor.ping_median();

	// IR distance sensor
	// Value meanings:
	// < 90 = infinite distance
	// < 147 = 50cm; 20in
	//irDistSensor = analogRead(0);

	delay(10);

	// If no object within 50 cm or if ping is unresponsive
	// indicating that there is an open space greater
	// than MAX_DISTANCE ahead
	if((leftPingSensor.convert_cm(left_uS) > 50  || !leftPingSensor.ping())
		&& (rightPingSensor.convert_cm(right_uS) > 50  || !rightPingSensor.ping())
		/*&& (irDistSensor < 147 || irDistSensor < 100)*/)
	{
		// No obstacles ahead, output drive command
		return 0;
	}
	else// if(leftPingSensor.convert_cm(left_uS) <= 50)
	{
		// Obstacle detected on left side
		// output stop command
		// turn right
		return 1;
	}
//	else if(rightPingSensor.convert_cm(left_uS) <= 50)
//	{
//		// Obstacle detected on right side
//		// output stop command
//		// turn left
//		return 2;
//	}
}

/*
	Determine which state to enter
*/
void Robot::actions(int state)
{

}

//******Begin move states******
// Send power to the motors
void Robot::nav(int obstacleSeen)
{
	// If no obstacle is detected
	if(obstacleSeen == 0)
	{	
		// Green light = drive
		digitalWrite(STOP_LED,LOW);
		digitalWrite(GO_LED,HIGH);
	
		LEFT_MOTOR->run(FORWARD);
		RIGHT_MOTOR->run(FORWARD);
	
		// Accelerate motors to full speed
		// in 2040 milliseconds
		if(currentSpeed == 0)
		{
			while(currentSpeed <= 180)
			{
				LEFT_MOTOR->setSpeed(currentSpeed);
				RIGHT_MOTOR->setSpeed(currentSpeed);
				currentSpeed++;

				// If any obstacle encountered, then break out of acceleration
				if(checkObstacles() == 1)
				{
					break;
					//enterMoveState();
				}
			}
			goForward();
			halt();
		}
//		else
//		{
//			while()
//		}
		
		RIGHT_MOTOR->run(RELEASE);
		LEFT_MOTOR->run(RELEASE);
	}
}

void Robot::goForward()
{
	LEFT_MOTOR->run(FORWARD);
	RIGHT_MOTOR->run(FORWARD);
	
	while(1)
	{
		LEFT_MOTOR->setSpeed(currentSpeed);
		RIGHT_MOTOR->setSpeed(currentSpeed);

		if(checkObstacles() == 1)
		{
			break;
		}
	}

	RIGHT_MOTOR->run(RELEASE);
	LEFT_MOTOR->run(RELEASE);
}

// Stop sending power to the motors
void Robot::halt()
{
	//uint8_t decel;
	
	// Red light = STOP!
	digitalWrite(GO_LED,LOW);
	digitalWrite(STOP_LED,HIGH);

	// Decelerate motors to until they stop
	while(currentSpeed > 0)
	{
		LEFT_MOTOR->setSpeed(currentSpeed);
		RIGHT_MOTOR->setSpeed(currentSpeed);
		currentSpeed--;
	}

	RIGHT_MOTOR->run(RELEASE);
	LEFT_MOTOR->run(RELEASE);
}

void Robot::reverse()
{
//	uint8_t accel;
	
	// Red light = STOP!
	digitalWrite(GO_LED,HIGH);
	digitalWrite(STOP_LED,HIGH);

	LEFT_MOTOR->run(BACKWARD);
	RIGHT_MOTOR->run(BACKWARD);
	
	// Accelerate motors to full speed
	// in 2040 milliseconds
	for(currentSpeed = 0; currentSpeed <= 240; currentSpeed++)
	{
		delay(8);
		LEFT_MOTOR->setSpeed(currentSpeed);
		RIGHT_MOTOR->setSpeed(currentSpeed);
	}
	// Stop motors
	LEFT_MOTOR->setSpeed(0);
	RIGHT_MOTOR->setSpeed(0);

	RIGHT_MOTOR->run(RELEASE);
	LEFT_MOTOR->run(RELEASE);
}

// Send power to the left motor motor
void Robot::turnLeft()
{
	//uint8_t accel;
	
	LEFT_MOTOR->run(FORWARD);
	RIGHT_MOTOR->setSpeed(0);
	//LEFT_MOTOR->setSpeed(255);

	for(currentSpeed = 0;currentSpeed <= 150;currentSpeed++)
	{
		delay(8);
		LEFT_MOTOR->setSpeed(currentSpeed);
	}

	RIGHT_MOTOR->run(RELEASE);
	LEFT_MOTOR->run(RELEASE);
}

// Send power to the right motor motor
void Robot::turnRight()
{
	//uint8_t accel;
	
	RIGHT_MOTOR->run(FORWARD);
	LEFT_MOTOR->setSpeed(0);

	for(currentSpeed = 0;currentSpeed <= 150;currentSpeed++)
	{
		delay(8);
		RIGHT_MOTOR->setSpeed(currentSpeed);
	}

	RIGHT_MOTOR->run(RELEASE);
	LEFT_MOTOR->run(RELEASE);
}
//******End move states******
