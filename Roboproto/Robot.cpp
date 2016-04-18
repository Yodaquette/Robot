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
#include <RF24.h>

// Left ultrsonic sensor
#define LEFT_TRIGGER_PIN 53
#define LEFT_ECHO_PIN 52
#define LEFT_MAX_DISTANCE 300

// Right ultrsonic sensor
#define RIGHT_TRIGGER_PIN 50
#define RIGHT_ECHO_PIN 51
#define RIGHT_MAX_DISTANCE 300

// Front ultrsonic sensor
#define FRONT_TRIGGER_PIN 49
#define FRONT_ECHO_PIN 48
#define FRONT_MAX_DISTANCE 400

// Left sensor
NewPing leftPingSensor(LEFT_TRIGGER_PIN,LEFT_ECHO_PIN,LEFT_MAX_DISTANCE);
// Right sensor
NewPing rightPingSensor(RIGHT_TRIGGER_PIN,RIGHT_ECHO_PIN,RIGHT_MAX_DISTANCE);
// Front sensor
NewPing frontPingSensor(FRONT_TRIGGER_PIN,FRONT_ECHO_PIN,FRONT_MAX_DISTANCE);

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
Robot::Robot(int leftTurnLED,int rightTurnLED,int reverseLED,int stopLED,int goLED)
{	
	// Assign LED pin numbers
	LTURN_LED = leftTurnLED;
	RTURN_LED = rightTurnLED;
	REVERSE_LED = reverseLED;
	STOP_LED = stopLED;
	GO_LED = goLED;

	// Set LEDs to output voltage
	pinMode(LTURN_LED,OUTPUT);
	pinMode(RTURN_LED,OUTPUT);
	pinMode(REVERSE_LED,OUTPUT);
	pinMode(STOP_LED,OUTPUT);
	pinMode(GO_LED,OUTPUT);

	// Set all LEDs to off
	digitalWrite(LTURN_LED,LOW);
	digitalWrite(RTURN_LED,LOW);
	digitalWrite(REVERSE_LED,LOW);
	digitalWrite(STOP_LED,LOW);
	digitalWrite(GO_LED,LOW);
}

/*
	Prime the motors
*/
void Robot::startMotors()
{
	// Start Adafruit motor shield
	AFMS.begin();

	// Set initial speed to 0
	currentSpeed = 0;
	// Set max speed to 180
	maxSpeed = 180;
	// Set turn speed to 120
	turnSpeed = 120;

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
	unsigned int left_uS = leftPingSensor.ping_median();
	//unsigned int right_uS = rightPingSensor.ping_median();
	//unsigned int front_uS = frontPingSensor.ping_median();

	delay(10);

	// If no object within 50 cm or if ping is unresponsive
	// indicating that there is an open space greater
	// than MAX_DISTANCE ahead
//	if((leftPingSensor.convert_cm(left_uS) > 50  || !leftPingSensor.ping())
//		&& (rightPingSensor.convert_cm(right_uS) > 50  || !rightPingSensor.ping())
//		&& (frontPingSensor.convert_cm(front_uS) > 50 || !frontPingSensor.ping()))
//	{
//		// No obstacles ahead, output drive command
//		return 0;
//	}
	if(leftPingSensor.convert_cm(left_uS) <= 30)
	{
		// Obstacle detected on left side
		// output stop command
		// turn right
		return 1;
	}
//	else if(rightPingSensor.ping() == true && rightPingSensor.convert_cm(right_uS) <= 30)
//	{
//		// Obstacle detected on right side
//		// output stop command
//		// turn left
//		return 2;
//	}
//	else if(frontPingSensor.ping() == true && frontPingSensor.convert_cm(front_uS) <= 50)
//	{
//		// Obstacle detected in the front
//		// output stop command
//		// reverse
//		return 3;
//	}
	else
	{
		// Nothing detected, keep moving
		return 0;
	}
}

/*
	Determines which state to enter
	by the value returned from checkObstacles() function
*/
void Robot::actions(int state)
{
	switch (state)
	{
		case 0:
			ramp();
			goForward();
			break;
		case 1:
			halt();
			turnRight();
			break;
		case 2:
			halt();
			turnLeft();
			break;
		case 3:
			halt();
			reverse();
			break;
	}
}

//******Begin move states******
// Send power to the motors
void Robot::ramp()
{
	// Green light = drive
	digitalWrite(LTURN_LED,LOW);
	digitalWrite(RTURN_LED,LOW);
	digitalWrite(REVERSE_LED,LOW);
	digitalWrite(STOP_LED,LOW);
	digitalWrite(GO_LED,HIGH);

	LEFT_MOTOR->run(FORWARD);
	RIGHT_MOTOR->run(FORWARD);

	// Accelerate motors to maximum speed
	if(currentSpeed == 0)
	{
		while(currentSpeed <= maxSpeed)
		{
			LEFT_MOTOR->setSpeed(currentSpeed);
			RIGHT_MOTOR->setSpeed(currentSpeed);
			currentSpeed++;

			// If any obstacle encountered, then break out of acceleration
			if(checkObstacles() > 0)
			{
				// Break out of the acceleration loop
				break;
			}

			// Delay 0.01 seconds
			delay(10);
		}

		if (checkObstacles() > 0)
		{
			// Enter the actions function
			actions(checkObstacles());
		}
	}
	else
	{
		// If motors are already running then
		// keep going forward
		goForward();
	}
}

void Robot::goForward()
{
	// Green light = drive
	digitalWrite(LTURN_LED,LOW);
	digitalWrite(RTURN_LED,LOW);
	digitalWrite(REVERSE_LED,LOW);
	digitalWrite(STOP_LED,LOW);
	digitalWrite(GO_LED,HIGH);
	
	LEFT_MOTOR->run(FORWARD);
	RIGHT_MOTOR->run(FORWARD);
	
	while(1)
	{
		LEFT_MOTOR->setSpeed(currentSpeed);
		RIGHT_MOTOR->setSpeed(currentSpeed);

		// If anything detected, stop driving forward
		if(checkObstacles() > 0)
		{
			break;
		}
	}
}

// Stop sending power to the motors
void Robot::halt()
{	
	// Red light = STOP!
	digitalWrite(LTURN_LED,LOW);
	digitalWrite(RTURN_LED,LOW);
	digitalWrite(REVERSE_LED,LOW);
	digitalWrite(STOP_LED,HIGH);
	digitalWrite(GO_LED,LOW);

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
	// White light = REVERSE!
	digitalWrite(LTURN_LED,LOW);
	digitalWrite(RTURN_LED,LOW);
	digitalWrite(REVERSE_LED,HIGH);
	digitalWrite(GO_LED,LOW);
	digitalWrite(STOP_LED,LOW);

	LEFT_MOTOR->run(BACKWARD);
	RIGHT_MOTOR->run(BACKWARD);
	
	// Accelerate motors in reverse
	for(currentSpeed = 0; currentSpeed <= maxSpeed; currentSpeed++)
	{
		LEFT_MOTOR->setSpeed(currentSpeed);
		RIGHT_MOTOR->setSpeed(currentSpeed);

		// If the view is clear
		// break the loop and stop driving
		// in reverse
		if(checkObstacles() == 0)
		{
			break;	
		}

		delay(10);
	}
	// Stop motors
	LEFT_MOTOR->setSpeed(0);
	RIGHT_MOTOR->setSpeed(0);

	RIGHT_MOTOR->run(RELEASE);
	LEFT_MOTOR->run(RELEASE);
}

// Send forward power to the left motor
// and backward power to the right motor
void Robot::turnLeft()
{	
	// Yellow light = TURN LEFT
	digitalWrite(LTURN_LED,HIGH);
	digitalWrite(RTURN_LED,LOW);
	digitalWrite(GO_LED,LOW);
	digitalWrite(STOP_LED,LOW);
	digitalWrite(REVERSE_LED,LOW);
	
	LEFT_MOTOR->run(FORWARD);
	RIGHT_MOTOR->run(BACKWARD);

	// Turn left until no obstacles detected
	while(1)
	{
		LEFT_MOTOR->setSpeed(turnSpeed);
		RIGHT_MOTOR->setSpeed(turnSpeed);

		if(checkObstacles() == 0)
		{
			break;
		}
	}

	RIGHT_MOTOR->run(RELEASE);
	LEFT_MOTOR->run(RELEASE);
}

// Send backward power to the left motor
// and forward power to the right motor
void Robot::turnRight()
{
	// Yellow light = TURN
	digitalWrite(LTURN_LED,LOW);
	digitalWrite(RTURN_LED,HIGH);
	digitalWrite(REVERSE_LED,LOW);
	digitalWrite(GO_LED,LOW);
	digitalWrite(STOP_LED,LOW);
	
	LEFT_MOTOR->run(BACKWARD);
	RIGHT_MOTOR->run(FORWARD);

	// Turn right
	while(1)
	{
		LEFT_MOTOR->setSpeed(turnSpeed);
		RIGHT_MOTOR->setSpeed(turnSpeed);

		// Turn right until no obstacles detected
		if(checkObstacles() == 0)
		{
			break;
		}
	}

	RIGHT_MOTOR->run(RELEASE);
	LEFT_MOTOR->run(RELEASE);
}
//******End move states******
