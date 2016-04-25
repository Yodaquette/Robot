/*
	Andrew Goodman
	March 6, 2016
 
	Robot
*/
#ifndef _robot_h
#define _robot_h

#include "Arduino.h"
//#include <NewPing.h>
//#include <Adafruit_MotorShield.h>

// Robot object
class Robot
{
	public:
		Robot(int,int,int,int,int);
		void startMotors();
		int checkObstacles();
		void actions(int);
		int irReadMedian();
		void ramp();
		void goForward();
		void halt();
		void reverse();
		void turnLeft();
		void turnRight();
	private:
		int LTURN_LED,RTURN_LED,REVERSE_LED,STOP_LED,GO_LED;
		uint8_t currentSpeed, maxSpeed, turnSpeed;
};
#endif
/*END _robot_h*/
