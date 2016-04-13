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
		Robot(int,int);
		void startMotors();
		int checkObstacles();
		void nav(int);
		void goForward();
		void halt();
		void reverse();
		void turnLeft();
		void turnRight();
	private:
		int STOP_LED,GO_LED;
		float irDistSensor;
		uint8_t currentSpeed;
};
#endif
/*END _robot_h*/
