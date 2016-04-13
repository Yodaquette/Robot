/*
	Andrew Goodman
	Fevruary 20, 2016

	Robot Test
*/
#include "Robot.h"

// Instantiate robot
Robot rob(6,5);

void setup()
{
	rob.startMotors();
}

void loop()
{
	rob.nav(rob.checkObstacles());
}

