/*
	Andrew Goodman
	February 20, 2016

	Run the robot
*/
#include "Robot.h"

// Instantiate robot
Robot rob(41,43,42,39,40);

void setup()
{
	rob.startMotors();
}

void loop()
{
	//rob.nav(rob.checkObstacles());
	rob.actions(rob.checkObstacles());
}
