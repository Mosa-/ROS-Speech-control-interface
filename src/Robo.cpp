#include "Robo.hpp"

Robo::Robo() {
	this->twistAngle = 0;
}

Robo::~Robo() {
	// TODO Auto-generated destructor stub
}

void Robo::resetRobo() {
	this->setSpeed(this->defaultRobotSpeed);
	this->setAccelerateFactor(this->defaultAccelerateFactor);
	this->disable();
	this->stopMoving();
	this->stopTurning();
}
void Robo::resetRoboStates(){
	this->stopMoving();
	this->stopTurning();
}
