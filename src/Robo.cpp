#include "Robo.hpp"

Robo::Robo() : velocity_msg(new geometry_msgs::Twist), MAX_SPEED(0.0), defaultRobotSpeed(0.0),
defaultAccelerateFactor(0.0), defaultTwistFactor(0.0), activated(false), move(false), grasp(false),
look(false), turn(false), speed(0.0), accelerateFactor(0.0),twistAngle(0),defaultTwistSpeed(0.0),
twistDirection("right"),seeAngle(0.0),moveDirection("forward"){

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
	this->resetVelocity();
}
void Robo::resetRoboStates(){
	this->stopMoving();
	this->stopTurning();
	this->resetVelocity();

}
