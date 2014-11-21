#include <string>
#include <geometry_msgs/Twist.h>
#include <metralabs_msgs/IDAndFloat.h>


#ifndef ROBO_H_
#define ROBO_H_

using namespace std;

class Robo {
public:
	Robo();
	virtual ~Robo();

	void resetRobo();
	void resetRoboStates();
	void stopMoving(){
			this->move = false;
		}
	void stopTurning(){
		this->turn = false;
	}
	void stopGrasp(){
		this->grasp = false;
	}
	void activate(){
		this->activated = true;
	}
	void disable(){
		this->activated = false;
	}
	void resetVelocity(){
		this->setVelocityAngular(0.0,0.0,0.0);
		this->setVelocityLinear(0.0,0.0,0.0);
	}

	geometry_msgs::TwistPtr getVelocity(){
		return this->velocity_msg;
	}
	metralabs_msgs::IDAndFloatPtr getGripper(){
		return this->gripper_msg;
	}

	void setVelocityAngular(float x, float y, float z){
		this->velocity_msg->angular.x = x;
		this->velocity_msg->angular.y = y;
		this->velocity_msg->angular.z = z;
	}
	void setVelocityAngularX(float x){
		this->velocity_msg->angular.x = x;
	}
	void setVelocityAngularY(float y){
		this->velocity_msg->angular.y = y;
	}
	void setVelocityAngularZ(float z){
		this->velocity_msg->angular.z = z;
	}
	void setVelocityLinear(float x, float y, float z) {
		this->velocity_msg->linear.x = x;
		this->velocity_msg->linear.y = y;
		this->velocity_msg->linear.z = z;
	}
	void setVelocityLinearX(float x) {
		this->velocity_msg->linear.x = x;
	}
	void setVelocityLinearY(float y) {
		this->velocity_msg->linear.y = y;
	}
	void setVelocityLinearZ(float z) {
		this->velocity_msg->linear.z = z;
	}


	bool isActivate() const {
		return activated;
	}

	void setActivate(bool activate) {
		this->activated = activate;
	}

	bool isGrasp() const {
		return grasp;
	}

	void setGrasp(bool grasp) {
		this->grasp = grasp;
	}

	bool isLook() const {
		return look;
	}

	void setLook(bool look) {
		this->look = look;
	}

	bool isMove() const {
		return move;
	}

	void setMove(bool move) {
		this->move = move;
	}

	float getSeeAngle() const {
		return seeAngle;
	}

	void setSeeAngle(float seeAngle) {
		this->seeAngle = seeAngle;
	}

	float getSpeed() const {
		return speed;
	}

	void setSpeed(float speed) {
		this->speed = speed;
	}

	bool isTurn() const {
		return turn;
	}

	void setTurn(bool turn) {
		this->turn = turn;
	}

	const string& getMoveDirection() const {
		return moveDirection;
	}

	void setMoveDirection(const string& moveDirection) {
		this->moveDirection = moveDirection;
	}

	float getAccelerateFactor() const {
		return accelerateFactor;
	}

	void setAccelerateFactor(float accelerateFactor) {
		this->accelerateFactor = accelerateFactor;
	}

	float getMaxSpeed() const {
		return MAX_SPEED;
	}

	void setMaxSpeed(float maxSpeed) {
		MAX_SPEED = maxSpeed;
	}

	float getDefaultAccelerateFactor() const {
		return defaultAccelerateFactor;
	}

	void setDefaultAccelerateFactor(float defaultAccelerateFactor) {
		this->defaultAccelerateFactor = defaultAccelerateFactor;
	}

	float getDefaultRobotSpeed() const {
		return defaultRobotSpeed;
	}

	void setDefaultRobotSpeed(float defaultRobotSpeed) {
		this->defaultRobotSpeed = defaultRobotSpeed;
	}

	int getDefaultTwistFactor() const {
		return defaultTwistFactor;
	}

	void setDefaultTwistFactor(int defaultTwistFactor) {
		this->defaultTwistFactor = defaultTwistFactor;
	}

	int getTwistAngle() const {
		return twistAngle;
	}

	void setTwistAngle(int twistAngle) {
		this->twistAngle = twistAngle;
	}

	const string& getTwistDirection() const {
		return twistDirection;
	}

	void setTwistDirection(const string& twistDirection) {
		this->twistDirection = twistDirection;
	}

	float getDefaultTwistSpeed() const {
		return defaultTwistSpeed;
	}

	void setDefaultTwistSpeed(float defaultTwistSpeed) {
		this->defaultTwistSpeed = defaultTwistSpeed;
	}

	const string& getGraspDirection() const {
		return graspDirection;
	}

	void setGraspDirection(const string& graspDirection) {
		this->graspDirection = graspDirection;
	}

	void setGraspValue(float graspValue){
		this->gripper_msg->id = 5;
		this->gripper_msg->value = graspValue;
	}

private:

	geometry_msgs::TwistPtr velocity_msg;
	metralabs_msgs::IDAndFloatPtr gripper_msg;

	float MAX_SPEED;
	float defaultRobotSpeed;
	float defaultAccelerateFactor;
	int defaultTwistFactor;

	bool activated;
	bool move;
	bool grasp;
	bool look;
	bool turn;

	float speed;
	float accelerateFactor;
	int twistAngle; // in °
	float defaultTwistSpeed; // rad/s
	string twistDirection;

	float seeAngle;
	string moveDirection;

	string graspDirection;
};

#endif /* ROBO_H_ */
