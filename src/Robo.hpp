#include <string>

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
	void activate(){
		this->activated = true;
	}
	void disable(){
		this->activated = false;
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

private:

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
};

#endif /* ROBO_H_ */
