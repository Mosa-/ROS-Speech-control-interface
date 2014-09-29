#include "CommandExecuter.hpp"

CommandExecuter::CommandExecuter() {
	this->running = true;
}


void* CommandExecuter::run(){

	//hier die Befehle umsetzen, in dem Attribute von Robo gelesen werden und
	// die Umsetzung angepasst werden, es gibt Timestamp der aktualisiert wird wie lange der
	// thread laufen soll -> ablauf timer thraed condition variable schlafen, bis neuer befehl aufwachen
	// -timestamp für befehle gibt standard


	refreshActualTimestamp();

	while(this->running){

		pthread_mutex_lock(&timestampMTX);
		if(isTimeout()){
			ROS_INFO("TIMEOUT");
			robot.resetRoboStates();
			pthread_cond_wait(&timestampCond, &timestampMTX);
		}
		pthread_mutex_unlock(&timestampMTX);

		pthread_mutex_lock(&robotAccess);
		if(robot.isActivate()){
			if(robot.isMove()){
				accelerate(robot.getMoveDirection(), robot.getSpeed());

				ros::Duration(0.1).sleep();
			}

			if(robot.isTurn()){
				ROS_INFO("turn");

				twistTo(robot.getTwistDirection());
				ros::Duration(0.1).sleep();

			}

		}else{
			ROS_INFO("[Execution failed]: Robo deactivated!");
		}
		pthread_mutex_unlock(&robotAccess);

	}
}

CommandExecuter::~CommandExecuter() {
}

void CommandExecuter::robo(const string& d) {
	pthread_mutex_lock(&robotAccess);
	wakeRobot();
	if (d.compare("start") == 0) {
		robot.activate();
		ROS_INFO("[Execution]: Robo started!");
	} else if (d.compare("stop") == 0 || d.compare("off") == 0) {
		accelerate(robot.getMoveDirection()	, 0);
		robot.resetRobo();
		ROS_INFO("[Execution]: Robo stopped!");
	} else {
		ROS_INFO("[Execution failed]: Command [%s] not found", d.c_str());
	}

	pthread_mutex_unlock(&robotAccess);

}

void CommandExecuter::move(const string& d) {
	pthread_mutex_lock(&robotAccess);
	wakeRobot();

	if (robot.isActivate()) {
		if (d.compare("forward") == 0 || d.compare("backward") == 0) {
			robot.setMoveDirection(d);
			robot.setMove(true);
			if(robot.getSpeed()==0){robot.setSpeed(robot.getAccelerateFactor());}

		} else if (d.compare("slow") == 0) {

			float newSpeed = robot.getSpeed()-robot.getAccelerateFactor();
			robot.setMove(true);

			if(newSpeed < 0){
				newSpeed = 0;
				robot.setMove(false);
			}
			robot.setSpeed(newSpeed);

		} else if (d.compare("fast") == 0) {

			float newSpeed = robot.getSpeed()+robot.getAccelerateFactor();
			robot.setMove(true);

			if(newSpeed > robot.getMaxSpeed()){newSpeed = robot.getMaxSpeed();}
			robot.setSpeed(newSpeed);

		} else {
			ROS_INFO("[Execution failed]: Command [%s] not found", d.c_str());
		}
	} else {
		ROS_INFO("[Execution failed]: Robo deactivated!");
	}
	pthread_mutex_unlock(&robotAccess);

}

void CommandExecuter::grasp(const string& d) {
	pthread_mutex_lock(&robotAccess);
	wakeRobot();

	if (robot.isActivate()) {
		if (d.compare("close") == 0) {
		} else if (d.compare("open") == 0) {

		} else {
			ROS_INFO("[Execution failed]: Command [%s] not found", d.c_str());
		}
	} else {
		ROS_INFO("[Execution failed]: Robo deactivated!");
	}
	pthread_mutex_unlock(&robotAccess);
}

void CommandExecuter::look(const string& d) {
	pthread_mutex_lock(&robotAccess);
	wakeRobot();

	if (robot.isActivate()) {
		if (d.compare("right") == 0) {
		} else if (d.compare("left") == 0) {

		} else if (d.compare("forward") == 0) {

		} else if (d.compare("backward") == 0) {

		} else {
			ROS_INFO("[Execution failed]: Command [%s] not found", d.c_str());
		}
	} else {
		ROS_INFO("[Execution failed]: Robo deactivated!");
	}
	pthread_mutex_unlock(&robotAccess);
}

void CommandExecuter::turn(const string& d) {
	pthread_mutex_lock(&robotAccess);
	wakeRobot();
	if (robot.isActivate()) {
		int newTwistAngle = 0;
		float newTwistAngleRad = 0.0;

		robot.setTurn(true);
		robot.setTwistDirection(d);

		if (d.compare("left") == 0) {
			newTwistAngle = robot.getTwistAngle() + robot.getDefaultTwistFactor();
		} else if (d.compare("right") == 0) {
			newTwistAngle = robot.getTwistAngle() - robot.getDefaultTwistFactor();
		} else {
			ROS_INFO("[Execution failed]: Command [%s] not found", d.c_str());
		}
		ROS_INFO("newTwistAngle %d",newTwistAngle);

		newTwistAngle = newTwistAngle % 360;
		robot.setTwistAngle(newTwistAngle);
		ROS_INFO("newTwistAngle modulo 360 %d",newTwistAngle);

		newTwistAngleRad = newTwistAngle * M_PI  /180;
		// t = rad/twistSpeed
		timeout_ms = (int)(newTwistAngleRad / robot.getDefaultTwistSpeed())*1000;
		ROS_INFO("%d, %f, %f, %d",newTwistAngle, newTwistAngleRad, robot.getDefaultTwistSpeed(), timeout_ms);

	} else {
		ROS_INFO("[Execution failed]: Robo deactivated!");
	}
	pthread_mutex_unlock(&robotAccess);
}

void CommandExecuter::accelerate(const string& dir, const float& ms) {
	geometry_msgs::TwistPtr velocity_msg(new geometry_msgs::Twist);

	if (dir.compare("forward") == 0) {
		// 0.1 m/s forward
		velocity_msg->linear.x = ms;
	} else {
		// 0.1 m/s backward
		velocity_msg->linear.x = -ms;
	}
	base_vel_pub->publish(velocity_msg);
}

void CommandExecuter::twistTo(const string& dir) {

	geometry_msgs::TwistPtr velocity_msg(new geometry_msgs::Twist);
	if(dir.compare("right") == 0){
		velocity_msg->angular.z = -robot.getDefaultTwistSpeed();
	}else if(dir.compare("left") == 0){
		velocity_msg->angular.z = robot.getDefaultTwistSpeed();
	}
	base_vel_pub->publish(velocity_msg);
}

void CommandExecuter::moveArmTo(const string& dir, const float& degree) {
}

void CommandExecuter::wakeRobot(){
	pthread_mutex_lock(&timestampMTX);
	refreshActualTimestamp();
	pthread_cond_signal(&timestampCond);
	pthread_mutex_unlock(&timestampMTX);

}


void CommandExecuter::setConfigParameter(int timeout_ms, float defaultRobotSpeed,
	float defaultAccelerateFactor, float MAX_SPEED, int defaultTwistFactor, float defaultTwistSpeed, Publisher* publisher) {
	this->timeout_ms = timeout_ms;
	this->defaultTimeout_ms = timeout_ms;
	this->robot.setSpeed(defaultRobotSpeed);
	this->robot.setAccelerateFactor(defaultAccelerateFactor);
	this->robot.setMaxSpeed(MAX_SPEED);

	this->robot.setDefaultRobotSpeed(defaultRobotSpeed);
	this->robot.setDefaultAccelerateFactor(defaultAccelerateFactor);
	this->robot.setDefaultTwistFactor(defaultTwistFactor);
	this->robot.setDefaultTwistSpeed(defaultTwistSpeed);

	this->base_vel_pub = publisher;
}

void CommandExecuter::refreshActualTimestamp(){
	this->timeout_ms = defaultTimeout_ms;
	this->actualTimestamp = getCurrentTimeIn_ms();
}

long CommandExecuter::getCurrentTimeIn_ms(){
	timeval currentTime;
	struct tm *tm;
	gettimeofday(&currentTime, NULL);
	tm = localtime(&currentTime.tv_sec);
	return (long) (tm->tm_hour*3600*1000+tm->tm_min*60*1000+tm->tm_sec*1000+currentTime.tv_usec/1000);
}

bool CommandExecuter::isTimeout(){
	return getCurrentTimeIn_ms() > actualTimestamp + timeout_ms;
}
