#include "CommandExecuter.hpp"

CommandExecuter::CommandExecuter() {
	this->running = true;
	this->currentExecutionCounter = 0;
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

		while(currentExecutionCounter < this->executionCount){
			pthread_mutex_lock(&robotAccess);
			if(robot.isActivate()){
				//ROS_INFO("active execution %d", currentExecutionCounter);

				if(robot.isMove()){
					accelerate(robot.getMoveDirection(), robot.getSpeed());
				}
				if(robot.isTurn()){
					twistTo(robot.getTwistDirection());
				}
				if(robot.isGrasp()){
					grispTo(robot.getGraspDirection());
				}

				if(robot.isLook()){
					moveArmTo("", 0);
				}

				ros::Duration(this->defaultSleeptime_s).sleep();
				currentExecutionCounter++;
			}else{
				ROS_INFO("[Execution failed]: Robo deactivated!");
			}
			pthread_mutex_unlock(&robotAccess);
		}
	}
}

CommandExecuter::~CommandExecuter() {
}

void CommandExecuter::robo(const string& d) {
	pthread_mutex_lock(&robotAccess);
	wakeRobotAndResetCurrentExecutionCount();
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
	//pthread_mutex_lock(&robotAccess);
	wakeRobotAndResetCurrentExecutionCount();

	if (robot.isActivate()) {
		robot.setMove(true);
		if (d.compare("forward") == 0 || d.compare("backward") == 0) {
			robot.setMoveDirection(d);
			if(robot.getSpeed()==0){robot.setSpeed(robot.getAccelerateFactor());}

		} else if (d.compare("slow") == 0) {

			float newSpeed = robot.getSpeed()-robot.getAccelerateFactor();

			if(newSpeed < 0){
				newSpeed = 0;
			}
			robot.setSpeed(newSpeed);

		} else if (d.compare("fast") == 0) {

			float newSpeed = robot.getSpeed()+robot.getAccelerateFactor();

			if(newSpeed > robot.getMaxSpeed()){newSpeed = robot.getMaxSpeed();}
			robot.setSpeed(newSpeed);

		} else {
			robot.setMove(false);
			ROS_INFO("[Execution failed]: Command [%s] not found", d.c_str());
		}
	} else {
		ROS_INFO("[Execution failed]: Robo deactivated!");
	}
	//pthread_mutex_unlock(&robotAccess);

}

void CommandExecuter::grasp(const string& d) {
	pthread_mutex_lock(&robotAccess);
	wakeRobotAndResetCurrentExecutionCount();

	if (robot.isActivate()) {
		robot.setGrasp(true);
		if (d.compare("close") == 0) {
			robot.setGraspDirection(d);
		} else if (d.compare("open") == 0) {
			robot.setGraspDirection(d);
		} else {
			robot.setGrasp(false);
			ROS_INFO("[Execution failed]: Command [%s] not found", d.c_str());
		}
	} else {
		ROS_INFO("[Execution failed]: Robo deactivated!");
	}
	pthread_mutex_unlock(&robotAccess);
}

void CommandExecuter::look(const string& d) {
	pthread_mutex_lock(&robotAccess);
	wakeRobotAndResetCurrentExecutionCount();

	ROS_INFO("look");

	if (robot.isActivate()) {
		robot.setLook(true);
		if (d.compare("right") == 0) {
		} else if (d.compare("left") == 0) {

		} else if (d.compare("forward") == 0) {

		} else if (d.compare("backward") == 0) {

		} else {
			robot.setLook(false);
			ROS_INFO("[Execution failed]: Command [%s] not found", d.c_str());
		}
	} else {
		ROS_INFO("[Execution failed]: Robo deactivated!");
	}
	this->executionCount = 2;

	pthread_mutex_unlock(&robotAccess);
}

void CommandExecuter::turn(const string& d) {
	//pthread_mutex_lock(&robotAccess);
	int actualTwistFactor = robot.getDefaultTwistFactor();
	wakeRobotAndResetCurrentExecutionCount();
	if (robot.isActivate()) {
		int newTwistAngle = 0;
		float newTwistAngleRad = 0.0;
		float twistRad = 0.0;

		robot.setTurn(true);
		robot.setTwistDirection(d);

		if (d.compare("left") == 0) {
			newTwistAngle = robot.getTwistAngle() - robot.getDefaultTwistFactor();
			if(newTwistAngle < 0){
				newTwistAngle = 360 + newTwistAngle;
			}
		} else if (d.compare("right") == 0) {
			newTwistAngle = robot.getTwistAngle() + robot.getDefaultTwistFactor();
			newTwistAngle = newTwistAngle % 360;
		} else if(d.compare("backward") == 0) {
			newTwistAngle = robot.getTwistAngle() + 180;
			actualTwistFactor = 180;
			newTwistAngle = newTwistAngle % 360;
		} else {
			robot.setTurn(false);
			ROS_INFO("[Execution failed]: Command [%s] not found", d.c_str());
		}
		ROS_INFO("newTwistAngle %d",newTwistAngle);

		robot.setTwistAngle(newTwistAngle);
		ROS_INFO("newTwistAngle modulo 360 %d",newTwistAngle);

		twistRad = actualTwistFactor * M_PI  /180;
		// t = rad/twistSpeed
		//timeout_ms = (int)(twistRad / robot.getDefaultTwistSpeed())*1000;
		this->executionCount = twistRad / (robot.getDefaultTwistSpeed() * this->defaultSleeptime_s);
		ROS_INFO("executionCount: %d",this->executionCount );
		ROS_INFO("%d, %f, %f, %d",newTwistAngle, newTwistAngleRad, robot.getDefaultTwistSpeed(), timeout_ms);

	} else {
		ROS_INFO("[Execution failed]: Robo deactivated!");
	}
	//pthread_mutex_unlock(&robotAccess);
}

void CommandExecuter::accelerate(const string& dir, const float& ms) {
	if (dir.compare("forward") == 0) {
		// 0.1 m/s forward
		robot.setVelocityLinearX(ms);
	} else {
		// 0.1 m/s backward
		robot.setVelocityLinearX(-ms);
	}
	base_vel_pub->publish(robot.getVelocity());
}

void CommandExecuter::twistTo(const string& dir) {

	geometry_msgs::TwistPtr velocity_msg(new geometry_msgs::Twist);
	if(dir.compare("right") == 0 || dir.compare("backward") == 0){
		robot.setVelocityAngularZ(-robot.getDefaultTwistSpeed());
	}else if(dir.compare("left") == 0){
		robot.setVelocityAngularZ(robot.getDefaultTwistSpeed());
	}
	base_vel_pub->publish(robot.getVelocity());
}

void CommandExecuter::grispTo(const string& dir){
	float currentGraspValue = robot.getCurrentGraspValue();
	this->executionCount = 3;
	if(dir.compare("open") == 0){
		robot.setGraspValue(currentGraspValue + robot.getDefaultGripperStep());
	}else{
		robot.setGraspValue(currentGraspValue - robot.getDefaultGripperStep());
	}
	gripper_pub->publish(robot.getGripper());
}

void CommandExecuter::moveArmTo(const string& dir, const float& degree) {
	geometry_msgs::TwistPtr arm_msg (new geometry_msgs::Twist);

	arm_msg->angular.y = 1 * 0.87;
	arm_msg->angular.z = -1 * 0.43;
	arm_vel_pub->publish(arm_msg);
}

void CommandExecuter::wakeRobotAndResetCurrentExecutionCount(){
	pthread_mutex_lock(&timestampMTX);
	refreshActualTimestamp();
	pthread_cond_signal(&timestampCond);
	pthread_mutex_unlock(&timestampMTX);
	currentExecutionCounter = 0;

	this->executionCount = defaultExecutionCount;
}


void CommandExecuter::setConfigParameter(int timeout_ms, float defaultSleeptime_s, int defaultExecutionCount, float defaultRobotSpeed,
	float defaultAccelerateFactor, float MAX_SPEED, int defaultTwistFactor, float defaultTwistSpeed, float defaultGripperStep, Publisher* base_vel_pub, Publisher* gripper_pub, Publisher* arm_vel_pub) {
	this->timeout_ms = timeout_ms;
	this->defaultTimeout_ms = timeout_ms;
	this->defaultSleeptime_s = defaultSleeptime_s;
	this->executionCount = defaultExecutionCount;
	this->defaultExecutionCount = defaultExecutionCount;
	this->robot.setSpeed(defaultRobotSpeed);
	this->robot.setAccelerateFactor(defaultAccelerateFactor);
	this->robot.setMaxSpeed(MAX_SPEED);

	this->robot.setDefaultRobotSpeed(defaultRobotSpeed);
	this->robot.setDefaultAccelerateFactor(defaultAccelerateFactor);
	this->robot.setDefaultTwistFactor(defaultTwistFactor);
	this->robot.setDefaultTwistSpeed(defaultTwistSpeed);
	this->robot.setDefaultGripperStep(defaultGripperStep);

	this->base_vel_pub = base_vel_pub;
	this->gripper_pub = gripper_pub;
	this->arm_vel_pub = arm_vel_pub;
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
