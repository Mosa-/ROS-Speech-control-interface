#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <time.h>
#include <math.h>

#include "ICommand.hpp"
#include "Robo.hpp"

using namespace std;
using namespace ros;

class CommandExecuter: public ICommand
{

private:
	Robo robot;

	bool running;
	int defaultTimeout_ms;
	int timeout_ms;
	long actualTimestamp;
	Publisher* base_vel_pub;

	pthread_mutex_t timestampMTX;
	pthread_cond_t timestampCond;

	pthread_mutex_t robotAccess;

	void accelerate(const string& dir, const float& ms);
	void twistTo(const string& dir);
	void moveArmTo(const string& dir, const float& degree);

	void wakeRobot();

	void refreshActualTimestamp();
	long getCurrentTimeIn_ms();
	bool isTimeout();

public:
	CommandExecuter();
	~CommandExecuter();
	void* run();
	static void *run_helper(void *context){
		return ((CommandExecuter*)context)->run();
	}

	virtual void robo(const string& d);
	virtual void move(const string& d);
	virtual void grasp(const string& d);
	virtual void look(const string& d);
	virtual void turn(const string& d);
	void setConfigParameter(int timeout_ms, float defaultRobotSpeed, float defaultAccelerateFactor,
			float MAX_SPEED, int defaultTwistFactor, float defaultTwistSpeed, Publisher* publisher);

};
