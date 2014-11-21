#include <string>
#include <stdlib.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <pthread.h>
#include "CommandExecuter.hpp"

using namespace std_msgs;
using namespace std;

ros::Publisher pubCmd;
vector<string> compoundCmd;

// data/cmd word -> Unique data/cmd
map<string, string> cmdOrDataToUniqueWord;
//Unique command/data -> cmd/data
map<string, string> wordClassifier;
CommandExecuter cmdExecuter;



// check the order of the command with data and execute it
void executeCmd(const vector<string>& cmd) {
	bool valid = true;
	vector<string> rightOrderCmd;
	string word1;
	string word2;
	word1 = wordClassifier.find(cmd.at(0))->second;
	word2 = wordClassifier.find(cmd.at(1))->second;

	if (word1.compare("cmd") == 0) {
		// invalid command, both words are command-words
		if (word2.compare("cmd") == 0) {
			valid = false;
		} else {
			rightOrderCmd.push_back(cmd.at(0));
			rightOrderCmd.push_back(cmd.at(1));
		}
	} else {
		// invalid command, both words are data-words
		if (word2.compare("data") == 0) {
			valid = false;
		} else {
			rightOrderCmd.push_back(cmd.at(1));
			rightOrderCmd.push_back(cmd.at(0));
		}
	}

	if (valid) {
		// use command interface for execute the command
		string cmd = rightOrderCmd.at(0);
		string data = rightOrderCmd.at(1);

		if (cmd.compare("robo") == 0) {
			cmdExecuter.robo(data.c_str());
		} else if (cmd.compare("move") == 0) {
			cmdExecuter.move(data.c_str());
		} else if (cmd.compare("grasp") == 0) {
			cmdExecuter.grasp(data.c_str());
		} else if (cmd.compare("look") == 0) {
			cmdExecuter.look(data.c_str());
		} else if (cmd.compare("turn") == 0) {
			cmdExecuter.turn(data.c_str());
		} else {
			valid = false;
		}

		if (valid) {
			ROS_INFO("Execute command: [%s, %s]", rightOrderCmd.at(0).c_str(),
					rightOrderCmd.at(1).c_str());
		} else {
			ROS_INFO("Execute command failed: [%s, %s]",
					rightOrderCmd.at(0).c_str(), rightOrderCmd.at(1).c_str());
		}

	} else {
		ROS_INFO("Invalid command: [%s, %s]", word1.c_str(), word2.c_str());
	}

}

string checkWord(const string& word) {
	string wordRtrn = "NONE";

	if (cmdOrDataToUniqueWord.count(word) == 1) {
		wordRtrn = cmdOrDataToUniqueWord.find(word)->second;
	}

	return wordRtrn;
}

void createStopCommand(){
	compoundCmd.clear();
	compoundCmd.push_back("robo");
	compoundCmd.push_back("stop");
	ROS_INFO("<<<Robo stop>>>");
}

// composition of a command: <command> [<data>]
void cmdParse(const std_msgs::String& cmd) {
	String cmdStr = cmd;

	string cmds = cmdStr.data;
	vector<string> tokens;
	stringstream mySstream(cmds);
	string temp;
	while (getline(mySstream, temp, ' ')) {
		tokens.push_back(temp);
	}

	if (tokens.size() > 2) {
		ROS_INFO("I heard more than 2 words: [%s]", cmdStr.data.c_str());
	} else if (tokens.size() == 2) {
		ROS_INFO("I heard 2 words: [%s]", cmdStr.data.c_str());
	} else {
		ROS_INFO("I heard 1 word: [%s]", cmdStr.data.c_str());
	}

	// TODO:
	//create interface for commands (methods) -> "HAL"
	//map like DB which maps speech cmds to interface cmd

	int cnt = 0;
	while (tokens.size() > cnt) {
		if (compoundCmd.size() > 2) {
			ROS_INFO("Error: Too many command pieces. Execute only the first two words of the command text.");
			break;
		}

		// check if word is in the command map
		string cmdValid = checkWord(tokens.at(cnt).c_str());

		if(cmdValid.compare("stop") == 0 || cmdValid.compare("off") == 0){
			createStopCommand();
		}else if (cmdValid.compare("NONE") != 0) {
			// put piece of the command (c_str() without whitespace) in the vector
			compoundCmd.push_back(cmdValid.c_str());
			ROS_INFO("Put [%s] as a piece of a command.\n Total words: [%d]",
					tokens.at(cnt).c_str(), compoundCmd.size());

		}
		cnt++;
	}

	if (compoundCmd.size() > 1) {
		ROS_INFO("Execute: [%s][%s]", compoundCmd.at(0).c_str(),
				compoundCmd.at(1).c_str());
		executeCmd(compoundCmd);
		// publish cmd to visualizer
		stringstream ss;
		ss << compoundCmd.at(0) << " " << compoundCmd.at(1);
		String publishMsg;
		publishMsg.data = ss.str();
		pubCmd.publish(publishMsg);
		compoundCmd.clear();
	}



  // if new word is first piece of cmd
  // then
	// check if word is in the map
		// yes -> put in the vector
		// no -> do nothing
  // if new word is second piece of cmd
  // then
    // check if word is in the map
		// yes ->
			// check if first or second entry in vector is the <command>
			// knowing the <command> -> knowing the method to use from the interface <- mapping
			// <data> mapping for parameter for the interface
			// execute method with <command> <data>
		// no -> do nothing

}


// better use a database
void initLookUpContainer(){

	string cmd = "cmd";
	string data = "data";

	// if cmd its already unique, no need for mapping
	wordClassifier.insert(pair<string,string> (string("robo"),cmd));
	wordClassifier.insert(pair<string,string> (string("move"),cmd));
	wordClassifier.insert(pair<string,string> (string("grasp"),cmd));
	wordClassifier.insert(pair<string,string> (string("look"),cmd));
	wordClassifier.insert(pair<string,string> (string("turn"),cmd));
	wordClassifier.insert(pair<string,string> (string("twist"),cmd));
	wordClassifier.insert(pair<string,string> (string("spin"),cmd));


	wordClassifier.insert(pair<string,string> (string("forward"),data));
	wordClassifier.insert(pair<string,string> (string("backward"),data));
	wordClassifier.insert(pair<string,string> (string("slow"),data));
	wordClassifier.insert(pair<string,string> (string("fast"),data));
	wordClassifier.insert(pair<string,string> (string("close"),data));
	wordClassifier.insert(pair<string,string> (string("open"),data));
	wordClassifier.insert(pair<string,string> (string("right"),data));
	wordClassifier.insert(pair<string,string> (string("left"),data));
	wordClassifier.insert(pair<string,string> (string("ahead"),data));
	wordClassifier.insert(pair<string,string> (string("back"),data));
	wordClassifier.insert(pair<string,string> (string("start"),data));
	wordClassifier.insert(pair<string,string> (string("run"),data));
	wordClassifier.insert(pair<string,string> (string("begin"),data));
	wordClassifier.insert(pair<string,string> (string("stop"),data));
	wordClassifier.insert(pair<string,string> (string("off"),data));

	cmdOrDataToUniqueWord.insert(pair<string,string>(string("robo"),string("robo")));
	cmdOrDataToUniqueWord.insert(pair<string,string>(string("move"),string("move")));
	cmdOrDataToUniqueWord.insert(pair<string,string>(string("grasp"),string("grasp")));
	cmdOrDataToUniqueWord.insert(pair<string,string>(string("grip"),string("grasp")));
	cmdOrDataToUniqueWord.insert(pair<string,string>(string("look"),string("look")));
	cmdOrDataToUniqueWord.insert(pair<string,string>(string("turn"),string("turn")));
	cmdOrDataToUniqueWord.insert(pair<string,string>(string("twist"),string("turn")));
	cmdOrDataToUniqueWord.insert(pair<string,string>(string("spin"),string("turn")));

	cmdOrDataToUniqueWord.insert(pair<string,string>(string("forward"),string("forward")));
	cmdOrDataToUniqueWord.insert(pair<string,string>(string("straight"),string("forward")));
	cmdOrDataToUniqueWord.insert(pair<string,string>(string("backward"),string("backward")));
	cmdOrDataToUniqueWord.insert(pair<string,string>(string("slow"),string("slow")));
	cmdOrDataToUniqueWord.insert(pair<string,string>(string("slower"),string("slow")));
	cmdOrDataToUniqueWord.insert(pair<string,string>(string("fast"),string("fast")));
	cmdOrDataToUniqueWord.insert(pair<string,string>(string("faster"),string("fast")));
	cmdOrDataToUniqueWord.insert(pair<string,string>(string("close"),string("close")));
	cmdOrDataToUniqueWord.insert(pair<string,string>(string("open"),string("open")));
	cmdOrDataToUniqueWord.insert(pair<string,string>(string("right"),string("right")));
	cmdOrDataToUniqueWord.insert(pair<string,string>(string("left"),string("left")));
	cmdOrDataToUniqueWord.insert(pair<string,string>(string("ahead"),string("forward")));
	cmdOrDataToUniqueWord.insert(pair<string,string>(string("back"),string("backward")));
	cmdOrDataToUniqueWord.insert(pair<string,string>(string("start"),string("start")));
	cmdOrDataToUniqueWord.insert(pair<string,string>(string("run"),string("start")));
	cmdOrDataToUniqueWord.insert(pair<string,string>(string("begin"),string("start")));
	cmdOrDataToUniqueWord.insert(pair<string,string>(string("go"),string("start")));
	cmdOrDataToUniqueWord.insert(pair<string,string>(string("halt"),string("stop")));
	cmdOrDataToUniqueWord.insert(pair<string,string>(string("abort"),string("stop")));
	cmdOrDataToUniqueWord.insert(pair<string,string>(string("stop"),string("stop")));
	cmdOrDataToUniqueWord.insert(pair<string,string>(string("exit"),string("off")));
	cmdOrDataToUniqueWord.insert(pair<string,string>(string("off"),string("off")));
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "speech_control_interface");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
	ros::NodeHandle n;
	Publisher base_cmd_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel_in/sci", 3);
	Publisher arm_vel_pub = n.advertise<geometry_msgs::Twist>("moveArmVelocity", 3);
	Publisher gripper_pub = n.advertise<metralabs_msgs::IDAndFloat>("/schunk/move_position", 1);

	int timeout_ms = 2000;
	// defaultSleepTime_s * defaultExecutionCount = execution duration [s]
	float defaultSleepTime_s = 0.3;
	int defaultExecutionCount = 5;
	float defaultRobotSpeed = 0.05; // m/s
	float defaultAccelerateFactor = 0.05; // m/s
	float MAX_SPEED = 0.35;
	float defaultTwistFactor = 30; // °
	float defaultTwistSpeed = 0.30; // rad/s

	cmdExecuter.setConfigParameter(timeout_ms, defaultSleepTime_s, defaultExecutionCount, defaultRobotSpeed, defaultAccelerateFactor,
			MAX_SPEED, defaultTwistFactor, defaultTwistSpeed, &base_cmd_vel, &gripper_pub, &arm_vel_pub);
	pthread_t cmdExecuterThread;
	pthread_create(&cmdExecuterThread, NULL, &CommandExecuter::run_helper, &cmdExecuter);

  // Subscribe output of the speech recognizer [store 2 messages]
  ros::Subscriber sub = n.subscribe("/recognizer/output", 2, cmdParse);
  cout << "test" << endl;

  // Set publisher for publishing the resulting commands [store 10 messages]
  pubCmd = n.advertise<std_msgs::String>("commandParser/cmd", 10);
  cout << "test2" << endl;

  // Need for arriving speech commands
  initLookUpContainer();
  cout << "test3" << endl;


  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
