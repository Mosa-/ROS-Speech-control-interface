#include <string>
using namespace std;

class ICommand
{

protected:
	ICommand(){}

public:
	virtual ~ICommand(){}
	
	virtual void robo(const string& d) = 0;
	virtual void move(const string& d) = 0;
	virtual void grasp(const string& d) = 0;
	virtual void look(const string& d) = 0;
	virtual void turn(const string& d) = 0;
};
