#include <aris.h>
//#include <Aris_Control.h>
#include <Robot_Gait.h>
#include <Robot_Type_I.h>
#include <string>
#include <map>


enum ROBOT_CMD_ID
{
	ENABLE,
	DISABLE,
	HOME,
	RESET_ORIGIN,
	RUN_GAIT,

	ROBOT_CMD_COUNT
};


extern Robots::RobotTypeI robot;


void DecodeMsg(const Aris::Core::MSG &msg, std::string &cmd, std::map<std::string,std::string> &params);
void GenerateCmdMsg(const std::string &cmd, const std::map<std::string,std::string> &params, Aris::Core::MSG &msg);
int tg(Aris::RT_CONTROL::CMachineData &,Aris::Core::RT_MSG &,Aris::Core::RT_MSG &);
