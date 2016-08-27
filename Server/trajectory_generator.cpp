#include "trajectory_generator.h"
#include <cstring>
#include <Aris_Plan.h>
#include <Robot_Base.h>


using namespace std;

extern Aris::RT_CONTROL::ACTUATION cs;

void DecodeMsg(const Aris::Core::MSG &msg, std::string &cmd, std::map<std::string,std::string> &params)
{
	char content[500];

	std::int32_t size=0;
	std::int32_t beginPos=0;

	msg.PasteStruct(size);
	beginPos+=4;
	msg.PasteAt(content,size,beginPos);
	cmd.assign(content);
	beginPos+=size;

	std::int32_t paramNum;
	msg.PasteAt(&paramNum,4,beginPos);
	beginPos+=4;

	for(int i=0;i<paramNum;++i)
	{
		std::string cmdd,param;

		msg.PasteAt(&size,4,beginPos);
		beginPos+=4;
		msg.PasteAt(content,size,beginPos);
		cmdd.assign(content);
		beginPos+=size;

		msg.PasteAt(&size,4,beginPos);
		beginPos+=4;
		msg.PasteAt(content,size,beginPos);
		param.assign(content);
		beginPos+=size;

		params.insert(std::make_pair(cmdd,param));
	}

}
void GenerateCmdMsg(const std::string &cmd, const std::map<std::string,std::string> &params, Aris::Core::MSG &msg)
{
	if(cmd=="en")
	{
		Robots::GAIT_PARAM_BASE robotState;
		robotState.cmdType=ENABLE;

		for(auto &i:params)
		{
			if(i.first=="all")
			{
				int motors[18]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17};
				std::memcpy(robotState.motorID,motors,sizeof(motors));
				robotState.motorNum=18;
			}
			else if(i.first=="left")
			{
				robotState.motorNum=9;
				int motors[9]={0,1,2,6,7,8,12,13,14};
				std::memcpy(robotState.motorID,motors,sizeof(motors));




			}
			else if(i.first=="right")
			{
				robotState.motorNum=9;
				int motors[9]={3,4,5,9,10,11,15,16,17};
				std::memcpy(robotState.motorID,motors,sizeof(motors));
			}
			else if(i.first=="motor")
			{
				int motors[1]={stoi(i.second)};
				std::memcpy(robotState.motorID,motors,sizeof(motors));
				robotState.motorNum=1;
			}
		}

		msg.CopyStruct(robotState);
		return;
	}

	if(cmd=="ds")
	{
		Robots::GAIT_PARAM_BASE robotState;
		robotState.cmdType=DISABLE;

		for(auto &i:params)
		{
			if(i.first=="all")
			{
				int motors[18]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17};
				std::memcpy(robotState.motorID,motors,sizeof(motors));
				robotState.motorNum=18;
			}
			else if(i.first=="left")
			{
				int motors[9]={0,1,2,6,7,8,12,13,14};
				std::memcpy(robotState.motorID,motors,sizeof(motors));
				robotState.motorNum=9;
			}
			else if(i.first=="right")
			{
				int motors[9]={3,4,5,9,10,11,15,16,17};
				std::memcpy(robotState.motorID,motors,sizeof(motors));
				robotState.motorNum=9;
			}
			else if(i.first=="motor")
			{
				int motors[1]={stoi(i.second)};
				std::memcpy(robotState.motorID,motors,sizeof(motors));
				robotState.motorNum=1;
			}
		}

		msg.CopyStruct(robotState);
		return;
	}

	if(cmd=="hm")
	{
		Robots::GAIT_PARAM_BASE robotState;
		robotState.cmdType=HOME;

		for(auto &i:params)
		{
			if(i.first=="all")
			{
				int motors[18]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17};
				std::memcpy(robotState.motorID,motors,sizeof(motors));
				robotState.motorNum=18;

				robotState.legNum=6;
				int legs[6]={0,1,2,3,4,5};
				std::memcpy(robotState.legID,legs,sizeof(legs));
			}
			else if(i.first=="left")
			{
				int motors[9]={0,1,2,6,7,8,12,13,14};
				std::memcpy(robotState.motorID,motors,sizeof(motors));
				robotState.motorNum=9;

				robotState.legNum=3;
				int legs[3]={0,2,4};
				std::memcpy(robotState.legID,legs,sizeof(legs));
			}
			else if(i.first=="right")
			{
				int motors[9]={3,4,5,9,10,11,15,16,17};
				std::memcpy(robotState.motorID,motors,sizeof(motors));
				robotState.motorNum=9;

				robotState.legNum=3;
				int legs[3]={1,3,5};
				std::memcpy(robotState.legID,legs,sizeof(legs));
			}
			else if(i.first=="motor")
			{
				int motors[1]={stoi(i.second)};
				std::memcpy(robotState.motorID,motors,sizeof(motors));
				robotState.motorNum=1;

				robotState.legNum=0;
			}
		}

		msg.CopyStruct(robotState);
		return;
	}

	if(cmd=="ro")
	{
		Robots::GAIT_PARAM_BASE robotState;
		robotState.cmdType=RESET_ORIGIN;
		msg.CopyStruct(robotState);
		return;
	}


	if(cmd=="ad")
	{
		Robots::ADJUST_PARAM  param;
		param.cmdType=RUN_GAIT;
		param.cmdID=0;

		param.totalCount=1000;

		param.legNum=0;
		for(auto &i:params)
		{
			if(i.first=="left")
			{
				param.legNum=3;
				param.legID[0]=0;
				param.legID[1]=2;
				param.legID[2]=4;

				param.motorNum=9;
				int id[9]={0,1,2,6,7,8,12,13,14};
				std::memcpy(param.motorID,id,sizeof(id));
			}

			if(i.first=="right")
			{
				param.legNum=3;
				param.legID[0]=1;
				param.legID[1]=3;
				param.legID[2]=5;

				param.motorNum=9;
				int id[9]={3,4,5,9,10,11,15,16,17};
				std::memcpy(param.motorID,id,sizeof(id));
			}
		}



		double middlePee[18]
		{
			-0.3,-0.75,-0.65,
			-0.45,-0.75,0,
			-0.3,-0.75,0.65,
			0.3,-0.75,-0.65,
			0.45,-0.75,0,
			0.3,-0.75,0.65,
		};
		double targetPee[18]
		{
			-0.3,-0.85,-0.65,
			-0.45,-0.85,0,
			-0.3,-0.85,0.65,
			0.3,-0.85,-0.65,
			0.45,-0.85,0,
			0.3,-0.85,0.65,
		};

		std::memcpy(param.targetPee,middlePee,sizeof(middlePee));
		std::memset(param.targetBodyPE,0,sizeof(param.targetBodyPE));


		msg.CopyStruct(param);
		cs.NRT_PostMsg(msg);

		std::memcpy(param.targetPee,targetPee,sizeof(targetPee));
		msg.CopyStruct(param);
		return;
	}


	if(cmd=="wk")
	{
		Robots::WALK_PARAM  param;
		param.cmdType=RUN_GAIT;
		param.cmdID=1;

		param.motorNum=18;
		int id[18]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17};
		std::memcpy(param.motorID,id,sizeof(id));

		param.legNum=6;
		int legid[6]={0,1,2,3,4,5};
		std::memcpy(param.legID,legid,sizeof(legid));


		for(auto &i:params)
		{
			if(i.first=="totalCount")
			{
				param.totalCount=std::stoi(i.second);
			}
			else if(i.first=="n")
			{
				param.n=stoi(i.second);
			}
			else if(i.first=="walkDirection")
			{
				param.walkDirection=stoi(i.second);
			}
			else if(i.first=="upDirection")
			{
				param.upDirection=stoi(i.second);
			}
			else if(i.first=="distance")
			{
				param.d=stod(i.second);
			}
			else if(i.first=="height")
			{
				param.h=stod(i.second);
			}
			else if(i.first=="alpha")
			{
				param.alpha=stod(i.second);
			}
			else if(i.first=="beta")
			{
				param.beta=stod(i.second);
			}
		}



		msg.CopyStruct(param);
		return;
	}
}
/*
int HEXBOT_HOME_OFFSETS_RESOLVER[18]=
{
		-15849882+349000,-16354509+349000,-16354509+349000,
		-15849882+349000,-16354509+349000,-16354509+349000,
		-15849882+349000,-16354509+349000,-16354509+349000,
		-16354509+349000,-15849882+349000,-16354509+349000,
		-15849882+349000,-16354509+349000,-16354509+349000,
		-16354509+349000,-16354509+349000,-15849882+349000,
};*/
const double meter2count = 1/0.01*3.5*65536;

const int MapAbsToPhy[18]
{
	10,11,9,
	12,14,13,
	17,15,16,
	6,8,7,
	3,5,4,
	0,2,1
};

const int MapPhyToAbs[18]
{
	15,17,16,
	12,14,13,
	9,11,10,
	2,0,1,
	3,5,4,
	7,8,6
};

void inline p2a(const int *phy, int *abs, int num = 18)
{
	for(int i=0;i<num;++i)
	{
		abs[i]=MapPhyToAbs[phy[i]];
	}
}
void inline a2p(const int *abs, int *phy, int num = 18)
{
	for(int i=0;i<num;++i)
	{
		phy[i]=MapAbsToPhy[abs[i]];
	}
}




int home(Robots::ROBOT_BASE *pRobot, const Robots::GAIT_PARAM_BASE *param, Aris::RT_CONTROL::CMachineData &data)
{
	double homeIn[18]=
	{
			0.675784824916295,0.697784816196987,0.697784816196987,
			0.675784824916295,0.697784816196987,0.697784816196987,
			0.675784824916295,0.697784816196987,0.697784816196987,
			0.675784824916295,0.697784816196987,0.697784816196987,
			0.675784824916295,0.697784816196987,0.697784816196987,
			0.675784824916295,0.697784816196987,0.697784816196987,};

	bool isAllHomed=true;

	int id[18];
	a2p(param->motorID,id,param->motorNum);

	for(int i=0;i< param->motorNum;++i)
	{
		if(data.isMotorHomed[id[i]])
		{
			data.motorsCommands[id[i]]=Aris::RT_CONTROL::EMCMD_RUNNING;
			data.commandData[id[i]].Position=-HEXBOT_HOME_OFFSETS_RESOLVER[id[i]];
		}
		else
		{
			data.motorsCommands[id[i]]=Aris::RT_CONTROL::EMCMD_GOHOME;
			data.commandData[id[i]].Position=-HEXBOT_HOME_OFFSETS_RESOLVER[id[i]];
			isAllHomed=false;

			if(param->count%1000==0)
			{
				rt_printf("motor %d not homed, physical id is:\n",id[i]);
				rt_printf("motor %d not homed, absolute id is:\n",param->motorID[i]);
			}
		}
	}


	if(isAllHomed)
	{
		double pBody[6]{0,0,0,0,0,0},vBody[6]{0};
		double vEE[18]{0};

		pRobot->SetPin(nullptr,pBody);

		for(int i=0;i < param->legNum;++i)
		{

			rt_printf("leg:%d\n",param->legID[i]);
			pRobot->pLegs[param->legID[i]]->SetPin(&homeIn[param->legID[i]*3]);
		}
		//pRobot->SetPin(homeIn,pBody);
		pRobot->SetVee(vEE,vBody);

		return 0;
	}
	else
	{


		return -1;
	}
};
int enable(Robots::ROBOT_BASE *pRobot, const Robots::GAIT_PARAM_BASE *param, Aris::RT_CONTROL::CMachineData &data)
{
	static Aris::RT_CONTROL::CMachineData lastCmdData;

	bool isAllRunning=true;

	int id[18];
	a2p(param->motorID,id,param->motorNum);

	for (int i=0;i< param->motorNum;++i)
	{
		if(data.motorsStates[id[i]]==Aris::RT_CONTROL::EMSTAT_RUNNING)
		{
			data.motorsCommands[id[i]]=Aris::RT_CONTROL::EMCMD_RUNNING;
			data.commandData[id[i]]=lastCmdData.commandData[id[i]];
		}
		else if(data.motorsStates[id[i]]==Aris::RT_CONTROL::EMSTAT_ENABLED)
		{
			data.motorsCommands[id[i]]=Aris::RT_CONTROL::EMCMD_RUNNING;
			data.commandData[id[i]]=data.feedbackData[id[i]];
			lastCmdData.commandData[id[i]]=data.feedbackData[id[i]];
			isAllRunning=false;
		}
		else
		{
			data.motorsCommands[id[i]]=Aris::RT_CONTROL::EMCMD_ENABLE;
			isAllRunning=false;
		}
	}

	if(isAllRunning)
	{
		return 0;
	}
	else
	{
		return -1;
	}
};
int disable(Robots::ROBOT_BASE *pRobot, const Robots::GAIT_PARAM_BASE *param, Aris::RT_CONTROL::CMachineData &data)
{
	int id[18];
	a2p(param->motorID,id,param->motorNum);


	bool isAllDisabled=true;
	for (int i=0;i< param->motorNum;++i)
	{
		if(data.motorsStates[id[i]]!=Aris::RT_CONTROL::EMSTAT_STOPPED)
		{
			data.motorsCommands[id[i]]=Aris::RT_CONTROL::EMCMD_STOP;
			isAllDisabled=false;
		}
	}

	if(isAllDisabled)
	{
		return 0;
	}
	else
	{
		return -1;
	}
}
int resetOrigin(Robots::ROBOT_BASE *pRobot, const Robots::GAIT_PARAM_BASE *param, Aris::RT_CONTROL::CMachineData &data)
{
	double pEE[18],pBody[6]{0},vEE[18],vBody[6]{0};
	pRobot->GetPee(pEE,"B");
	pRobot->GetVee(vEE,"B");

	pRobot->SetPee(pEE,pBody,"G");
	pRobot->SetVee(vEE,vBody,"G");

	return 0;
}
int runGait(Robots::ROBOT_BASE *pRobot, const Robots::GAIT_PARAM_BASE *pParam, Aris::RT_CONTROL::CMachineData &data)
{
	int ret=0;
	double pIn[18];

	ret = pRobot->RunGait(pParam->cmdID,pParam);

	rt_printf("return %d\n",ret);

	pRobot->GetPin(pIn);

	for(int i=0;i<pParam->motorNum;++i)
	{
		data.motorsCommands[MapAbsToPhy[pParam->motorID[i]]]=Aris::RT_CONTROL::EMCMD_RUNNING;
		data.commandData[MapAbsToPhy[pParam->motorID[i]]].Position=pIn[pParam->motorID[i]]*meter2count;
	}


	return ret;
}



Robots::ROBOT_III robot;
int execute_cmd(int count,char *cmd, Aris::RT_CONTROL::CMachineData &data)
{
	static double pBody[6]{0},vBody[6]{0},pEE[18]{0},vEE[18]{0};

	int ret;

	Robots::GAIT_PARAM_BASE *pParam=reinterpret_cast<Robots::GAIT_PARAM_BASE *>(cmd);
	pParam->count=count;

	memcpy(pParam->beginPee,pEE,sizeof(pEE));
	memcpy(pParam->beginVee,vEE,sizeof(vEE));
	memcpy(pParam->beginBodyPE,pBody,sizeof(pBody));
	memcpy(pParam->beginBodyVel,pBody,sizeof(vBody));

	switch (pParam->cmdType)
	{
	case ENABLE:
		ret = enable(&robot, pParam , data);
		break;
	case DISABLE:
		ret = disable(&robot, pParam ,data);
		break;
	case HOME:
		ret = home(&robot, pParam, data);
		break;
	case RESET_ORIGIN:
		ret = resetOrigin(&robot, pParam,data);
		break;
	case RUN_GAIT:
		ret = runGait(&robot, pParam,data);
		break;
	default:
		rt_printf("unknown cmd type\n");
		break;

	}

	if(ret==0)
	{
		robot.GetBodyPe(pBody);
		robot.GetPee(pEE);
		robot.GetBodyVel(vBody);
		robot.GetVee(vEE);


		rt_printf("%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n"
				,pEE[0],pEE[1],pEE[2],pEE[3],pEE[4],pEE[5],pEE[6],pEE[7],pEE[8]
				,pEE[9],pEE[10],pEE[11],pEE[12],pEE[13],pEE[14],pEE[15],pEE[16],pEE[17]);
		rt_printf("%f %f %f %f %f %f\n"
						,pBody[0],pBody[1],pBody[2],pBody[3],pBody[4],pBody[5]);
	}


	return ret;
}
int tg(Aris::RT_CONTROL::CMachineData &data,Aris::Core::RT_MSG &recvMsg,Aris::Core::RT_MSG &sendMsg)
{
	static double pBodyPE[6]{0},pEE[18]{0};

	static const int cmdSize=8192;

	static char cmdQueue[50][cmdSize];

	static int currentCmd=0;
	static int cmdNum=0;

	static int count = 0;

	static Aris::RT_CONTROL::CMachineData lastCmdData=data,lastStateData=data;
	static Aris::RT_CONTROL::CMachineData stateData, cmdData;

	stateData=data;
	cmdData=data;

	switch (recvMsg.GetMsgID())
	{
	case 0:
		recvMsg.Paste(cmdQueue[(currentCmd+cmdNum)%10]);
		++cmdNum;
		break;
	default:
		break;
	}


	if(cmdNum>0)
	{
		if(execute_cmd(count,cmdQueue[currentCmd],cmdData)==0)
		{
			count = 0;
			currentCmd= (currentCmd+1)%10;
			cmdNum--;
			rt_printf("cmd finished\n");
		}
		else
		{
			count++;
		}

		if(count%1000==0)
		{
			rt_printf("the cmd is:%d in count:%d\n",cmdData.motorsCommands[0],count);
		}
	}
	else
	{
		cmdData=lastCmdData;
	}



	for(int i=0;i<18;++i)
	{
		if(lastCmdData.motorsCommands[i]==Aris::RT_CONTROL::EMCMD_RUNNING)
		{
			if(cmdData.motorsCommands[i]==Aris::RT_CONTROL::EMCMD_RUNNING)
			{


				if(std::abs(lastCmdData.commandData[i].Position-cmdData.commandData[i].Position)>20000)
				{
					rt_printf("data is not continuous\n");
					data=lastCmdData;
					return 0;
				}
			}
		}
	}


	data=cmdData;

	lastStateData=stateData;
	lastCmdData=cmdData;





	return 0;
}
