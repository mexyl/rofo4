#include "rofo4.h"
#include "log_data.h"
#include<iostream>
using namespace std;
using namespace  Rofo;

//force_gait::GaitRobot rofo;

bool IsRofoEnd=false;

LogData temp_log_data;
// static variables for adjustY function
Ay_Param* pAP;

double adjustYTargetPee[18];

TrajectoryGenerator::HexapodRofoGait rofo;

auto Rofo::rofoEndParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
    CLIMB_PARAM  param;
/*
    for(auto &i:params)
    {
        if(i.first=="count")
        {
            param.count=std::stoi(i.second);
        }
    }
*/
    msg.copyStruct(param);
    if(IsRofoEnd)
    {
        IsRofoEnd=false;
        std::cout<<"climb enabled"<<std::endl;

    }
    else
    {
        IsRofoEnd=true;
        std::cout<<"climb disabled"<<std::endl;
    }

    IsRofoEnd=true;

    std::cout<<"finished parse  endclimb"<<std::endl;
}

auto Rofo::rofoEndGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
    rt_printf("end climb rt executed\n");
    auto &param = static_cast<const Robots::WalkParam &>(param_in);
    rt_printf("LAST TARGE; LAST FEEDBACK; TARGET; FEEDBACK;\n");
    for(int i=0;i<18;i++)
    {
        rt_printf("%d\t%d\t%d\t%d\n",param.last_motion_raw_data->at(i).target_pos
                  ,param.last_motion_raw_data->at(i).feedback_pos
                  ,param.motion_raw_data->at(i).target_pos
                  ,param.motion_raw_data->at(i).feedback_pos);
    }

    return 0;
}


int Rofo::RofoWalkInit()
{
    rofo.setTentative(false);
    rofo.reset();
    rofo.setForceMode(TrajectoryGenerator::ForceMode::INDIRECT);
}

auto Rofo::rofoParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
    CLIMB_PARAM param;

    for(auto &i:params)
    {
        if(i.first=="distance")
        {

            param.d=std::stod(i.second);
        }
        else if(i.first=="n")
        {
            param.n=std::stoi(i.second);
        }
		else if (i.first == "height")
		{
            param.h = std::stod(i.second);
		}
		else if (i.first=="totalCount")
		{
			param.totalCount = std::stoi(i.second);
		}
        else if(i.first=="zthr")
        {
            param.zt=std::stod(i.second);
        }
        else if(i.first=="ythr")
        {
            param.yt=std::stod(i.second);
        }

    }


    msg.copyStruct(param);
    std::cout<<"finished parse climb"<<std::endl;

}

auto Rofo::rofoGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{

    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const CLIMB_PARAM &>(param_in);


    TrajectoryGenerator::MotionID mot=TrajectoryGenerator::MotionID::FORWARD;
    static int ret;

//    if(param.count%1000==0)
//    {
//        rt_printf("rofoGait is running\n");
//    }

    ret=rofo.generateRobotGait(robot,mot,param);

    return ret;

}

auto Rofo::ayParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
    Ay_Param param;

    param.if_check_pos_min = false;
    param.if_check_pos_max = false;


    for(auto &i : params)
    {
        if (i.first == "all")
        {
            std::fill_n(param.active_leg, 6, true);
        }
        else if (i.first == "first")
        {
            param.active_leg[0] = true;
            param.active_leg[1] = false;
            param.active_leg[2] = true;
            param.active_leg[3] = false;
            param.active_leg[4] = true;
            param.active_leg[5] = false;
            std::fill_n(param.active_motor, 18, false);
            std::fill_n(param.active_motor + 0, 3, true);
            std::fill_n(param.active_motor + 6, 3, true);
            std::fill_n(param.active_motor + 12, 3, true);
        }
        else if (i.first == "second")
        {
            param.active_leg[0] = false;
            param.active_leg[1] = true;
            param.active_leg[2] = false;
            param.active_leg[3] = true;
            param.active_leg[4] = false;
            param.active_leg[5] = true;
            std::fill_n(param.active_motor, 18, false);
            std::fill_n(param.active_motor + 3, 3, true);
            std::fill_n(param.active_motor + 9, 3, true);
            std::fill_n(param.active_motor + 15, 3, true);
        }
        else if(i.first=="ay")
        {
            double ay=stod(i.second);
            if(ay>-0.72||ay<-1.08)
            {
                ay=-0.85;
                std::cout<<"ay out of range default set to %d"<<ay<<std::endl;
            }
            param.aY=ay;
            param.IsRel=false;
        }
        else if(i.first=="dy")
        {
            param.dY=stod(i.second);
            if(param.dY<-0.3||param.dY>0.3)
            {
                param.dY=-0.05;
            }
            param.IsRel=true;
        }
        else
        {
            throw std::runtime_error("unknown param in ayParse func");
        }

    }
    msg.copyStruct(param);
}

auto Rofo::ayGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
    //a duplicated function of recoverGait()
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const Ay_Param &>(param_in);


    for(int i=0;i<18;i++)
    {
        // 2016-04-12: raw_data is already in abs layer, i think.

        temp_log_data.raw_data_position[i]=param.motion_raw_data->at(i).feedback_pos;
        temp_log_data.raw_data_torque[i]=param.motion_raw_data->at(i).feedback_cur;
        temp_log_data.count=param.count;
    }
    rt_dev_sendto(robot_log.file_id_real_time,&temp_log_data,sizeof(temp_log_data),0,NULL,0);

    static aris::server::ControlServer &cs = aris::server::ControlServer::instance();

    static double beginPee[18], endPee[18];

    if (param.count == 0)
    {
        //std::copy_n(param.motion_feedback_pos->data(), 18, beginPin);
        // get beginPee, then calculate the endPee
        robot.GetPee(beginPee, robot.body());
        robot.GetPee(endPee, robot.body());

        const double pe[6]{ 0 };
        robot.SetPeb(pe);
//        robot.SetPee(param.alignPee);
//        robot.GetPin(alignPin);
        robot.SetPee(beginPee, robot.body());

        // set endPee
        if(param.IsRel)
        {
            for(int i=0;i<6;i++)
            {
                endPee[i*3+1]+=param.dY;
            }
        }
        else
        {
            for(int i=0;i<6;i++)
            {

                endPee[i*3+1]=param.aY;
            }

        }
    }

    //int leftCount = param.count < param.recover_count ? 0 : param.recover_count;
    //int rightCount = param.count < param.recover_count ? param.recover_count : param.recover_count + param.align_count;

    double s = -(PI / 2)*cos(PI * (param.count + 1) / param.ay_count) + PI / 2;

    for (int i = 0; i < 6; ++i)
    {
        if (param.active_leg[i])
        {
            {
                double pEE[3];
                for (int j = 0; j < 3; ++j)
                {
                    pEE[j] = beginPee[i * 3 + j] * (cos(s) + 1) / 2 + endPee[i * 3 + j] * (1 - cos(s)) / 2;
                }

                robot.pLegs[i]->SetPee(pEE);
            }
        }
    }

    // recover 自己做检查 //
    for (int i = 0; i<18; ++i)
    {
        if (param.active_motor[i] && (param.last_motion_raw_data->at(i).cmd == aris::control::EthercatMotion::RUN))
        {

            if (param.motion_raw_data->at(i).target_pos >(cs.controller().motionAtAbs(i).maxPosCount() + param.margin_offset * cs.controller().motionAtAbs(i).pos2countRatio()))
            {
                rt_printf("Motor %i's target position is bigger than its MAX permitted value in recover, you might forget to GO HOME\n", i);
                rt_printf("The min, max and current count are:\n");
                for (std::size_t i = 0; i < cs.controller().motionNum(); ++i)
                {
                    rt_printf("%d   %d   %d\n", cs.controller().motionAtAbs(i).minPosCount(), cs.controller().motionAtAbs(i).maxPosCount(), param.motion_raw_data->at(i).target_pos);
                }
                rt_printf("recover failed\n");
                return 0;
            }
            if (param.motion_raw_data->at(i).target_pos < (cs.controller().motionAtAbs(i).minPosCount() - param.margin_offset * cs.controller().motionAtAbs(i).pos2countRatio()))
            {
                rt_printf("Motor %i's target position is smaller than its MIN permitted value in recover, you might forget to GO HOME\n", i);
                rt_printf("The min, max and current count are:\n");
                for (std::size_t i = 0; i < cs.controller().motionNum(); ++i)
                {
                    rt_printf("%d   %d   %d\n", cs.controller().motionAtAbs(i).minPosCount(), cs.controller().motionAtAbs(i).maxPosCount(), param.motion_raw_data->at(i).target_pos);
                }
                rt_printf("recover failed\n");
                return 0;
            }
        }
    }

    return param.ay_count - param.count - 1;

}

