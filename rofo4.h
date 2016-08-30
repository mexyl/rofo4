#ifndef ROFO4_H
#define ROFO4_H

#include <aris.h>
#include <Robot_Base.h>
#include <Robot_Type_I.h>
#include "TrajectoryGenerator.h"
#include "RofoConfig.h"
#include "log_data.h"


using namespace aris::core;

//extern force_gait::GaitRobot rofo;

extern bool IsRofoEnd;

extern LogData temp_log_data;

extern TrajectoryGenerator::HexapodRofoGait rofo;

namespace Rofo {







struct Ay_Param: public aris::server::GaitParamBase
{
    double aY; // real value of y, -1.1<aY<-0.72
    double dY;
    bool IsRel=false;
    std::int32_t ay_count{5000};
    //same as RecoverParam
    std::int32_t recover_count{ 3000 };
    std::int32_t align_count{ 3000 };
    bool active_leg[6]{ true,true,true,true,true,true };
    double margin_offset{0.01};//meter

};


int RofoWalkInit();

auto rofoParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto rofoGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

auto rofoEndParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto rofoEndGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

auto ayParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto ayGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

}
#endif

