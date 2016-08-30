// git 627cf7 milestone 
// git bbec68 milestone
#pragma once

#include"Curves.h"
#include <Robot_Type_I.h>
#include <Robot_Gait.h>
#include <functional>
#include"Filter.h"
#include"log.h"
#include<utility>

struct CLIMB_PARAM: public aris::server::GaitParamBase
{
    double yt{1000.0};
    double zt{800.0};
    std::int32_t totalCount{ 3000 };
    std::int32_t n{ 2 };
    double d{ 0.5 };
    double h{ 0.05 };
    double alpha{ 0.3 };
    double beta{ 0.3 };
};

extern peripherals::Log robot_log;

namespace TrajectoryGenerator
{
	enum LegID { LF = 0, LM, LR, RF, RM, RR };
	enum MotionID {IDLE,STANDSTILL,FORWARD,BACKWARD,TURNLEFT,TURNRIGHT};
	enum SequenceStage {INIT,RUNNING};
	enum ForceMode { NONE, INDIRECT, SENSOR };
	

	class SinglePointTrajBase
	{
	public:
		virtual void setID(int id) { this->_id = id; };
		virtual int getID() { return _id; };
		virtual void setStage(SequenceStage stg) { _sqeStage = stg; };
		virtual SequenceStage getStage() { return _sqeStage; };
		int _id;
		SequenceStage _sqeStage=INIT;// initneed
		Eigen::Vector3d _trajStartPoint;
	};
	class HexapodBody :public SinglePointTrajBase
	{
	public:

		/* everything about body*/
		CurvesPlan::OneSplineSequence bodyFirstSequence;
		CurvesPlan::OneSplineSequence bodySecondSequence;

		CurvesPlan::CurvesSequenceBase *currentSequence = nullptr;
		CurvesPlan::CurvesSequenceBase *lastSequence = nullptr;

		Eigen::Vector3d body_position_ref_beginMak;// loop update
		Eigen::Vector3d body_position_origin;// not change at all
	};


	class HexapodSingleLeg :public SinglePointTrajBase
	{
	public:
		CurvesPlan::NormalSequence normalSequence;
		CurvesPlan::ObstacleSequence obstacleSquence;
		CurvesPlan::TentativeSequence tentativeSequence;
		CurvesPlan::RetractSequence retractSequence;

		/* in first tentative, */
		/* when change to standstill ,tentativeCounts reset to 0 
			when setStage, tentativeCounts+1
		*/
		int tentativeCounts = 0;

		CurvesPlan::StandStillSequence standstillSequence;

		CurvesPlan::CurvesSequenceBase *currentSequence=nullptr;
		CurvesPlan::CurvesSequenceBase *lastSequence=nullptr;

		Eigen::Vector3d _refPosition;
		// _refSpace, x,y,z; -neg,+pos
		Eigen::Matrix<double,1, 2> _refSpaceY;


		 

		double foot_position_ref_body[3];
		double foot_position_ref_beginMak[3];
		
		double force_jacobian_direct_ref_body[3][3];
		double force_jacobian_direct_ref_beginMak[3][3];
		//double prismatic_dynamic_force[3];
		//double prismatic_actuation_force[3];
		double prismatic_external_force[3];

		double foot_force_extern_ref_body[3];
		double foot_force_extern_ref_beginMak[3];

		Robots::LegI *model;

		Filter::Threshold thrYpos;
		Filter::Threshold thrZpos;
		Filter::Threshold thrZneg;

		Filter::Threshold thrYposInd;
		Filter::Threshold thrZposInd;
		Filter::Threshold thrZnegInd;

		Filter::CFilterFIR_I fyFilter;
		Filter::CFilterFIR_I fzFilter;

		Filter::CFilterFIR_I fyFilterInd;
		Filter::CFilterFIR_I fzFilterInd;

		Filter::CFilterFIR_I *pfyFilter;
		Filter::CFilterFIR_I *pfzFilter;

		Filter::Threshold *pthrYpos;
		Filter::Threshold *pthrZpos;
		Filter::Threshold *pthrZneg;

		/* buggy */
		std::pair<Filter::CFilterFIR_I*, Filter::Threshold*> yPosForceDetector{nullptr,nullptr};
		std::pair<Filter::CFilterFIR_I*, Filter::Threshold*> zPosForceDetector{ nullptr,nullptr };
		std::pair<Filter::CFilterFIR_I*, Filter::Threshold*> zNegForceDetector{ nullptr,nullptr };

		
	};


	/* Trajectories */
	class HexapodTrajectoriesBase
	{
	public:
		/* clear static values */
		virtual void reset() = 0;
		/* input/output */
		/* WARNING, at the first cycle, you need last position data */

        //virtual void setCurrentMotorTorque(Eigen::Matrix<double, 18, 1> &tor) = 0;
        //virtual void setCurrentMotorPostion(Eigen::Matrix<double, 18, 1> &pos) = 0;

        //virtual void setCurrentFeetPos(Eigen::Matrix<double, 6, 3> &pee) = 0;
        //virtual void setCurrentBodyPos(Eigen::Matrix<double, 2, 3> &bee) = 0;
        //virtual void setCurrentForceSensorData(Eigen::Matrix<double, 6, 6> &fdata) = 0;

        //virtual void getTargetFeetPos(Eigen::Matrix<double, 6, 3> &pee) = 0;
        //virtual void getTargetBodyPos(Eigen::Matrix<double, 2, 3>&bee) = 0;

		Eigen::Matrix<double, 18, 1> torque,position,velocity,acceleration;
		Eigen::Matrix<double, 18, 1> torque_filtered, position_filtered, velocity_filtered, acceleration_filtered;
		Eigen::Matrix<double, 18, 1> position_last, velocity_last;
		Eigen::Matrix<double, 6, 3> currentPee, targetPee; // feet
		Eigen::Matrix<double, 2, 3> currentBee, targetBee; // body
		Eigen::Matrix<double, 6, 6> sensorFceData;

		/*model calculation*/
		// set filters in the derived class;
        //virtual void evaluateModel() = 0;
		Eigen::Matrix<double, 6, 3> indirectFceData;

		/* gaitGeneration */
        //virtual void trajectoryGeneration() = 0;
	};

	class HexapodRofoGait :public HexapodTrajectoriesBase
	{
		/* have a function to control force sensor */
		/* have a function to control indirect force sensor */
		
	public:
		virtual void reset();
        int generateRobotGait(Robots::RobotBase& rbt,MotionID motion,const aris::dynamic::PlanParamBase &param_in);

		void setTentative(bool b) { this->isTentative = b; };
		void setForceMode(ForceMode mode);

	public:
        std::vector<HexapodSingleLeg> legTraj = std::vector<HexapodSingleLeg>(6);
		
		ForceMode forceMode = INDIRECT;
		const int motNum = 18;
		std::vector<Filter::CFilterFIR_I>  posFilter = std::vector<Filter::CFilterFIR_I>(motNum);
		std::vector<Filter::CFilterFIR_I>  velFilter = std::vector<Filter::CFilterFIR_I>(motNum);
		std::vector<Filter::CFilterFIR_I>  accFilter = std::vector<Filter::CFilterFIR_I>(motNum);
		std::vector<Filter::CFilterFIR_I>  torFilter = std::vector<Filter::CFilterFIR_I>(motNum);
        std::vector<Filter::CFilterFIR_I> fyFilterInd = std::vector<Filter::CFilterFIR_I>(6);
		std::vector<Filter::CFilterFIR_I> fzFilterInd = std::vector<Filter::CFilterFIR_I>(6);
		std::vector<Filter::CFilterFIR_I> fyFilter = std::vector<Filter::CFilterFIR_I>(6);
		std::vector<Filter::CFilterFIR_I> fzFilter = std::vector<Filter::CFilterFIR_I>(6);

		const int fceNum = 6;
		const int legNum = 6;

        bool isTentative = true;
	//private:
		double *pIn, *vIn, *aIn;
		double fIn[18];
		double fInExtern[18];
		//aris::dynamic::FloatMarker beginMak;
		MotionID currentMotion=IDLE;
		//MotionID nextMotion = IDLE;
		Eigen::Matrix<double, 3, 3> _rot2Bot;
		
		//the use of step count TBD
		// 1: f 0.3 s 0.3; f=1 s=1  2
		// 2: f 0.3 s 0.6 f 0.3; f=2 s=1  3
		// 3: f 0.3 s 0.6 f 0.6 s 0.6 f 0.3; f=3 s=2 5
		// 

		int stepCount; // used for 
		int totalStepCounts;
		double stepLength = 0.25;
		double stepHeight = 0.05;

		/* everything for robot body */
		HexapodBody bodyPos;
		HexapodBody bodyAng;
		Eigen::Vector3d bodyPosVec;
		Eigen::Vector3d bodyAngVec;
	};
}
