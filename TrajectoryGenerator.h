#pragma once

#include"Curves.h"
#include <Robot_Type_I.h>
#include <Robot_Gait.h>
#include <functional>
#include"Filter.h"

namespace TrajectoryGenerator
{
	enum LegID { LF = 0, LM, LR, RF, RM, RR };

	class SinglePointTrajBase
	{
	public:
	};
	class HexapodSingleLeg :public SinglePointTrajBase
	{
	public:
		CurvesPlan::NormalSequence normalSequence;
		CurvesPlan::ObstacleSequence obstacleSquence;
		CurvesPlan::TentativeSequence tentativeSequence;
		CurvesPlan::StandStill standstillSequence;

		CurvesPlan::CurvesSequenceBase *currentSequence;
		CurvesPlan::CurvesSequenceBase *lastSequence;

		Eigen::Vector3d _refPosition;
		int _sequenceCounts;
		int _currentCounts;

		double foot_position_ref_body[3];
		double foot_position_ref_outside[3];
		
		double force_jacobian_direct_ref_body[3][3];
		double force_jacobian_direct_ref_outside[3][3];
		//double prismatic_dynamic_force[3];
		//double prismatic_actuation_force[3];
		double prismatic_external_force[3];

		double foot_force_extern_ref_body[3];
		double foot_force_extern_ref_outside[3];

		Robots::LegI *model;

		Filter::Threshold thrYpos;
		Filter::Threshold thrZpos;
		Filter::Threshold thrZneg;
		Filter::CFilterFIR_I *fyFilter;
		Filter::CFilterFIR_I *fzFilter;
		
	};


	/* Trajectories */
	class HexapodTrajectoriesBase
	{
	public:
		/* clear static values */
		virtual void reset() = 0;
		/* input/output */
		/* WARNING, at the first cycle, you need last position data */
		virtual void setCurrentMotorTorque(Eigen::Matrix<double, 18, 1> &tor) = 0;
		virtual void setCurrentMotorPostion(Eigen::Matrix<double, 18, 1> &pos) = 0;

		virtual void setCurrentFeetPos(Eigen::Matrix<double, 6, 3> &pee) = 0;
		virtual void setCurrentBodyPos(Eigen::Matrix<double, 2, 3> &bee) = 0;
		virtual void setCurrentForceSensorData(Eigen::Matrix<double, 6, 6> &fdata) = 0;

		virtual void getTargetFeetPos(Eigen::Matrix<double, 6, 3> &pee) = 0;
		virtual void getTargetBodyPos(Eigen::Matrix<double, 2, 3>&bee) = 0;

		Eigen::Matrix<double, 18, 1> torque,position,velocity,acceleration;
		Eigen::Matrix<double, 18, 1> torque_filtered, position_filtered, velocity_filtered, acceleration_filtered;
		Eigen::Matrix<double, 18, 1> position_last, velocity_last;
		Eigen::Matrix<double, 6, 3> currentPee, targetPee; // feet
		Eigen::Matrix<double, 2, 3> currentBee, targetBee; // body
		Eigen::Matrix<double, 6, 6> sensorFceData;

		/*model calculation*/
		// set filters in the derived class;
		virtual void evaluateModel() = 0;
		Eigen::Matrix<double, 6, 3> indirectFceData;

		/* gaitGeneration */
		virtual void trajectoryGeneration() = 0;
	};

	class HexapodRofoGait :public HexapodTrajectoriesBase
	{
		/* have a function to control force sensor */
		/* have a function to control indirect force sensor */
		enum ForceMode {NONE,INDIRECT,SENSOR};
	public:
		virtual void reset();
		void generateRobotGait(Robots::RobotBase& rbt,const Robots::WalkParam &param);

		void setTentative(bool b) { this->isTentative = b; };
		void setForceMode(ForceMode mode);

	public:
		std::vector<HexapodSingleLeg> legTraj = std::vector<HexapodSingleLeg>(6);
		
		ForceMode forceMode = SENSOR;
		const int motNum = 18;
		std::vector<Filter::CFilterFIR_I>  posFilter = std::vector<Filter::CFilterFIR_I>(motNum);
		std::vector<Filter::CFilterFIR_I>  velFilter = std::vector<Filter::CFilterFIR_I>(motNum);
		std::vector<Filter::CFilterFIR_I>  accFilter = std::vector<Filter::CFilterFIR_I>(motNum);
		std::vector<Filter::CFilterFIR_I>  torFilter = std::vector<Filter::CFilterFIR_I>(motNum);
		const int fceNum = 6;
		std::vector<Filter::CFilterFIR_I> fyFilter = std::vector<Filter::CFilterFIR_I>(fceNum);
		std::vector<Filter::CFilterFIR_I> fzFilter = std::vector<Filter::CFilterFIR_I>(fceNum);
		const int legNum = 6;
		std::vector<Filter::CFilterFIR_I> fyFilterInd = std::vector<Filter::CFilterFIR_I>(legNum);
		std::vector<Filter::CFilterFIR_I> fzFilterInd = std::vector<Filter::CFilterFIR_I>(legNum);


		bool isTentative = true;
	//private:
		double *pIn, *vIn, *aIn;
		double fIn[18];
		double fInExtern[18];
		aris::dynamic::FloatMarker beginMak;
	};
}
