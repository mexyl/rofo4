#pragma once
#include"Curves.h"

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

		Eigen::Matrix<double, 18, 1> torque, position;
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
		void indirectForceEstimation() {};

	public:
		std::vector<HexapodSingleLeg> legTraj;
		


	};
}
