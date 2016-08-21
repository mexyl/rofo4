#pragma once
#include"Curves.h"

namespace TrajectoryGenerator
{
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
		Eigen::Matrix<double, 6, 3> currentPee, targetPee;
		Eigen::Matrix<double, 2, 3> currentBee, targetBee;
		Eigen::Matrix<double, 6, 6> fce_data;

		/*model calculation*/
		// set filters in the derived class;
		virtual void evaluateModel() = 0;
		Eigen::Matrix<double, 6, 3> indirectFceData;

		/* gaitGeneration */
		virtual void trajectoryGeneration() = 0;
	};

	class HexapodRofoGait :public HexapodTrajectoriesBase
	{

	};
}
