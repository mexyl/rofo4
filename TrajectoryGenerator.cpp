#include"TrajectoryGenerator.h"



void TrajectoryGenerator::HexapodRofoGait::generateRobotGait(Robots::RobotBase& rbt, const Robots::WalkParam &param)
{
	auto &robot = static_cast<Robots::RobotTypeI &>(rbt);

	auto getData = [&]() { 
		for (int i = 0;i < 6;i++)
		{
			this->sensorFceData.row(i)
				<< param.force_data->at(i).Fx
				, param.force_data->at(i).Fy
				, param.force_data->at(i).Fz
				, param.force_data->at(i).Mx
				, param.force_data->at(i).My
				, param.force_data->at(i).Mz;
		}
		
		for (int i = 0; i < 18;i++)
		{
			//for VIII
			static double input2count = 9216000;
			this->torque[i] = ((double)param.last_motion_raw_data->at(i).feedback_cur)
				/ 1000.0 * 18 * 38.5 / 1000 * 2 * M_PI / (32.0 / 1000)*4.5;
			this->position[i] = (double)param.last_motion_raw_data->at(i).feedback_pos / input2count;
			
			if (param.count == 0)
			{
				this->position_last[i] = this->position[i];
			}
			this->velocity[i] = (this->position[i] - this->position_last[i])*1000;
			if (param.count == 0)
			{
				this->velocity_last[i] = this->velocity[i];
			}
			this->acceleration[i] = (this->velocity[i] - this->velocity_last[i])*1000;

			this->position_last[i] = this->position[i];
			this->velocity_last[i] = this->velocity[i];

		}

	};
	getData();



};