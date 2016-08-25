#include"TrajectoryGenerator.h"

void TrajectoryGenerator::HexapodRofoGait::reset()
{

};

void TrajectoryGenerator::HexapodRofoGait::setForceMode(ForceMode mode)
{
	this->forceMode = mode;
	switch (forceMode)
	{
	case TrajectoryGenerator::HexapodRofoGait::NONE:
		for (int i = 0;i < 6;i++)
		{
			legTraj[i].fyFilter = NULL;
		}
		break;
	case TrajectoryGenerator::HexapodRofoGait::INDIRECT:
		for (int i = 0;i < 6;i++)
		{
			legTraj[i].fyFilter = &this->fyFilterInd[i];
		}
		break;
	case TrajectoryGenerator::HexapodRofoGait::SENSOR:
		for (int i = 0;i < 6;i++)
		{
			legTraj[i].fyFilter = &this->fyFilter[i];
		}
		break;
	default:
		break;
	}
}

void TrajectoryGenerator::HexapodRofoGait::generateRobotGait(Robots::RobotBase& rbt,MotionID motion, const Robots::WalkParam &param)
{
	auto &robot = static_cast<Robots::RobotTypeI &>(rbt);

	auto getData = [&]() 
	{ 	
		// need to add mapping in this for loop
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

		if (param.count == 0)
		{
			for (int i = 0;i < 60;i++)
			{
				for (int j = 0;j < 6;j++)
				{
					this->fzFilter[j].FeedData(sensorFceData(j, 2));
					this->fyFilter[j].FeedData(sensorFceData(j, 1));
				}
			}
		}
		else
		{
			for (int j = 0;j < 6;j++)
			{
				this->fzFilter[j].FeedData(sensorFceData(j, 2));
				this->fyFilter[j].FeedData(sensorFceData(j, 1));
			}
		}
		
		for (int i = 0; i < 18;i++)
		{
			//for VIII begin
			static double input2count = 9216000;
			this->torque[i] = ((double)param.last_motion_raw_data->at(i).feedback_cur)
				/ 1000.0 * 18 * 38.5 / 1000 * 2 * M_PI / (32.0 / 1000)*4.5;
			this->position[i] = (double)param.last_motion_raw_data->at(i).feedback_pos / input2count;
			// for VIII end

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
			if (param.count == 0)
			{
				/*fill filter*/
				for (int i = 0;i < 60;i++)
				{
					for (int j = 0;j < 18;j++)
					{
						this->posFilter[j].FeedData(this->position[j]);
						this->velFilter[j].FeedData(this->velocity[j]);
						this->accFilter[j].FeedData(this->acceleration[j]);
						this->torFilter[j].FeedData(this->torque[j]);
					}
				}

			}
			else
			{
				/*normal filter*/
				for (int j = 0;j < 18;j++)
				{
					this->posFilter[j].FeedData(this->position[j]);
					this->velFilter[j].FeedData(this->velocity[j]);
					this->accFilter[j].FeedData(this->acceleration[j]);
					this->torFilter[j].FeedData(this->torque[j]);
				}
			}
			for (int i = 0;i < 18;i++)
			{
				this->position_filtered[i] = this->posFilter[i].GetData();
				this->velocity_filtered[i] = this->velFilter[i].GetData();
				this->acceleration_filtered[i] = this->accFilter[i].GetData();
				this->torque_filtered[i] = this->torFilter[i].GetData();
			}
		}

	};
	getData();

	auto evalModel = [&]() 
	{
		legTraj[0].model = robot.pLF;
		legTraj[1].model = robot.pLM;
		legTraj[2].model = robot.pLR;
		legTraj[3].model = robot.pRF;
		legTraj[4].model = robot.pRM;
		legTraj[5].model = robot.pRR;


		pIn = &this->position_filtered[0];
		robot.SetPin(pIn);
		vIn = &this->velocity_filtered[0];
		robot.SetVin(vIn);
		aIn = &this->acceleration_filtered[0];
		robot.SetAin(aIn);

		robot.SetFixFeet("000000");
		robot.FastDyn();
		robot.GetFin(fIn);
		/* get external force result on prismatic joint */
		for (int i = 0;i < motNum;i++)
		{
			fInExtern[i] = this->torque_filtered[i] - fIn[i];
		}
		for (int i = 0;i < 6;i++)
		{
			legTraj[i].prismatic_external_force[0] = fInExtern[i * 3];
			legTraj[i].prismatic_external_force[1] = fInExtern[i * 3 + 1];
			legTraj[i].prismatic_external_force[2] = fInExtern[i * 3 + 2];
			
			legTraj[i].model->GetPee(legTraj[i].foot_position_ref_outside,beginMak);

			legTraj[i].model->GetPee(legTraj[i].foot_position_ref_body,robot.body());
			legTraj[i].model->GetJfd(*legTraj[i].force_jacobian_direct_ref_body,robot.body());

			legTraj[i].model->GetPee(legTraj[i].foot_position_ref_outside, beginMak);
			legTraj[i].model->GetJfd(*legTraj[i].force_jacobian_direct_ref_outside, beginMak);

			aris::dynamic::s_dgemm(3,1,3,-1.0,
				*legTraj[i].force_jacobian_direct_ref_body,3,
				legTraj[i].prismatic_external_force,1,
				0,legTraj[i].foot_force_extern_ref_body,1);

			aris::dynamic::s_dgemm(3, 1, 3, -1.0,
				*legTraj[i].force_jacobian_direct_ref_outside, 3,
				legTraj[i].prismatic_external_force, 1,
				0, legTraj[i].foot_force_extern_ref_outside, 1);
		}
		if (param.count == 0)
		{
			for (int j=0;j < 60;j++)
			{
				for (int i = 0;i < 6;i++)
				{
					//*********************************************
					// here may need more discussion: body or world coordinate
					//*********************************************
					fyFilterInd[i].FeedData(legTraj[i].foot_force_extern_ref_outside[1]);
					fzFilterInd[i].FeedData(legTraj[i].foot_force_extern_ref_outside[2]);
				}
			}

		}
		else
		{
			for (int i = 0;i < 6;i++)
			{
				//*********************************************
				// here may need more discussion: body or world coordinate
				//*********************************************
				fyFilterInd[i].FeedData(legTraj[i].foot_force_extern_ref_outside[1]);
				fzFilterInd[i].FeedData(legTraj[i].foot_force_extern_ref_outside[2]);
			}
		}
		
		for (int i = 0;i < 6;i++)
		{
			if (legTraj[i].fyFilter)
			{
				legTraj[i].thrYpos.threshold(legTraj[i].fyFilter->GetData());
			}
			if (legTraj[i].fzFilter)
			{
				legTraj[i].thrZpos.threshold(legTraj[i].fzFilter->GetData());
				legTraj[i].thrZneg.threshold(legTraj[i].fzFilter->GetData());
			}
		}
		
	};
	evalModel();

	auto gaitGeneration = [&](MotionID mot) 
	{

	};
	gaitGeneration(motion);

};