#include"TrajectoryGenerator.h"

void TrajectoryGenerator::HexapodRofoGait::reset()
{
	_rot2Bot <<
		0, 0, 1,
		0, 1, 0,
		1, 0, 0;
	//  start
	// VIII
	legTraj[LF]._refPosition<<-0.3,-0.9,-065;
	legTraj[LM]._refPosition<<-0.45,-0.9,0;
	legTraj[LR]._refPosition<<-0.3,-0.9,0.65;
	legTraj[RF]._refPosition<<0.3,-0.9,-0.65;
	legTraj[RM]._refPosition<<0.45,-0.9,0;
	legTraj[RR]._refPosition<<0.3,-0.9,0.65;
	
	for (auto &i : legTraj)
	{
		i._refSpaceY << -1.05,-0.73;
	}
	// VIII
	this->setForceMode(this->forceMode);

	legTraj[LF].setID(LF);
	legTraj[LM].setID(LM);
	legTraj[LR].setID(LR);

	legTraj[RF].setID(RF);
	legTraj[RM].setID(RM);
	legTraj[RR].setID(RR);
	

};

void TrajectoryGenerator::HexapodRofoGait::setForceMode(ForceMode mode)
{
	this->forceMode = mode;
	switch (forceMode)
	{
	case TrajectoryGenerator::HexapodRofoGait::NONE:
		for (int i = 0;i < 6;i++)
		{
			legTraj[i].yPosForceDetector.first = NULL;
			legTraj[i].zPosForceDetector.first = NULL;
			legTraj[i].zNegForceDetector.first = NULL;
			legTraj[i].yPosForceDetector.second = NULL;
			legTraj[i].zPosForceDetector.second = NULL;
			legTraj[i].zNegForceDetector.second = NULL;
		}
		break;
	case TrajectoryGenerator::HexapodRofoGait::INDIRECT:
		for (int i = 0;i < 6;i++)
		{
			legTraj[i].yPosForceDetector.first = &legTraj[i].fyFilterInd;
			legTraj[i].zPosForceDetector.first = &legTraj[i].fzFilterInd;
			legTraj[i].zNegForceDetector.first = &legTraj[i].fzFilterInd;
			legTraj[i].yPosForceDetector.second = &legTraj[i].thrYposInd;
			legTraj[i].zPosForceDetector.second = &legTraj[i].thrZposInd;
			legTraj[i].zNegForceDetector.second = &legTraj[i].thrZnegInd;
		}
		break;
	case TrajectoryGenerator::HexapodRofoGait::SENSOR:
		for (int i = 0;i < 6;i++)
		{
			legTraj[i].yPosForceDetector.first = &legTraj[i].fyFilter;
			legTraj[i].zPosForceDetector.first = &legTraj[i].fzFilter;
			legTraj[i].zNegForceDetector.first = &legTraj[i].fzFilter;
			legTraj[i].yPosForceDetector.second = &legTraj[i].thrYpos;
			legTraj[i].zPosForceDetector.second = &legTraj[i].thrZpos;
			legTraj[i].zNegForceDetector.second = &legTraj[i].thrZneg;
		}
		break;
	default:
		for (int i = 0;i < 6;i++)
		{
			legTraj[i].yPosForceDetector.first = NULL;
			legTraj[i].zPosForceDetector.first = NULL;
			legTraj[i].zNegForceDetector.first = NULL;
			legTraj[i].yPosForceDetector.second = NULL;
			legTraj[i].zPosForceDetector.second = NULL;
			legTraj[i].zNegForceDetector.second = NULL;

		}
		break;
	}
}

void TrajectoryGenerator::HexapodRofoGait::generateRobotGait(Robots::RobotBase& rbt,MotionID motion, const Robots::WalkParam &param)
{
	auto &robot = static_cast<Robots::RobotTypeI &>(rbt);
	static aris::dynamic::FloatMarker beginMak{robot.ground()};

	auto getData = [&]() 
	{ 	
		if (param.count == 0)
		{
			// set postion of beginMak to the current robot postion
			// .pm() always return coordinates of the ground frame
			beginMak.setPrtPm(*robot.body().pm());
			beginMak.update();
		}
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
		
		if (param.count == 0 && forceMode == ForceMode::SENSOR)
		{
			for (int j = 0;j < 60;j++)
			{
				for (int i = 0;i < 6;i++)
				{
					legTraj[i].yPosForceDetector.first->FeedData(param.force_data->at(i).Fy);
					legTraj[i].zPosForceDetector.first->FeedData(param.force_data->at(i).Fz);
					legTraj[i].zNegForceDetector.first->FeedData(param.force_data->at(i).Fz);
				}
			}
		}
		else
		{
			for (int i = 0;i < 6;i++)
			{
				legTraj[i].yPosForceDetector.first->FeedData(param.force_data->at(i).Fy);
				legTraj[i].zPosForceDetector.first->FeedData(param.force_data->at(i).Fz);
				legTraj[i].zNegForceDetector.first->FeedData(param.force_data->at(i).Fz);
			}
			
		}

		for (int i = 0; i < 18;i++)
		{
			//for VIII begin
			static double input2count = 9216000;
			this->torque[i] = ((double)param.last_motion_raw_data->at(i).feedback_cur)
				/ 1000.0 * 18 * 38.5 / 1000 * 2 * M_PI / (32.0 / 1000)*4.5;

			// for count 
			//this->position[i] = (double)param.last_motion_raw_data->at(i).feedback_pos / input2count;
			this->position[i] = (double)param.last_motion_raw_data->at(i).target_pos / input2count;
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
			
			legTraj[i].model->GetPee(legTraj[i].foot_position_ref_beginMak,beginMak);

			legTraj[i].model->GetPee(legTraj[i].foot_position_ref_body,robot.body());
			legTraj[i].model->GetJfd(*legTraj[i].force_jacobian_direct_ref_body,robot.body());

			legTraj[i].model->GetPee(legTraj[i].foot_position_ref_beginMak, beginMak);
			legTraj[i].model->GetJfd(*legTraj[i].force_jacobian_direct_ref_beginMak, beginMak);

			aris::dynamic::s_dgemm(3,1,3,-1.0,
				*legTraj[i].force_jacobian_direct_ref_body,3,
				legTraj[i].prismatic_external_force,1,
				0,legTraj[i].foot_force_extern_ref_body,1);

			aris::dynamic::s_dgemm(3, 1, 3, -1.0,
				*legTraj[i].force_jacobian_direct_ref_beginMak, 3,
				legTraj[i].prismatic_external_force, 1,
				0, legTraj[i].foot_force_extern_ref_beginMak, 1);
		}
		//if (param.count == 0)
		//{
		//	for (int j=0;j < 60;j++)
		//	{
		//		for (int i = 0;i < 6;i++)
		//		{
		//			//*********************************************
		//			// here may need more discussion: body or world coordinate
		//			//*********************************************
		//			fyFilterInd[i].FeedData(legTraj[i].foot_force_extern_ref_beginMak[1]);
		//			fzFilterInd[i].FeedData(legTraj[i].foot_force_extern_ref_beginMak[2]);
		//		}
		//	}

		//}
		//else
		//{
		//	for (int i = 0;i < 6;i++)
		//	{
		//		//*********************************************
		//		// here may need more discussion: body or world coordinate
		//		//*********************************************
		//		fyFilterInd[i].FeedData(legTraj[i].foot_force_extern_ref_beginMak[1]);
		//		fzFilterInd[i].FeedData(legTraj[i].foot_force_extern_ref_beginMak[2]);
		//	}
		//}
		
		if (param.count == 0 && forceMode == ForceMode::INDIRECT)
		{
			for (int j = 0;j < 60;j++)
			{
				for (int i = 0;i < 6;i++)
				{
					legTraj[i].yPosForceDetector.first->FeedData(legTraj[i].foot_force_extern_ref_beginMak[1]);
					legTraj[i].zPosForceDetector.first->FeedData(legTraj[i].foot_force_extern_ref_beginMak[2]);
					legTraj[i].zNegForceDetector.first->FeedData(legTraj[i].foot_force_extern_ref_beginMak[2]);
				}
			}
		}
		else
		{
			for (int i = 0;i < 6;i++)
			{
				legTraj[i].yPosForceDetector.first->FeedData(legTraj[i].foot_force_extern_ref_beginMak[1]);
				legTraj[i].zPosForceDetector.first->FeedData(legTraj[i].foot_force_extern_ref_beginMak[2]);
				legTraj[i].zNegForceDetector.first->FeedData(legTraj[i].foot_force_extern_ref_beginMak[2]);
			}

		}

		/* get threshold result */
		if (forceMode == INDIRECT || forceMode == SENSOR)
		{
			for (auto &i : legTraj)
			{
				i.yPosForceDetector.second->threshold(i.yPosForceDetector.first->GetData());
				i.zPosForceDetector.second->threshold(i.zPosForceDetector.first->GetData());
				i.zNegForceDetector.second->threshold(i.zNegForceDetector.first->GetData());
			}
		}
		
		
	};
	evalModel();

	if (currentMotion != motion)
	{
		for (auto &i : legTraj)
		{
			i.setStage(SequenceStage::INIT);
		}
	}
	else
	{
		// do nothing, other condition controls zero
	}
	
	/* we have a bunch of functions for init sequences  */
	/*   */
	auto initSequences = [&](HexapodSingleLeg & leg, MotionID mot)
	{
		/* set start  point*/
		leg._trajStartPoint << 
			leg.foot_position_ref_beginMak[0],
			leg.foot_position_ref_beginMak[1],
			leg.foot_position_ref_beginMak[2];

		switch (leg.currentSequence->_seqType)
		{
		case CurvesPlan::SequenceType::NS:
			double strH, ellH;
			double ellL = this->stepLength/2;
			switch (mot)
			{
			case TrajectoryGenerator::IDLE:
				break;
			case TrajectoryGenerator::STANDSTILL:
				break;
			case TrajectoryGenerator::FORWARD:
				double z_offset = leg.foot_position_ref_body[2] - leg._refPosition(2);
				double stepLengthActual = stepLength - z_offset;
				/* tricky part */
				strH = 0.05;
				ellH = 0.03;
				double H = strH + ellH;
				if (H + leg.foot_force_extern_ref_body[1] > leg._refSpaceY(1))
				{
					H = leg._refSpaceY(1) - leg.foot_force_extern_ref_body[1];
					if (H < ellH)
					{
						ellH = H;
						strH = 0;
					}
					else
					{
						strH = H - ellH;
					}
				}
				leg.normalSequence._strBoundUp._bound_mat << 0, 0, 0,
					0, strH, 0;
				leg.normalSequence._strBoundDown._bound_mat << stepLengthActual, strH, 0,
					stepLengthActual, leg._refSpaceY(0), 0;
				leg.normalSequence._ellMidBound._bound_mat << 0, strH, 0,
					stepLengthActual, strH, 0,
					stepLengthActual / 2, strH, 0;
				leg.normalSequence._ellMidBound._parameters << abs(stepLengthActual / 2.0), ellH, M_PI, 0;
				leg.normalSequence.reset();

				break;
			case TrajectoryGenerator::BACKWARD:
				
				break;
			case TrajectoryGenerator::TURNLEFT:
				break;
			case TrajectoryGenerator::TURNRIGHT:
				break;
			default:
				break;
			}

			break;
		case CurvesPlan::SequenceType::OS:
			double strH, ellH1, ellH2, ellL1, ellL2;

			switch (mot)
			{
			case TrajectoryGenerator::IDLE:
				break;
			case TrajectoryGenerator::STANDSTILL:
				break;
			case TrajectoryGenerator::FORWARD:
				/* tricky part */

				break;
			case TrajectoryGenerator::BACKWARD:
				break;
			case TrajectoryGenerator::TURNLEFT:
				break;
			case TrajectoryGenerator::TURNRIGHT:
				break;
			default:
				break;
			}

			break;
		case CurvesPlan::SequenceType::TS:
			// ell str
			double strH, ellH, ellL;
			if (leg.lastSequence->getCurrentSequenceType() == CurvesPlan::SequenceType::TS)
			{
				// move back
				
			}
			else if (leg.lastSequence->getCurrentSequenceType() == CurvesPlan::SequenceType::OS
				|| leg.lastSequence->getCurrentSequenceType() == CurvesPlan::SequenceType::NS)
			{
				// move forward

			}
			else
			{
				
			}
			break;
		case CurvesPlan::SequenceType::SS:
			leg.standstillSequence._stsBound._bound_mat << 0, 0, 0;
			break;
		default:
			break;
		}
	};

	// init the current sequences
	for (auto &i : legTraj)
	{
		if (i.getStage() == SequenceStage::INIT)
		{
			initSequences(i,motion);
		}
	}

	
	/* this part need to set INIT state */
	auto gaitTransition = [&](MotionID mot) 
	{
		switch (mot)
		{
		case TrajectoryGenerator::IDLE:
			break;
		case TrajectoryGenerator::STANDSTILL:
			break;
		case TrajectoryGenerator::FORWARD:
			break;
		case TrajectoryGenerator::BACKWARD:
			break;
		case TrajectoryGenerator::TURNLEFT:
			break;
		case TrajectoryGenerator::TURNRIGHT:
			break;
		default:
			break;
		}

	};
	gaitTransition(motion);
	for (auto &i : legTraj)
	{
		Eigen::Vector3d point=i._trajStartPoint + this->_rot2Bot //rotate
			*i.currentSequence->getPoint(
				(double)(param.count-i.currentSequence->getStartTime())
				/i.currentSequence->getTotalCounts());
		//assignment of target pee
		this->targetPee(i.getID(), 0) = point(0);
		this->targetPee(i.getID(), 1) = point(1);
		this->targetPee(i.getID(), 2) = point(2);
	}

	// TBD
	// clean the varibles, this should be put in the first place of this function
	auto gaitClean = [&]()
	{};
	gaitClean();

};