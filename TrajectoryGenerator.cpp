#include"TrajectoryGenerator.h"

peripherals::Log robot_log;

void TrajectoryGenerator::HexapodRofoGait::reset()
{
	_rot2Bot <<
		0, 0, 1,
		0, 1, 0,
		1, 0, 0;
	//  start
	// VIII
	double y = 0.85;
	legTraj[LF]._refPosition<<-0.3,y,-065;
	legTraj[LM]._refPosition<<-0.45,y,0;
	legTraj[LR]._refPosition<<-0.3,y,0.65;
	legTraj[RF]._refPosition<<0.3,y,-0.65;
	legTraj[RM]._refPosition<<0.45,y,0;
	legTraj[RR]._refPosition<<0.3,y,0.65;
	
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


    rt_printf("rofo reset\n");

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

int TrajectoryGenerator::HexapodRofoGait::generateRobotGait(Robots::RobotBase& rbt,MotionID motion, const Robots::WalkParam &param)
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

	// this happened at first time
	if (currentMotion != motion)
	{
		/*set start Sequnence*/
		legTraj.at(LF).currentSequence = &legTraj.at(LF).normalSequence;
		legTraj.at(RM).currentSequence = &legTraj.at(RM).normalSequence;
		legTraj.at(LR).currentSequence = &legTraj.at(LR).normalSequence;
		legTraj.at(RF).currentSequence = &legTraj.at(RF).standstillSequence;
		legTraj.at(LM).currentSequence = &legTraj.at(LM).standstillSequence;
		legTraj.at(RR).currentSequence = &legTraj.at(RR).standstillSequence;

		for (auto &i : legTraj)
		{
			i.setStage(SequenceStage::INIT);
		}
        currentMotion=motion;
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

        leg.setStage(SequenceStage::RUNNING);

		leg.currentSequence->setStartTime(param.count);

		switch (leg.currentSequence->_seqType)
		{
		case CurvesPlan::SequenceType::NS:
		{
			double strH, ellH;
			double ellL = this->stepLength / 2;
			switch (mot)
			{
			case TrajectoryGenerator::IDLE:
				break;
			case TrajectoryGenerator::STANDSTILL:
				break;
			case TrajectoryGenerator::FORWARD:
			{
				double z_offset = leg.foot_position_ref_body[2] - leg._refPosition(2);
				double stepLengthActual = stepLength - z_offset;
				/* tricky part */
				strH = 0.05;
				ellH = 0.03;
				double H = strH + ellH;
				if (H + leg.foot_position_ref_body[1] > leg._refSpaceY(1))
				{
					H = leg._refSpaceY(1) - leg.foot_position_ref_body[1];
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
					stepLengthActual, leg._refSpaceY(0)- leg.foot_position_ref_body[1], 0;
				leg.normalSequence._ellMidBound._bound_mat << 0, strH, 0,
					stepLengthActual, strH, 0,
					stepLengthActual / 2, strH, 0;
				leg.normalSequence._ellMidBound._parameters << abs(stepLengthActual / 2.0), ellH, M_PI, 0;

				leg.normalSequence.reset();

				// velocity =0.2 m/s
				leg.normalSequence.setTotalCounts((int)round((leg.normalSequence.getTotalLength() / 0.2) * 1000));

                rt_printf("ns init %d\n",leg.getID());
			}

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
		}
			break;
		case CurvesPlan::SequenceType::OS:
		{
			double ellH1, ellH2, ellL1, ellL2;
			switch (mot)
			{
			case TrajectoryGenerator::IDLE:
				break;
			case TrajectoryGenerator::STANDSTILL:

				break;
			case TrajectoryGenerator::FORWARD:
			{
				/* tricky part */
				double stepLengthIncrement = 0.5;//Parameters that can be tweaked
				ellH1 = 0.04;//Parameters that can be tweaked
				if (ellH1 * 2.0 + leg.foot_position_ref_body[1] > leg._refSpaceY(1))
				{
					ellH1 = (leg._refSpaceY(1) - leg.foot_force_extern_ref_body[1]) / 2.0;
				}
				ellL1 = 0.03;
				ellH2 = 0.03;
				ellL2 = stepLengthIncrement;
				leg.obstacleSquence._ellReflexBound._bound_mat << 0, 0, 0,
					0, 2 * ellH1, 0,
					0, ellH1, 0;
				leg.obstacleSquence._ellReflexBound._parameters << ellL1, ellH1, -0.5*M_PI, -1.5*M_PI;
				leg.obstacleSquence._ellForwardBound._bound_mat.row(0) << 0, 2 * ellH1, 0;
				leg.obstacleSquence._ellForwardBound._bound_mat.row(1) << ellL2, 2 * ellH1 - ellH2, 0;
				leg.obstacleSquence._ellForwardBound._bound_mat.row(2) << 0, 2 * ellH1 - ellH2, 0;
				leg.obstacleSquence._ellForwardBound._parameters << ellL2, ellH2, 0.5*M_PI, 0;
				leg.obstacleSquence._strDownBound._bound_mat.row(0) = leg.obstacleSquence._ellForwardBound.getEndPoint();
				leg.obstacleSquence._strDownBound._bound_mat.row(1) = leg.obstacleSquence._ellForwardBound.getEndPoint();
				leg.obstacleSquence._strDownBound._bound_mat(0) = leg._refSpaceY(0)- leg.foot_position_ref_body[1];
				leg.obstacleSquence.reset();

				//velocity =0.2 m/s
				leg.obstacleSquence.setTotalCounts((int)round((leg.obstacleSquence.getTotalLength() / 0.2) * 1000));
                rt_printf("os init %d\n",leg.getID());
			}
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

		}
			break;
		case CurvesPlan::SequenceType::TS:
		{
			// ell str
			double ellH, ellL;
			if (leg.lastSequence->getCurrentSequenceType() == CurvesPlan::SequenceType::TS)
			{
				// move back
				switch (mot)
				{
				case TrajectoryGenerator::IDLE:
					break;
				case TrajectoryGenerator::STANDSTILL:
					break;
				case TrajectoryGenerator::FORWARD:
				{
					/*tricky part TS -> TS*/
					/* lastPoint is not important, is can be omitted */
					//Eigen::Vector3d lastPoint = leg.lastSequence->getPoint(leg.lastSequence->getCurrentRatio());
					ellH = 0.03;
					if (leg.yPosForceDetector.second->is_on())
					{
						ellL = 0.025 / 2;
					}
					else
					{
						ellL = 0.025 * 2;
					}

					leg.tentativeSequence._ellTentativeBound._bound_mat << 0, 0, 0,
						ellL * 2, 0, 0,
						ellL, 0, 0;
					leg.tentativeSequence._ellTentativeBound._parameters << ellL, ellH, M_PI, 0;
					leg.tentativeSequence._strDownBound._bound_mat << ellL * 2, 0, 0,
						ellL * 2, leg._refSpaceY(0) - leg.foot_force_extern_ref_body[1], 0;

					leg.tentativeSequence.reset();
					// velocity is 0.2
					leg.tentativeSequence.setTotalCounts((int)round((leg.tentativeSequence.getTotalLength() / 0.2) * 1000));

                    rt_printf("ts init %d\n",leg.getID());
				}
				break;
					

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

			}
			else if (leg.lastSequence->getCurrentSequenceType() == CurvesPlan::SequenceType::OS
				|| leg.lastSequence->getCurrentSequenceType() == CurvesPlan::SequenceType::NS)
			{
				// move forward
				switch (mot)
				{
				case TrajectoryGenerator::IDLE:
					break;
				case TrajectoryGenerator::STANDSTILL:
					break;
				case TrajectoryGenerator::FORWARD:
				{
					/*tricky part NS,OS -> TS*/
					/* lastPoint is not important, is can be omitted */
					Eigen::Vector3d lastPoint = leg.lastSequence->getPoint(leg.lastSequence->getCurrentRatio());
					ellH = 0.03;
					ellL = 0.025;
					leg.tentativeSequence._ellTentativeBound._bound_mat << 0, 0, 0,
						ellL * 2, 0, 0,
						ellL, 0, 0;
					leg.tentativeSequence._ellTentativeBound._parameters << ellL, ellH, M_PI, 0;
					leg.tentativeSequence._strDownBound._bound_mat << ellL * 2, 0, 0,
						ellL * 2, leg._refSpaceY(0) - leg.foot_force_extern_ref_body[1], 0;

					leg.tentativeSequence.reset();
					// velocity is 0.2
					leg.tentativeSequence.setTotalCounts((int)round((leg.tentativeSequence.getTotalLength() / 0.2) * 1000));

                    rt_printf("ts init %d\n",leg.getID());
				}
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

			}
			else
			{
				//undefined state

			}

		}
			break;
		case CurvesPlan::SequenceType::SS:
			leg.standstillSequence._stsBound._bound_mat << 0, 0, 0;
			break;
		case CurvesPlan::SequenceType::RS:
		{
			/* retract sequence */
			/* only one condition we need this */
			if (leg.tentativeCounts == 1 
				&& !leg.yPosForceDetector.second->is_on()
				&& leg.lastSequence->getCurrentSequenceType()== CurvesPlan::SequenceType::TS)
			{
				double strHup = leg.tentativeSequence._strDownBound._bound_mat(0, 1) 
					- leg.tentativeSequence._strDownBound._bound_mat(1, 1);
				double ellL = leg.tentativeSequence._strDownBound._bound_mat(0, 0);
				ellL = (ellL + 0.05) / 2.0;
				double ellH = leg.tentativeSequence._ellTentativeBound._parameters(1);
				double strHdn = strHup;

				/* after get corrent value, set bounds */
				leg.retractSequence._strBoundUp._bound_mat << 0, 0, 0,
					0, strHup, 0;
				leg.retractSequence._strBoundDown._bound_mat << -2 * ellL, strHdn, 0,
					-2 * ellL, 0, 0;

				leg.retractSequence._ellMidBound._bound_mat << 0, strHup, 0,
					-2 * ellL, strHdn, 0,
					-ellL, ellH, 0;
				leg.retractSequence._ellMidBound._parameters << ellL, ellH, 0, M_PI;

				leg.retractSequence.reset();
				// velocity is 0.2
				leg.retractSequence.setTotalCounts((int)round((leg.retractSequence.getTotalLength() / 0.2) * 1000));

                rt_printf("ss init %d\n",leg.getID());
			}
			else
			{
				std::cout << "Insufficient condition fot RetractSequence initialization. legID:"<<leg.getID()<< std::endl;
			}

		}
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
			for (auto &i:legTraj)
			{
				switch (i.currentSequence->getCurrentSequenceType())
				{
				case CurvesPlan::SequenceType::NS:
				{
					if (i.yPosForceDetector.second->is_on())
					{
						// step on something
						// change to TS
						if (this->isTentative)
						{
							i.lastSequence = &i.normalSequence;
							i.currentSequence = &i.tentativeSequence;
							i.setStage(SequenceStage::INIT);
							i.tentativeCounts = 1;
						}
						else
						{
							i.currentSequence = &i.standstillSequence;
							i.setStage(SequenceStage::INIT);
						}
						

					}
					else if (i.zPosForceDetector.second->is_on())
					{
						// obstacle
						// change to OS
						i.lastSequence = &i.normalSequence;
						i.currentSequence =&i.obstacleSquence;
						i.setStage(SequenceStage::INIT);

					}
					/* here should add an retract */
					else if((param.count-i.currentSequence->getStartTime())>=i.currentSequence->getTotalCounts())
					{
						/* finished */
						// should not happen
						
					}
				}
				break;
				case CurvesPlan::SequenceType::OS:
				{
					if (i.yPosForceDetector.second->is_on())
					{
						// step on something
						// change to TS
						
						if (this->isTentative)
						{
							i.lastSequence = &i.obstacleSquence;
							i.currentSequence = &i.tentativeSequence;
							i.setStage(SequenceStage::INIT);
							i.tentativeCounts = 1;
						}
						else
						{
							i.lastSequence = &i.obstacleSquence;
							i.currentSequence = &i.standstillSequence;
							i.setStage(SequenceStage::INIT);
						}

					}
					else if (i.zPosForceDetector.second->is_on())
					{
						// obstacle
						// change to OS
						i.lastSequence = &i.obstacleSquence;
						i.currentSequence = &i.obstacleSquence;
						i.setStage(SequenceStage::INIT);

					}
					else if ((param.count - i.currentSequence->getStartTime()) >= i.currentSequence->getTotalCounts())
					{
						/*finished*/
						//should not happen

					}
				}
				break;
				case CurvesPlan::SequenceType::TS: // gait transistion is isTentative is off, then the program will never come here
				{
					if (i.yPosForceDetector.second->is_on())
					{
						// step on something
						if (i.tentativeCounts == 1)
						{
							// TS
							i.lastSequence = &i.tentativeSequence;
							i.currentSequence = &i.tentativeSequence;
							i.setStage(SequenceStage::INIT);
							i.tentativeCounts = 2;
						}
						else if (i.tentativeCounts == 2)
						{
							//SS
							i.lastSequence = &i.tentativeSequence;
							i.currentSequence = &i.standstillSequence;
							i.setStage(SequenceStage::INIT);
							i.tentativeCounts = 0;

						}
					}
					else if (i.zPosForceDetector.second->is_on())
					{
						// reverse
						// add time protect TBD 
						i.tentativeSequence.reverse(i.currentSequence->getCurrentRatio());
						i.tentativeSequence.setStartTime(param.count);

					}
					else if ((param.count - i.currentSequence->getStartTime()) >= i.currentSequence->getTotalCounts())
					{
						if (i.tentativeCounts == 1 && !i.yPosForceDetector.second->is_on())
						{
							// can not reverse, but here need a new sequences
							i.lastSequence = &i.tentativeSequence;
							i.currentSequence = &i.retractSequence;
							i.setStage(SequenceStage::INIT);
						}
						else
						{
							//should never come here
						}
					}
				}
				break;
				case CurvesPlan::SequenceType::SS:
				{}
				break;
				default:
					break;
				}
			}
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

        if(i.getID()==LF && param.count%2000==0)
        {
            std::cout<<point<<std::endl;
            std::cout<<i.currentSequence->getPoint(
                           (double)(param.count-i.currentSequence->getStartTime())
                           /i.currentSequence->getTotalCounts())<<std::endl;

        }
	}


    if(param.count%2000==0)
    {
        for(int i=0;i<6;i++)
        {
          rt_printf("%f\t%f\t%f\n",targetPee(i,0),targetPee(i,1),targetPee(i,2));
        }
        rt_printf("\n");
    }

	// TBD
	// clean the varibles, this should be put in the first place of this function
	auto gaitClean = [&]()
	{};
	gaitClean();

	auto isAllStandStill = [&]() 
	{
		int allss = 0;
		for (auto &i : legTraj)
		{
			if (i.currentSequence->getCurrentSequenceType() == CurvesPlan::SequenceType::SS)
			{
				allss++;
			}
		}
		return allss;
	};

	if (isAllStandStill() == 6)
	{
        currentMotion=IDLE;
		return 0;
	}
	else
	{
		return 1000;
	}
	
};
