#pragma once
#include "Eigen/Dense"
#include <memory>
#include <iostream>
#include <array>
#include <vector>
#include <utility>
#include <aris.h>
#include "rtdk.h"
namespace CurvesPlan
{
	
	/* Time span */
	class TimeSpanBase
	{
	public:
		virtual double getRatio(double Tratio, double Tacc, double Tdec) = 0;
	};

	class TriangleAcc :public TimeSpanBase
	{
	public:
		virtual double getRatio(double Tratio, double Tacc, double Tdec)
		{
			double Pratio = 0;
			if (Tratio > 1.0)
			{
				Tratio = 1.0;
			}
			if (Tacc + Tdec > 1.001)
			{
				std::cout << "invalid parameter" << std::endl;
				return Pratio;
			}
			else
			{
				double Tcon = 1.0 - Tacc - Tdec;

                double Racc = Tacc*Tacc / Tdec/Tdec;

                double Ka = 1.0 / (1.0 / 3.0 * Tacc * Tacc * Tacc +
                    1.0 / 2.0 * Tacc *Tacc * (1.0 - Tacc - Tdec) +
                    1.0 / 3.0 * Tacc *Tacc * Tdec
					);
				double Kd = Ka*Racc;

                //rt_printf("Acc: %f,%f,%f\n",Ka,Tacc,Tratio);

				if (Tratio <= Tacc)
				{
                    Pratio = 1.0 / 2.0 * Ka*Tacc*Tratio*Tratio - 1.0 / 6.0 * Ka*Tratio*Tratio*Tratio;
				}
				else if (Tratio>Tacc + Tcon)
				{
                    Pratio = 1.0 / 3.0 * pow(Tacc, 3.0) * Ka
                        + 1.0 / 2.0 * pow(Tacc, 2.0) * (1 - Tacc - Tdec)*Ka
                        - (Ka*pow(Tacc, 2.0) * (-1.0 + Tratio + Tdec)*(1.0 + pow(Tratio, 2.0) + 2.0 * Tratio*(-1.0 + Tdec) - 2.0 * Tdec*(1.0 + Tdec))) / (6.0 * pow(Tdec, 2));
				}
				else
				{
                    Pratio = 1.0 / 2.0 * Ka*Tacc*pow(Tacc, 2.0) - 1.0 / 6.0 * Ka*pow(Tacc, 3.0)
                        + 1.0 / 2.0 * Ka*pow(Tacc, 2.0) * (Tratio - Tacc);
				}
				return Pratio;
			}
		}

	};

	class TrapezoidalVel :public TimeSpanBase
	{
	public:
		virtual double getRatio(double Tratio, double Tacc, double Tdec)
		{
			double Pratio = 0;
			if (Tratio > 1.0)
			{
				Tratio = 1.0;
			}
			if (Tacc + Tdec > 1.001)
			{
				std::cout << "invalid parameter" << std::endl;
				return Pratio;
			}
			else
			{
				double Tcon = 1.0 - Tacc - Tdec;
				double Vel = 1.0 / (1.0 - 0.5*(Tacc + Tdec));
				double Acc = Vel / Tacc;
				double Dec = Vel / Tdec;
				if (Tratio <= Tacc)
				{
					Pratio = 0.5*Acc*Tratio*Tratio;
				}
				else if (Tratio > Tacc + Tcon)
				{
					Pratio = 1.0 - 0.5*Vel / Tdec*pow((1.0 - Tratio), 2);
				}
				else
				{
					Pratio = 0.5*Acc*Tacc*Tacc + (Tratio - Tacc)*Vel;
				}
			}
		}
	};
	
	enum CurveType {STRAIGHT=1,ELLIPSE,CUBICSPLINE,STANDSTILL};

	/* Bounds for Curves */
	class BoundBase
	{
	public:
		CurveType _curveType;
		// true: positive, clockwise; false:negetive, counter-clockwise
		// mainly for ellipse
		bool _direction=true;
		
	public:
		virtual bool getDirection() { return this->_direction; };
		virtual void setDirection(bool b) { this->_direction = b; };
		virtual CurveType getCurveType() = 0;
		virtual Eigen::Vector3d getStartPoint() = 0;
		virtual Eigen::Vector3d getEndPoint() = 0;
	};
	class StandStillBound :public BoundBase
	{
	public:
		CurveType _curveType = STANDSTILL;
		Eigen::Vector3d _bound_mat;
	public:
		virtual CurveType getCurveType() { return _curveType; };
		virtual Eigen::Vector3d getStartPoint() { return _bound_mat; };
		virtual Eigen::Vector3d getEndPoint() { return _bound_mat; };

	};

	class StraightBound :public BoundBase
	{
	public:
		CurveType _curveType=STRAIGHT;
		Eigen::Matrix<double, 2, 3> _bound_mat;
	public:
		virtual CurveType getCurveType() { return _curveType; };

		virtual Eigen::Vector3d getStartPoint() {
			//std::cout << "StraightBound" << std::endl;
			return _bound_mat.row(0);
		};
		virtual Eigen::Vector3d getEndPoint() {
			//std::cout << "StraightBound" << std::endl;
			return _bound_mat.row(1);
		};

	};

	class EllipseBound : public BoundBase
	{
	public:
		CurveType _curveType = ELLIPSE;
		Eigen::Matrix<double, 3, 3> _bound_mat;// start point; end point, center point
		Eigen::Matrix<double, 4, 1> _parameters;// a, b , start theta, end theta
		
		double startParam;// designed for move back

	public:
		virtual CurveType getCurveType() { return _curveType; };

		virtual Eigen::Vector3d getStartPoint() {
//			std::cout << "EllipseBound" << std::endl;
			return _bound_mat.row(0);
		};
		virtual Eigen::Vector3d getEndPoint() {
//			std::cout << "EllipseBound" << std::endl;
			return _bound_mat.row(1);
		};


	};

	class CubicBound :public BoundBase
	{
	public:
		CurveType _curveType = CUBICSPLINE;
		// _bound_mat is for cubic spline
		Eigen::Matrix<double, 4, 3> _bound_mat;
		double _precision = 0.008;

	public:
		virtual CurveType getCurveType() { return _curveType; };

		virtual Eigen::Vector3d getStartPoint() {
//			std::cout << "CubicBound" << std::endl;
			return _bound_mat.row(0);
		};
		virtual Eigen::Vector3d getEndPoint() {
//			std::cout << "CubicBound" << std::endl;
			return _bound_mat.row(2);
		};

	};


	/* Curves */

	class CurveBase
	{
	public:
		virtual void setBound(BoundBase &bd) = 0;
		virtual Eigen::Vector3d getPoint(double t) = 0;
		//virtual Eigen::Vector3d getPoint(int t) = 0;
		virtual double getLength() = 0;
		virtual Eigen::Vector3d getDelta(double t_minus, double t_plus)
		{
			return this->getPoint(t_plus) - this->getPoint(t_minus);
		};
		virtual double getCurrentParameter() { return this->_base_u; };

	public:
		BoundBase* _base_refBound;
		double _base_u;//one parameter of the curve
	};
	class StandStill :public CurveBase
	{
	public:
		StandStillBound bound;
	public:
		virtual void setBound(BoundBase &bd)
		{
			auto tmp = static_cast<StandStillBound&>(bd);
			bound = tmp;
			_base_refBound = &bound;
		}
		virtual Eigen::Vector3d getPoint(double t) {
			_base_u = t;
			return bound._bound_mat; };
		virtual double getLength() { return 0; };


		

	};

	class Straight :public CurveBase
	{
	public:
		StraightBound bound;
	public:
		virtual void setBound(BoundBase &bd)
		{
			_base_refBound = &bound;
			auto tmp = static_cast<StraightBound&>(bd);
			bound = tmp;
//			std::cout << "Curve type straight " << bd.getCurveType() << "\t"
//				<< _base_refBound->getCurveType() << "\t" << bound.getCurveType()<<std::endl;
		}

		virtual Eigen::Vector3d getPoint(double t)
		{
			_base_u = t;
			return bound.getStartPoint()+(bound.getEndPoint() - bound.getStartPoint())*t;
		};

		virtual double getLength()
		{
			Eigen::Vector3d d = bound.getEndPoint() - bound.getStartPoint();
			return sqrt(d.squaredNorm());
		};

	};

	class Ellipse :public CurveBase
	{
	public:
		EllipseBound bound;
	public:
		virtual void setBound(BoundBase &bd)
		{
			_base_refBound = &bound;
			auto tmp = static_cast<EllipseBound&>(bd);
			bound = tmp;
			a = bound._parameters(0,0);
			b = bound._parameters(1,0);
			theta0 = bound._parameters(2,0);
			theta1 = bound._parameters(3,0);
		}
		
		virtual Eigen::Vector3d getPoint(double t)
		{
			/*
			using theta0 and theta1 determining the direction is enough
			for example clockwise pi->0
			counter clockwise 0->pi
			*/
			_base_u = t;
			double theta = theta0 + (theta1 - theta0)*t;
			Eigen::Vector3d origin,point;
			origin = bound._bound_mat.row(2);
			point(0) = origin(0) + cos(theta)*a;
			point(1) = origin(1) + sin(theta)*b;
//			std::cout << a << " " << b << std::endl;
			// third axis
			Eigen::Vector3d startPoint;
			startPoint = bound.getStartPoint();
			point(2) = startPoint(2);
			return point;
		};

		virtual double getLength()
		{

			/* Only special cases can be calculated 
				1, 1/2, 1/4 of a ellipse can be calculated 
			*/

			//calculate full at first

			double arc_length = 0;
//			std::cout << bound._parameters << std::endl;
			
			double c, d;
			if (bound._parameters(0) < bound._parameters(1))
			{
				c = bound._parameters(1);
				d = bound._parameters(0);
			}
			else
			{
				c = bound._parameters(0);
				d = bound._parameters(1);
			}
			arc_length = M_PI*(c + d)*(1 + 3 * pow(((c - d) / (c + d)) , 2 )/ (10 + sqrt(4 - 3 * pow(((c - d) / (c + d)) , 2))) + (4 / M_PI - 14 / 11)* pow( ((c - d) / (c + d)) , (14.233 + 13.981*pow(((c - d) / (c + d)) , 6.42)) ) );
//			std::cout << "arc_length " <<arc_length<<"a:"<<theta0<<"b:"<<theta1<<std::endl;

			double delta_theta = abs(theta1 - theta0);
//			std::cout << "delta theta " << delta_theta << std::endl;
			if (abs(delta_theta - 2 * M_PI) < 0.1)
			{
				/* full */
				return arc_length;
			}
			else if (abs(delta_theta - M_PI) < 0.1)
			{
				/* 1/2 */
				return arc_length*0.5;
			}
			else if (abs(delta_theta - 0.5*M_PI) < 0.1)
			{
				/* 1/4 */
				return arc_length*0.25;
			}
			else
			{
				/* this is a bug, maybe fixed after race */
				return 0.0;
			}
		};
	private:
		double a, b, theta0, theta1;
	};

	class CubicSpline :public CurveBase
	{
	public:
		CubicBound bound;
	public:
		virtual void setBound(BoundBase &bd)
		{
			_base_refBound = &bound;
			auto tmp = static_cast<CubicBound&>(bd);
			bound = tmp;
			_basis << 2, -2, 1, 1,
				-3, 3, -2, -1,
				0, 0, 1, 0,
				1, 0, 0, 0;
			control_mat = bound._bound_mat;
		}

		virtual Eigen::Vector3d getPoint(double t)
		{
			_base_u = t;
			param_vector << t*t*t, t*t, t, 1;
			Eigen::Vector3d point;
			point = control_mat.transpose()*_basis.transpose()*param_vector;
			//std::cout << control_mat << std::endl << std::endl<< _basis<<std::endl
			//	 << std::endl<<param_vector<<std::endl;
			return point;
		};

		virtual double getLength()
		{
			double increment = bound._precision;
			double u = 0.0;
			double sectionLength = 0.0;
			double length = 0.0;
			Eigen::Vector3d last_point,current_point,delta;
			last_point = this->getPoint(u);
			while (abs(u - 1.0) > increment)
			{
				u += increment;
				current_point = this->getPoint(u);
				
				delta = current_point - last_point;
				last_point = current_point;
				sectionLength = sqrt(delta.squaredNorm());
				length += sectionLength;
				//rt_printf("Spline getLength() %f\t%f\t%f\t%f\t%f\n"
				//	,u
				//	, length
				//,this->bound._bound_mat(1,0)
				//, this->bound._bound_mat(1, 1)
				//, this->bound._bound_mat(1, 2));
			}
			//rt_printf("Spline getLength() %f\n",length);
			return length;
		};

	private:
		Eigen::Matrix<double, 4, 4> _basis;
		Eigen::Vector4d param_vector;
		Eigen::Matrix<double, 4, 3> control_mat;

	};




	enum SequenceType { NS, OS, TS, SS, RS, CS };

	class CurvesSequenceBase
	{
	public:
		/* for memory allocation, run at very first. */
		
		virtual void init() = 0;
		/* Every time  start the gait */
		virtual void reset() = 0;
		
		//virtual Eigen::Vector3d getPoint(int t) = 0;

		//virtual Eigen::Vector3d getPoint(double t)=0;

		

		// recommand 
		virtual Eigen::Vector3d getTargetPoint(double time)
		{
			
			_currentCurveRatio = time;
            // here we do the mapping, choose between following 2
			double t = time;
			/* 0.5 0.5 cause error */
			//t = this->timeTrapezoidalVel.getRatio(time, 0.49, 0.49);
			t = this->timeTriangleAcc.getRatio(time, 0.49, 0.49);

			
			
            Eigen::Vector3d point;
//            rt_printf("getTargetPoint %d  %ull\n",this->_sequencesPair.size(),&this->_sequencesPair);
//                      //,this->_sequencesPair.front().first->_base_refBound->getCurveType());

//            point<<0,0,0;
//            return point;

//            if(this->_sequencesPair.size()>0)
//            {
//                rt_printf("sequencesPair size %d\n",this->_sequencesPair.size());
//            }
//            else
//            {
//                rt_printf("sequencesPair size %d\n",this->_sequencesPair.size());
//                point<<0,0,0;
//                return point;
//            }
			if (this->_sequencesPair.at(0).first->_base_refBound->getCurveType() == STANDSTILL)
			{
				_currentCurveType = STANDSTILL;
				return  this->_sequencesPair.at(0).first->getPoint(t);
			}

			for (unsigned int i = 0;i < this->_sequencesPair.size();i++)
			{
				if (t > this->_ratioSegment.at(i).first && t <= this->_ratioSegment.at(i).second)
				{
					_currentCurveType = this->_sequencesPair.at(i).first->_base_refBound->getCurveType();
					point = this->_sequencesPair.at(i).first->getPoint((t- this->_ratioSegment.at(i).first)/
						(this->_ratioSegment.at(i).second-this->_ratioSegment.at(i).first));
					break;
				}
				else
				{
					continue;
				}
			}
			if (t <= this->_ratioSegment.at(0).first)
			{
				point = this->_sequencesPair.at(0).first->getPoint(0);
			}
			else if(t>1.0)
			{
				point = this->_sequencesPair.back().first->getPoint(1);
			}

			return point;
		};



        virtual int getTotalCounts() { return this->_total_counts; }
        virtual double getTotalLength() { return this->_total_length; }
        virtual int getCurrentIndex() { return this->_currentCurveIndex; }
        virtual double getCurrentRatio() { return this->_currentCurveRatio; }
        virtual void setTotalCounts(int t) { this->_total_counts = t; }
        virtual CurveType getCurrentCurveType() { return _currentCurveType; }
        virtual void setStartTime(int t) { this->_startTime = t; }
        virtual int getStartTime() { return this->_startTime; }
        virtual void setCurrentSequenceCurve(SequenceType st) { _seqType = st; }
        virtual SequenceType getCurrentSequenceType() { return _seqType; }

		int _startTime;
		int _total_counts;
		double _total_length;
		int _currentCurveIndex;
		double _currentCurveRatio;
		double _overall_vel_ref = 0.0;// m/s used to estimate other sequences/s time
		CurveType _currentCurveType;
		SequenceType _seqType;
		
		std::vector<std::pair<double, double>> _ratioSegment;
		std::vector<std::pair<CurveBase*, BoundBase*>> _sequencesPair;

		/*time span*/
		TriangleAcc timeTriangleAcc;
		TrapezoidalVel timeTrapezoidalVel;

	};
	/* 
		There are there kind of sequences: 
		Normal: str-ell-str; 
		Obstacle: ell-ell-str; 
		Tentative: ell-str-ell-str 

		standstill is another sequence
	*/
	class StandStillSequence :public CurvesSequenceBase
	{
	public:
		StandStillSequence()
		{
			_seqType = SS;
			this->init();
			this->_sequencesPair.push_back(std::make_pair(&this->_stsCurve,&this->_stsBound));

		}
		virtual void init()
		{
			
		}
		virtual void reset()
		{
			this->_sequencesPair.at(0).first->setBound(*this->_sequencesPair.at(0).second);
            this->_stsBound._bound_mat<<0,0,0;
			this->_currentCurveRatio = 0;
			this->_currentCurveIndex = 0;
		}

        virtual void setTotalCounts(int t) { this->_total_counts = t; }

		virtual Eigen::Vector3d getPoint(double t)
		{
			return _stsCurve.getPoint(t);
		}

		virtual Eigen::Vector3d getPoint(int t)
		{
			return _stsCurve.getPoint(1.0);
		}


		StandStillBound _stsBound;
		StandStill _stsCurve;

		std::vector<CurveBase*> _curveSequences = std::vector<CurveBase*>(3);
		std::vector<BoundBase*> _curveBounds = std::vector<BoundBase*>(3);
        std::vector<std::pair<CurveBase*, BoundBase*>> _pairedSequence;
        //std::vector<std::pair<int*, double*>> _pairedSequence;
	};
	class NormalSequence :public CurvesSequenceBase
	{
	// Normal: str - ell - str
	public:
		NormalSequence()
		{
			_seqType = NS;
			this->init();
			this->_sequencesPair.push_back(std::make_pair(&this->_strLineUp,&this->_strBoundUp));
			this->_sequencesPair.push_back(std::make_pair(&this->_ellMid, &this->_ellMidBound));
			this->_sequencesPair.push_back(std::make_pair(&this->_strLineDown,&this->_strBoundDown));

            for (unsigned int i = 0;i < this->_countSequences.size();i++)
			{
				this->_ratioSegment.push_back(std::make_pair(0,0));
			}
		}
		virtual void init()
		{
			//this->_curveSequences = new CurveBase*[3];
			this->_curveSequences.at(0) = &this->_strLineUp;
			this->_curveSequences.at(1) = &this->_ellMid;
			this->_curveSequences.at(2) = &this->_strLineDown;
			this->_curveBounds.at(0) = &this->_strBoundUp;
			this->_curveBounds.at(1) = &this->_ellMidBound;
			this->_curveBounds.at(2) = &this->_strBoundDown;

			std::vector<CurveBase*>::iterator j_cur = this->_curveSequences.begin();
			std::vector<BoundBase*>::iterator j_bnd = this->_curveBounds.begin();
            _pairedSequence.resize(3);
            for (auto &i : _pairedSequence)
			{
				i.first = *j_cur;
				i.second = *j_bnd;
				j_cur++;
				j_bnd++;
			}

			this->_currentCurveIndex = 0;

			/* init settings for bounds */

			this->_strBoundUp._bound_mat << 0, 0, 0,
				0, 0.05, 0;
			this->_strBoundDown._bound_mat << 0.5, 0.05, 0,
				0.5, -0.1, 0;

			this->_ellMidBound._bound_mat << 0, 0.05, 0,
				0.5, 0.05, 0,
				0.25, 0.05, 0;
			this->_ellMidBound._parameters << 0.25, 0.05, M_PI, 0;
			

			/**
			this->_strBoundUp._bound_mat << 0, 0, 0,
				0, 0.05, 0;
			this->_strBoundDown._bound_mat << 0.5, 0.05, 0,
				0.5, -0.1, 0;

			this->_ellMidBound._bound_mat << 0, 0.05, 0,
				0.5, 0.05, 0,
				0.25, 0.05, 0;
			this->_ellMidBound._parameters << 0.25, 0.05, M_PI, 0;
			*/
			this->_total_counts = 5000;
			/* init settings finished */

		}
		virtual void reset()
		{
			/* don't forget set each bounds and total counts*/

			/* set all bounds */
			for (auto &i : _pairedSequence)
			{
				i.first->setBound(*i.second);
			}

			/* calculate curve length */
			_total_length=0.0;
			std::vector<double>::iterator j = _length.begin();
			for (auto &i : _pairedSequence)
			{
//				std::cout << i.first->getLength()<<" "<<(int)i.first->_base_refBound->getCurveType()<< std::endl;
				*j = i.first->getLength();
				_total_length += *j;
				j++;
			}
			this->_currentCurveRatio = 0;
			this->_currentCurveIndex = 0;
			///* redistribute time counts */
			//this->_countSequences.at(0) = (int)round(_length[0] / _total_length*(double)this->_total_counts);
			//this->_countSequences.at(1) = (int)round(_length[1] / _total_length*(double)this->_total_counts);
			//this->_countSequences.at(2) = this->_total_counts
			//	- this->_countSequences.at(0)
			//	- this->_countSequences.at(1);
			//this->_ratioSequences.at(0) = _length[0] / _total_length;
			//this->_ratioSequences.at(1) = _length[1] / _total_length;
			//this->_ratioSequences.at(2) = _length[2] / _total_length;

			//this->_ratioSegment.at(0).first = 0;
			//this->_ratioSegment.at(0).second = this->_ratioSequences.at(0);

			//this->_ratioSegment.at(1).first = this->_ratioSequences.at(0);
			//this->_ratioSegment.at(1).second = this->_ratioSequences.at(0)+ this->_ratioSequences.at(1);

			//this->_ratioSegment.at(2).first = this->_ratioSegment.at(1).second;
			//this->_ratioSegment.at(2).second = 1;


			//_overall_vel_ref = _total_length / (double)_total_counts*1000; // m/s

			

		}
		virtual void setTotalCounts(int t) 
		{ 
			this->_total_counts = t;

			/* redistribute time counts */
			this->_countSequences.at(0) = (int)round(_length[0] / _total_length*(double)this->_total_counts);
			this->_countSequences.at(1) = (int)round(_length[1] / _total_length*(double)this->_total_counts);
			this->_countSequences.at(2) = this->_total_counts
				- this->_countSequences.at(0)
				- this->_countSequences.at(1);
			this->_ratioSequences.at(0) = _length[0] / _total_length;
			this->_ratioSequences.at(1) = _length[1] / _total_length;
			this->_ratioSequences.at(2) = _length[2] / _total_length;

			this->_ratioSegment.at(0).first = 0;
			this->_ratioSegment.at(0).second = this->_ratioSequences.at(0);

			this->_ratioSegment.at(1).first = this->_ratioSequences.at(0);
			this->_ratioSegment.at(1).second = this->_ratioSequences.at(0) + this->_ratioSequences.at(1);

			this->_ratioSegment.at(2).first = this->_ratioSegment.at(1).second;
			this->_ratioSegment.at(2).second = 1;

			this->_currentCurveRatio = 0;
			_overall_vel_ref = _total_length / (double)_total_counts * 1000; // m/s

		};

		//virtual Eigen::Vector3d getPoint(int t)
		//{
		//	Eigen::Vector3d point;
		//	_currentCurveIndex = t;
		//	//std::cout <<"getPoint"<< point << std::endl;
		//	if (t <= _countSequences.at(0))
		//	{
		//		/*first*/
		//		point = _pairedSequence.at(0).first->getPoint((double)t / (double)_countSequences.at(0));
		//		//std::cout << point << std::endl;
		//	}
		//	else if (t> _countSequences.at(0) 
		//		&& t<=(_countSequences.at(0)+_countSequences.at(1)))
		//	{
		//		/* second */
		//		point = _pairedSequence.at(1).first->getPoint(
		//			(double)(t - _countSequences.at(0)) / (double)_countSequences.at(1));
		//	}
		//	else if (t>(_countSequences.at(0) + _countSequences.at(1)) 
		//		&& t<=(_countSequences.at(0)+ _countSequences.at(1)+ _countSequences.at(2)))
		//	{
		//		/* third */
		//		point = _pairedSequence.at(2).first->getPoint(
		//			(double)(t - _countSequences.at(0)-_countSequences.at(1)) / (double)_countSequences.at(2));
		//	}
		//	else if(t>(_countSequences.at(0) + _countSequences.at(1) + _countSequences.at(2)))
		//	{
		//		/* just return last point */
		//		point = _pairedSequence.at(2).first->getPoint(1.0);
		//	}
		//	else
		//	{
		//		/* error  return the origin points */
		//		std::cout << "negetive counts " << t << std::endl;
		//		point = _pairedSequence.at(0).first->getPoint(0.0);
		//	}
		//	return point;
		//}

		///* TBD */
		//virtual Eigen::Vector3d getPoint(double t)
		//{
		//	Eigen::Vector3d point;
		//	_currentCurveRatio = t;
		//	//std::cout <<"getPoint"<< point << std::endl;
		//	if (t <= _ratioSequences.at(0))
		//	{
		//		/*first*/
		//		point = _pairedSequence.at(0).first->getPoint((double)t / (double)_ratioSequences.at(0));
		//		//std::cout << point << std::endl;
		//	}
		//	else if (t> _ratioSequences.at(0)
		//		&& t <= (_ratioSequences.at(0) + _ratioSequences.at(1)))
		//	{
		//		/* second */
		//		point = _pairedSequence.at(1).first->getPoint(
		//			(double)(t - _ratioSequences.at(0)) / (double)_ratioSequences.at(1));
		//	}
		//	else if (t>(_ratioSequences.at(0) + _ratioSequences.at(1))
		//		&& t <= (_ratioSequences.at(0) + _ratioSequences.at(1) + _ratioSequences.at(2)))
		//	{
		//		/* third */
		//		point = _pairedSequence.at(2).first->getPoint(
		//			(double)(t - _ratioSequences.at(0) - _ratioSequences.at(1)) / (double)_ratioSequences.at(2));
		//	}
		//	else if (t>(_ratioSequences.at(0) + _ratioSequences.at(1) + _ratioSequences.at(2)))
		//	{
		//		/* just return last point */
		//		point = _pairedSequence.at(2).first->getPoint(1.0);
		//	}
		//	else
		//	{
		//		/* error  return the origin points */
		//		std::cout << "negetive ratio " << t << std::endl;
		//		point = _pairedSequence.at(0).first->getPoint(0.0);
		//	}
		//	return point;
		//}
		//
		/* these bound are set in trajectory generator
		   then, reset will calculate the time span
		*/
		StraightBound _strBoundUp;
		Straight _strLineUp;
		EllipseBound _ellMidBound;
		Ellipse _ellMid;
		StraightBound _strBoundDown;
		Straight _strLineDown;
		
		
		std::vector<double> _length= std::vector<double>(3);// _length of each curve
		double _overall_vel_ref = 0.0;// m/s used to estimate other sequences/s time

		/*Generate by reset*/
		std::vector<int> _countSequences=std::vector<int>(3);
		std::vector<double> _ratioSequences = std::vector<double>(3);

		std::vector<CurveBase*> _curveSequences= std::vector<CurveBase*>(3);
		std::vector<BoundBase*> _curveBounds = std::vector<BoundBase*>(3);
        std::vector<std::pair<CurveBase*, BoundBase*>> _pairedSequence;
	};

	class ObstacleSequence :public CurvesSequenceBase
	{
	// Obstacle: ell-ell-str
	public:
		ObstacleSequence()
		{
			_seqType = OS;
			this->init();
			this->_sequencesPair.push_back(std::make_pair(&this->_ellReflex,&this->_ellReflexBound));
			this->_sequencesPair.push_back(std::make_pair(&this->_ellForward, &this->_ellForwardBound));
			this->_sequencesPair.push_back(std::make_pair(&this->_strDown, &this->_strDownBound));

			for (unsigned int i = 0;i < this->_countSequences.size();i++)
			{
				this->_ratioSegment.push_back(std::make_pair(0, 0));
			}
		}
		virtual void init()
		{
			this->_curveSequences.at(0) = &this->_ellReflex;
			this->_curveSequences.at(1) = &this->_ellForward;
			this->_curveSequences.at(2) = &this->_strDown;
			this->_curveBounds.at(0) = &this->_ellReflexBound;
			this->_curveBounds.at(1) = &this->_ellForwardBound;
			this->_curveBounds.at(2) = &this->_strDownBound;

			std::vector<CurveBase*>::iterator j_cur = this->_curveSequences.begin();
			std::vector<BoundBase*>::iterator j_bnd = this->_curveBounds.begin();
            _pairedSequence.resize(3);
			for (auto &i : _pairedSequence)
			{
				i.first = *j_cur;
				i.second = *j_bnd;
				j_cur++;
				j_bnd++;
			}
			
			this->_currentCurveIndex = 0;
			
			/* init settings for bounds */

			this->_ellReflexBound._bound_mat << 0, 0, 0,
				0, 0.1, 0,
				0, 0.05, 0;
			this->_ellReflexBound._parameters <<0.03,0.05,-0.5*M_PI,-1.5*M_PI;

			this->_ellForwardBound._bound_mat.row(0) = this->_ellReflexBound.getEndPoint();
			//end point
			this->_ellForwardBound._bound_mat.row(1) << this->_ellForwardBound._bound_mat(0, 0) + 0.1,this->_ellForwardBound._bound_mat(0, 1) - 0.03,this->_ellForwardBound._bound_mat(0, 2);
			this->_ellForwardBound._bound_mat.row(2) << this->_ellReflexBound.getEndPoint()(0), this->_ellForwardBound._bound_mat(1,1), 0;

			this->_ellForwardBound._parameters << this->_ellForwardBound._bound_mat(1, 0)- this->_ellForwardBound._bound_mat(0, 0), this->_ellForwardBound._bound_mat(0, 1)-this->_ellForwardBound._bound_mat(1, 1),0.5*M_PI,0;
//			std::cout << this->_ellForwardBound._parameters << "ellForward" << std::endl;
			this->_strDownBound._bound_mat.row(0) = this->_ellForwardBound.getEndPoint();
			this->_strDownBound._bound_mat.row(1)= this->_ellForwardBound.getEndPoint();
			this->_strDownBound._bound_mat(1, 1) = -0.1;// must add reference points later

			this->_total_counts = 5000;
			/* init settings finished */


		}
		virtual void reset()
		{
			/* don't forget set each bounds and total counts*/

			/* set all bounds */
			for (auto &i : _pairedSequence)
			{
				i.first->setBound(*i.second);
			}

			/* calculate curve length */
			_total_length = 0.0;
			std::vector<double>::iterator j = _length.begin();

			for (auto &i : _pairedSequence)
			{
//				std::cout << i.first->getLength() << " " << (int)i.first->_base_refBound->getCurveType() << std::endl;
				*j = i.first->getLength();
				_total_length += *j;
				j++;
			}
			this->_currentCurveRatio = 0;
			this->_currentCurveIndex = 0;
			///* redistribute time counts */
			//this->_countSequences.at(0) = (int)round(_length[0] / _total_length*(double)this->_total_counts);
			//this->_countSequences.at(1) = (int)round(_length[1] / _total_length*(double)this->_total_counts);
			//this->_countSequences.at(2) = this->_total_counts
			//	- this->_countSequences.at(0)
			//	- this->_countSequences.at(1);
			//this->_ratioSequences.at(0) = _length[0] / _total_length;
			//this->_ratioSequences.at(1) = _length[1] / _total_length;
			//this->_ratioSequences.at(2) = _length[2] / _total_length;

			//this->_ratioSegment.at(0).first = 0;
			//this->_ratioSegment.at(0).second = this->_ratioSequences.at(0);

			//this->_ratioSegment.at(1).first = this->_ratioSequences.at(0);
			//this->_ratioSegment.at(1).second = this->_ratioSequences.at(0) + this->_ratioSequences.at(1);

			//this->_ratioSegment.at(2).first = this->_ratioSegment.at(1).second;
			//this->_ratioSegment.at(2).second = 1;

			//_overall_vel_ref = _total_length / (double)_total_counts * 1000; // m/s
		
		}
		virtual void setTotalCounts(int t)
		{
			this->_total_counts = t;

			/* redistribute time counts */
			this->_countSequences.at(0) = (int)round(_length[0] / _total_length*(double)this->_total_counts);
			this->_countSequences.at(1) = (int)round(_length[1] / _total_length*(double)this->_total_counts);
			this->_countSequences.at(2) = this->_total_counts
				- this->_countSequences.at(0)
				- this->_countSequences.at(1);
			this->_ratioSequences.at(0) = _length[0] / _total_length;
			this->_ratioSequences.at(1) = _length[1] / _total_length;
			this->_ratioSequences.at(2) = _length[2] / _total_length;

			this->_ratioSegment.at(0).first = 0;
			this->_ratioSegment.at(0).second = this->_ratioSequences.at(0);

			this->_ratioSegment.at(1).first = this->_ratioSequences.at(0);
			this->_ratioSegment.at(1).second = this->_ratioSequences.at(0) + this->_ratioSequences.at(1);

			this->_ratioSegment.at(2).first = this->_ratioSegment.at(1).second;
			this->_ratioSegment.at(2).second = 1;


			_overall_vel_ref = _total_length / (double)_total_counts * 1000; // m/s

		};

		//virtual Eigen::Vector3d getPoint(int t)
		//{
		//	Eigen::Vector3d point;
		//	_currentCurveIndex = t;
		//	//std::cout <<"getPoint"<< point << std::endl;
		//	if (t <= _countSequences.at(0))
		//	{
		//		/*first*/
		//		point = _pairedSequence.at(0).first->getPoint((double)t / (double)_countSequences.at(0));
		//		//std::cout << point << std::endl;
		//	}
		//	else if (t> _countSequences.at(0)
		//		&& t <= (_countSequences.at(0) + _countSequences.at(1)))
		//	{
		//		/* second */
		//		point = _pairedSequence.at(1).first->getPoint(
		//			(double)(t - _countSequences.at(0)) / (double)_countSequences.at(1));
		//	}
		//	else if (t>(_countSequences.at(0) + _countSequences.at(1))
		//		&& t <= (_countSequences.at(0) + _countSequences.at(1) + _countSequences.at(2)))
		//	{
		//		/* third */
		//		point = _pairedSequence.at(2).first->getPoint(
		//			(double)(t - _countSequences.at(0) - _countSequences.at(1)) / (double)_countSequences.at(2));
		//	}
		//	else if (t>(_countSequences.at(0) + _countSequences.at(1) + _countSequences.at(2)))
		//	{
		//		/* just return last point */
		//		point = _pairedSequence.at(2).first->getPoint(1.0);
		//	}
		//	else
		//	{
		//		/* error  return the origin points */
		//		std::cout << "negetive counts " << t << std::endl;
		//		point = _pairedSequence.at(0).first->getPoint(0.0);
		//	}
		//	return point;
		//}
		//virtual Eigen::Vector3d getPoint(double t)
		//{
		//	Eigen::Vector3d point;
		//	_currentCurveRatio = t;
		//	//std::cout <<"getPoint"<< point << std::endl;
		//	if (t <= _ratioSequences.at(0))
		//	{
		//		/*first*/
		//		point = _pairedSequence.at(0).first->getPoint((double)t / (double)_ratioSequences.at(0));
		//		//std::cout << point << std::endl;
		//	}
		//	else if (t> _ratioSequences.at(0)
		//		&& t <= (_ratioSequences.at(0) + _ratioSequences.at(1)))
		//	{
		//		/* second */
		//		point = _pairedSequence.at(1).first->getPoint(
		//			(double)(t - _ratioSequences.at(0)) / (double)_ratioSequences.at(1));
		//	}
		//	else if (t>(_ratioSequences.at(0) + _ratioSequences.at(1))
		//		&& t <= (_ratioSequences.at(0) + _ratioSequences.at(1) + _ratioSequences.at(2)))
		//	{
		//		/* third */
		//		point = _pairedSequence.at(2).first->getPoint(
		//			(double)(t - _ratioSequences.at(0) - _ratioSequences.at(1)) / (double)_ratioSequences.at(2));
		//	}
		//	else if (t>(_ratioSequences.at(0) + _ratioSequences.at(1) + _ratioSequences.at(2)))
		//	{
		//		/* just return last point */
		//		point = _pairedSequence.at(2).first->getPoint(1.0);
		//	}
		//	else
		//	{
		//		/* error  return the origin points */
		//		std::cout << "negetive ratio " << t << std::endl;
		//		point = _pairedSequence.at(0).first->getPoint(0.0);
		//	}
		//	return point;
		//}

		EllipseBound _ellReflexBound;
		Ellipse _ellReflex;
		EllipseBound _ellForwardBound;
		Ellipse _ellForward;
		StraightBound _strDownBound;
		Straight _strDown;

		std::vector<double> _length = std::vector<double>(3);// _length of each curve
		

									  /*Generate by reset*/
		std::vector<int> _countSequences = std::vector<int>(3);
		std::vector<double> _ratioSequences = std::vector<double>(3);

		std::vector<CurveBase*> _curveSequences = std::vector<CurveBase*>(3);
		std::vector<BoundBase*> _curveBounds = std::vector<BoundBase*>(3);
        std::vector<std::pair<CurveBase*, BoundBase*>> _pairedSequence;

	};
	class TentativeSequence :public CurvesSequenceBase
	{
	// Tentative: ell-str
	// two TentativeSequnences combination one tentative motion
	public:
		TentativeSequence()
		{
			_seqType = TS;
			this->init();
			this->_sequencesPair.push_back(std::make_pair(&this->_ellTentative, &this->_ellTentativeBound));
			this->_sequencesPair.push_back(std::make_pair(&this->_strDown, &this->_strDownBound));

			for (unsigned int i = 0;i < this->_countSequences.size();i++)
			{
				this->_ratioSegment.push_back(std::make_pair(0, 0));
			}
		}

		virtual void init()
		{
			this->_curveSequences.at(0) = &this->_ellTentative;
			this->_curveSequences.at(1) = &this->_strDown;
			
			this->_curveBounds.at(0) = &this->_ellTentativeBound;
			this->_curveBounds.at(1) = &this->_strDownBound;
			

			std::vector<CurveBase*>::iterator j_cur = this->_curveSequences.begin();
			std::vector<BoundBase*>::iterator j_bnd = this->_curveBounds.begin();
            _pairedSequence.resize(2);
			for (auto &i : _pairedSequence)
			{
				i.first = *j_cur;
				i.second = *j_bnd;
				j_cur++;
				j_bnd++;
			}

			/* this part should be used with reset() */

			this->_currentCurveIndex = 0;

			/* init settings for bounds */
			this->_ellTentativeBound._bound_mat << 0, 0, 0,
				0.05, 0, 0,
				0.025, 0, 0;
			this->_ellTentativeBound._parameters << 0.025, 0.02, M_PI, 0;

			this->_strDownBound._bound_mat << 0.05, 0, 0,
				0.05, -0.1, 0;
			this->_total_counts = 5000;
		}

		virtual void reset()
		{
			/* don't forget set each bounds and total counts*/

			/* set all bounds */
			for (auto &i : _pairedSequence)
			{
				i.first->setBound(*i.second);
			}

			/* calculate curve length */
			_total_length = 0.0;
			std::vector<double>::iterator j = _length.begin();
			for (auto &i : _pairedSequence)
			{
//				std::cout << i.first->getLength() << " " << (int)i.first->_base_refBound->getCurveType() << std::endl;
				*j = i.first->getLength();
				_total_length += *j;
				j++;
			}
			isReversed = false;
			this->_currentCurveRatio = 0;
			this->_currentCurveIndex = 0;
			///* redistribute time counts */
			//this->_countSequences.at(0) = (int)round(_length[0] / _total_length*(double)this->_total_counts);
			//this->_countSequences.at(1) = this->_total_counts - this->_countSequences.at(0);
			//
			//this->_ratioSequences.at(0) = _length[0] / _total_length;
			//this->_ratioSequences.at(1) = _length[1] / _total_length;

			//this->_ratioSegment.at(0).first = 0;
			//this->_ratioSegment.at(0).second= _length[1] / _total_length;

			//this->_ratioSegment.at(1).first = this->_ratioSegment.at(0).second;
			//this->_ratioSegment.at(1).second = 1.0;



			//_overall_vel_ref = _total_length / (double)_total_counts * 1000; // m/s

		}
		virtual void setTotalCounts(int t)
		{
            this->_total_counts=t;
			/* redistribute time counts */
			this->_countSequences.at(0) = (int)round(_length[0] / _total_length*(double)this->_total_counts);
			this->_countSequences.at(1) = this->_total_counts - this->_countSequences.at(0);

			this->_ratioSequences.at(0) = _length[0] / _total_length;
			this->_ratioSequences.at(1) = _length[1] / _total_length;

			this->_ratioSegment.at(0).first = 0;
            this->_ratioSegment.at(0).second = _length[0] / _total_length;

			this->_ratioSegment.at(1).first = this->_ratioSegment.at(0).second;
			this->_ratioSegment.at(1).second = 1.0;

			_overall_vel_ref = _total_length / (double)_total_counts * 1000; // m/s

		}

		//virtual Eigen::Vector3d getPoint(int t)
		//{
		//	Eigen::Vector3d point;
		//	_currentCurveIndex = t;
		//	//std::cout <<"getPoint"<< point << std::endl;
		//	if (t <= _countSequences.at(0))
		//	{
		//		/*first*/
		//		point = _pairedSequence.at(0).first->getPoint((double)t / (double)_countSequences.at(0));
		//		//std::cout << point << std::endl;
		//	}
		//	else if (t> _countSequences.at(0)
		//		&& t <= (_countSequences.at(0) + _countSequences.at(1)))
		//	{
		//		/* second */
		//		point = _pairedSequence.at(1).first->getPoint(
		//			(double)(t - _countSequences.at(0)) / (double)_countSequences.at(1));
		//	}
		//	else if (t>(_countSequences.at(0) + _countSequences.at(1)))
		//	{
		//		/* just return last point */
		//		point = _pairedSequence.at(1).first->getPoint(1.0);
		//	}
		//	else
		//	{
		//		/* error  return the origin points */
		//		std::cout << "negetive counts " << t << std::endl;
		//		point = _pairedSequence.front().first->getPoint(0.0);
		//	}
		//	return point;
		//}
		//virtual Eigen::Vector3d getPoint(double t)
		//{
		//	Eigen::Vector3d point;
		//	_currentCurveRatio = t;
		//	//std::cout <<"getPoint"<< point << std::endl;
		//	if (t <= _ratioSequences.at(0))
		//	{
		//		/*first*/
		//		point = _pairedSequence.at(0).first->getPoint((double)t / (double)_ratioSequences.at(0));
		//		//std::cout << point << std::endl;
		//	}
		//	else if (t> _ratioSequences.at(0)
		//		&& t <= (_ratioSequences.at(0) + _ratioSequences.at(1)))
		//	{
		//		/* second */
		//		point = _pairedSequence.at(1).first->getPoint(
		//			(double)(t - _ratioSequences.at(0)) / (double)_ratioSequences.at(1));
		//	}
		//	else if (t>(_ratioSequences.at(0) + _ratioSequences.at(1)))
		//	{
		//		/* just return last point */
		//		point = _pairedSequence.at(1).first->getPoint(1.0);
		//	}
		//	else
		//	{
		//		/* error  return the origin points */
		//		std::cout << "negetive ratio " << t << std::endl;
		//		point = _pairedSequence.front().first->getPoint(0.0);
		//	}
		//	return point;
		//}


		/* reverse is specially designed for this sequences */
		void reverse(double t,int startTime)
		{
			/* set new bounds */
			this->_ellTentativeBound._bound_mat.row(1) = this->_ellTentativeBound._bound_mat.row(0);
            this->_ellTentativeBound._bound_mat.row(0) = this->getTargetPoint(t);
			// row(2) is the origin, it should not be changed
			// then change the parameters a,b is the same
			// change start theta to end theta, then calculate the start theta
			if (t < this->_ratioSequences[0])
			{
				//double current_theta = this->_ellTentativeBound._parameters(2)
				//	+ (t / this->_ratioSequences[0]) // this t is not right, cause we have a mapping here
				//	*(this->_ellTentativeBound._parameters(3) - this->_ellTentativeBound._parameters(2));

				double current_theta = this->_ellTentativeBound._parameters(2)
					+ (this->timeTriangleAcc.getRatio(t, 0.49, 0.49) / this->_ratioSequences[0])
					*(this->_ellTentativeBound._parameters(3) - this->_ellTentativeBound._parameters(2));

				double previous_end_theta = this->_ellTentativeBound._parameters(3);

				this->_ellTentativeBound._parameters(3) = this->_ellTentativeBound._parameters(2);
				this->_ellTentativeBound._parameters(2) = current_theta;
				//this part is the init
				this->setStartTime(startTime);
				this->_currentCurveRatio = 0;
				this->_currentCurveIndex = 0;
			}
			else
			{
				std::cout << "Can not reverse" << std::endl;
			}

			
			//Eigen::Vector3d currentRad = this->_ellTentativeBound._bound_mat.row(0) - this->_ellTentativeBound._bound_mat.row(2);
			this->_strDownBound._bound_mat.row(0) = this->_ellTentativeBound._bound_mat.row(1);
			this->_strDownBound._bound_mat.row(1) = this->_ellTentativeBound._bound_mat.row(1);
			this->_strDownBound._bound_mat(1, 1) = -0.1;

			for (auto &i : _pairedSequence)
			{
				i.first->setBound(*i.second);
			}
			/* after set bound */
			this->reset();
			double vel = 0.05;
			this->setTotalCounts((int)round(this->getTotalLength()/vel*1000));

		}
		EllipseBound _ellTentativeBound;
		Ellipse _ellTentative;
		StraightBound _strDownBound;
		Straight _strDown;

		const static int sectionsNum = 2;

		std::vector<double> _length = std::vector<double>(sectionsNum);// _length of each curve
		double _overall_vel_ref = 0.0;// m/s used to estimate other sequences/s time

		std::vector<int> _countSequences = std::vector<int>(sectionsNum);
		std::vector<double> _ratioSequences = std::vector<double>(sectionsNum);

		std::vector<CurveBase*> _curveSequences = std::vector<CurveBase*>(sectionsNum);
		std::vector<BoundBase*> _curveBounds = std::vector<BoundBase*>(sectionsNum);
        std::vector<std::pair<CurveBase*, BoundBase*>> _pairedSequence;
		bool isReversed = false;
	};
	class RetractSequence :public NormalSequence
	{
	public:
		RetractSequence()
		{
			this->_seqType = RS;
			this->init();
		}
	};

	class OneSplineSequence:public CurvesSequenceBase
	{
	public:
		OneSplineSequence()
		{
			_seqType = CS;
			this->init();

			this->_sequencesPair.push_back(std::make_pair(&this->_cbcLine, &this->_cbcBound));
			std::cout << "OneSplineConstructor" << _sequencesPair.size()<<std::endl;
			for (unsigned int i = 0;i < this->_countSequences.size();i++)
			{
				this->_ratioSegment.push_back(std::make_pair(0, 0));
			}
		}

		virtual void init()
		{
			this->_curveSequences.at(0) = &this->_cbcLine;


			this->_curveBounds.at(0) = &this->_cbcBound;

			std::vector<CurveBase*>::iterator j_cur = this->_curveSequences.begin();
			std::vector<BoundBase*>::iterator j_bnd = this->_curveBounds.begin();

            _pairedSequence.resize(1);
			for (auto &i : _pairedSequence)
			{
				i.first = *j_cur;
				i.second = *j_bnd;
				j_cur++;
				j_bnd++;
			}

			/* this part should be used with reset() */
		}
		virtual void reset()
		{
			rt_printf("body pos reset: %d\t",_pairedSequence.size());
			for (auto &i : _pairedSequence)
			{
				i.first->setBound(*i.second);
			}

			/* calculate curve length */
			_total_length = 0.0;
			std::vector<double>::iterator j = _length.begin();
			for (auto &i : _pairedSequence)
			{
//				std::cout << i.first->getLength() << " " << (int)i.first->_base_refBound->getCurveType() << std::endl;
				*j = i.first->getLength();
				_total_length += *j;
				rt_printf("%f\n", _total_length);
				j++;
			}
			rt_printf("%f\n", _total_length);
			this->_currentCurveRatio = 0;
			this->_currentCurveIndex = 0;
		}
		
		//virtual Eigen::Vector3d getPoint(int t)
		//{
		//	this->getTargetPoint((double)t/(double)this->getTotalCounts());
		//};

		//virtual Eigen::Vector3d getPoint(double t)
		//{
		//	this->getTargetPoint(t);
		//};

		virtual void setTotalCounts(int t)
		{
			/* redistribute time counts */
			this->_total_counts = t;
			this->_countSequences.at(0) = (int)round(_length[0] / _total_length*(double)this->_total_counts);


			this->_ratioSequences.at(0) = 1.0;

			this->_ratioSegment.at(0).first = 0;
			this->_ratioSegment.at(0).second =1.0;

			rt_printf("body set total counts:\n%f %f\n",
				_length[0], _total_length);

			_overall_vel_ref = _total_length / (double)_total_counts * 1000; // m/s

		}

		CubicBound _cbcBound;
		CubicSpline _cbcLine;

		const static int sectionsNum = 1;

		std::vector<double> _length = std::vector<double>(sectionsNum);// _length of each curve
		double _overall_vel_ref = 0.0;// m/s used to estimate other sequences/s time

		std::vector<int> _countSequences = std::vector<int>(sectionsNum);
		std::vector<double> _ratioSequences = std::vector<double>(sectionsNum);

		std::vector<CurveBase*> _curveSequences = std::vector<CurveBase*>(sectionsNum);
		std::vector<BoundBase*> _curveBounds = std::vector<BoundBase*>(sectionsNum);
        std::vector<std::pair<CurveBase*, BoundBase*>> _pairedSequence;

	};

}
