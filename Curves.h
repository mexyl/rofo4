#pragma once
#include "Eigen/Dense"
#include <memory>
#include <iostream>
namespace CurvesPlan
{
	/*

	*/
	
	enum CurveType {STRAIGHT=1,ELLIPSE,CUBICSPLINE};

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

	class StraightBound :public BoundBase
	{
	public:
		CurveType _curveType=STRAIGHT;
		Eigen::Matrix<double, 2, 3> _bound_mat;
	public:
		virtual CurveType getCurveType() { return _curveType; };

		virtual Eigen::Vector3d getStartPoint() {
			std::cout << "StraightBound" << std::endl;
			return _bound_mat.row(0);
		};
		virtual Eigen::Vector3d getEndPoint() {
			std::cout << "StraightBound" << std::endl;
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
			std::cout << "EllipseBound" << std::endl;
			return _bound_mat.row(0);
		};
		virtual Eigen::Vector3d getEndPoint() {
			std::cout << "EllipseBound" << std::endl;
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
			std::cout << "CubicBound" << std::endl; 
			return _bound_mat.row(0);
		};
		virtual Eigen::Vector3d getEndPoint() {
			std::cout << "CubicBound" << std::endl;
			return _bound_mat.row(2);
		};

	};


	/* Curves */

	class CurveBase
	{
	public:
		virtual void setBound(BoundBase &bd) = 0;
		virtual Eigen::Vector3d getPoint(double t) = 0;
		virtual double getLength() = 0;
		virtual Eigen::Vector3d getDelta(double t_minus, double t_plus)
		{
			return this->getPoint(t_plus) - this->getPoint(t_minus);
		};
		virtual double getCurrentParameter() { return this->_u; };

	public:
		BoundBase* _refBound;
		double _u;//one parameter of the curve
		
	};

	class Straight :public CurveBase
	{
	public:
		StraightBound bound;
	public:
		virtual void setBound(BoundBase &bd)
		{
			_refBound = &bound;
			auto tmp = static_cast<StraightBound&>(bd);
			bound = tmp;
		}

		virtual Eigen::Vector3d getPoint(double t)
		{
			_u = t;
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
			_refBound = &bound;
			auto tmp = static_cast<EllipseBound&>(bd);
			bound = tmp;
			a = bound._parameters(0);
			b = bound._parameters(1);
			theta0 = bound._parameters(2);
			theta1 = bound._parameters(3);
		}
		
		virtual Eigen::Vector3d getPoint(double t)
		{
			/*
			using theta0 and theta1 determining the direction is enough
			for example clockwise pi->0
			counter clockwise 0->pi
			*/
			double theta = theta0 + (theta1 - theta0)*t;
			Eigen::Vector3d origin,point;
			origin = bound._bound_mat.row(3);
			point(0) = origin(0) + cos(theta)*a;
			point(1) = origin(1) + sin(theta)*b;			
			return point;
		};

		virtual double getLength()
		{

			/* Only special cases can be calculated 
				1, 1/2, 1/4 of a ellipse can be calculated 
			*/

			//calculate full at first

			double arc_length = 0;

			arc_length = M_PI*(a + b)*(1 + 3 * pow(((a - b) / (a + b)) , 2 )/ (10 + sqrt(4 - 3 * pow(((a - b) / (a + b)) , 2))) + (4 / M_PI - 14 / 11)* pow( ((a - b) / (a + b)) , (14.233 + 13.981*pow(((a - b) / (a + b)) , 6.42)) ) );

			double delta_theta = abs(theta1 - theta0);
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
			_refBound = &bound;
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
			param_vector << t*t*t, t*t, t, 1;
			Eigen::Vector3d point;
			point = control_mat.transpose()*_basis.transpose()*param_vector;
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
			}
			return length;
		};

	private:
		Eigen::Matrix<double, 4, 4> _basis;
		Eigen::Vector4d param_vector;
		Eigen::Matrix<double, 4, 3> control_mat;

	};

	
	class TimeSpanBase
	{
	public:
		virtual double getRatio(double Tratio, double Tacc, double Tdec)=0;
	};

	class TriangleAcc :public TimeSpanBase
	{
	public:
		virtual double getRatio(double Tratio, double Tacc, double Tdec)
		{
			double Pratio = 0;
			if (Tacc + Tdec > 1.001)
			{
				std::cout << "invalid parameter" << std::endl;
				return Pratio;
			}
			else
			{
				double Tcon = 1.0 - Tacc - Tdec;
				double Racc = pow(Tacc, 2) / pow(Tdec, 2);
				double Ka = 1.0 / (1 / 3 * Tacc * Tacc * Tacc +
					1 / 2 * Tacc *Tacc * (1 - Tacc - Tdec) +
					1 / 3 * Tacc *Tacc * Tdec
					);
				double Kd = Ka*Racc;
				if (Tratio <= Tacc)
				{
					Pratio = 1 / 2 * Ka*Tacc*pow(Tratio, 2) - 1 / 6 * Ka*pow(Tratio, 3);
				}
				else if (Tratio>Tacc + Tcon)
				{
					Pratio = 1 / 3 * pow(Tacc, 3) * Ka
						+ 1 / 2 * pow(Tacc, 2) * (1 - Tacc - Tdec)*Ka
						- (Ka*pow(Tacc, 2) * (-1 + Tratio + Tdec)*(1 + pow(Tratio, 2) + 2 * Tratio*(-1 + Tdec) - 2 * Tdec*(1 + Tdec))) / (6 * pow(Tdec, 2));
				}
				else
				{
					Pratio = 1 / 2 * Ka*Tacc*pow(Tacc, 2) - 1 / 6 * Ka*pow(Tacc, 3)
						+ 1 / 2 * Ka*pow(Tacc, 2) * (Tratio - Tacc);
				}
				return Pratio;
			}
		}

	};

	class TrapezoidalVel :public TimeSpanBase
	{
		virtual double getRatio(double Tratio, double Tacc, double Tdec)
		{
			double Pratio = 0;
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
					Pratio = 1.0 - 0.5*Vel / Tdec*pow((1.0 - Tratio),2);
				}
				else
				{
					Pratio = 0.5*Acc*Tacc*Tacc + (Tratio - Tacc)*Vel;
				}
			}
		}
	};
}
