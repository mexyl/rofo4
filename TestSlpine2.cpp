// TestSlpine2.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "Curves.h"
#include <fstream>

int main()
{	
	CurvesPlan::StraightBound sb;
	sb._bound_mat <<0,0,0,
	                 0.5,0.2,0;
	CurvesPlan::CurveBase * cbase;
	CurvesPlan::Straight st;
	st.setBound(sb);
	cbase = &st;
	std::cout<<cbase->getDelta(0,1)<<std::endl<<cbase->_refBound->getCurveType()<<std::endl;

	/* test normal sequence */
	CurvesPlan::NormalSequence ns;
	CurvesPlan::ObstacleSequence os;
	CurvesPlan::CurvesSequenceBase *csb;
	csb = &ns;
	csb = &os;
	csb->init();
	std::cout << "init ok" << std::endl;
	csb->reset();
	std::cout << "reset ok" << std::endl;
	std::cout << csb->getTotalCounts() << "\t" << csb->getTotalLength() << std::endl;
	std::ofstream fs("data.txt");
	std::cout << csb->getTotalCounts() <<"\t"<<csb->getTotalLength()<< std::endl;
	//fs.close();
	//getchar();
	//return 0;
	int count = 0;
	Eigen::Vector3d point;
	while (count <= csb->getTotalCounts())
	{
		//std::cout << "running" << std::endl;
		point = csb->getPoint((double)count/ (double)csb->getTotalCounts());
		fs << point.transpose() << std::endl;
		count++;
	}

	fs.close();

	getchar();




    return 0;
}

