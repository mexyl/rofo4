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
	CurvesPlan::CurvesSequenceBase *csb;
	csb = &ns;
	csb->init();
	csb->reset();

	std::ofstream fs("data.txt");
	std::cout << csb->getTotalCounts() <<"\t"<<csb->getTotalLength()<< std::endl;


	

	getchar();




    return 0;
}

