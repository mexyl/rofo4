// TestSlpine2.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "Curves.h"

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

	CurvesPlan::NormalSequence ns;
	ns.init();

	getchar();




    return 0;
}

