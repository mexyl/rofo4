#include <iostream>
#include <cstring>
#include <iomanip> 
#include <bitset>
#include <cstring>
#include <map>
#include <string>

using namespace std;

#include <stdlib.h>

#include <aris.h>
//#include <Aris_Core.h>
//#include <Aris_Message.h>
//#include <Aris_Control.h>
//#include <Robot_Server.h>
#include<Robot_Type_I.h>


#include "../rofo4.h"
#include "../RofoConfig.h"


using namespace aris::core;
using namespace Rofo;



int main(int argc, char *argv[])
{

    std::string xml_address;

    if (argc <= 1)
    {
        std::cout << "you did not type in robot name, in this case ROBOT-III will start" << std::endl;
//        xml_address = "/home/hex/code6/rofo3/resource/Robot_III.xml";
        xml_address = _XML_PATH;
        xml_address.append("Robot_III.xml");
    }
    else if (std::string(argv[1]) == "III")
    {
//        xml_address = "/home/hex/code6/rofo3/resource/Robot_III.xml";
        xml_address = _XML_PATH;
        xml_address.append("Robot_III.xml");
    }
    else if (std::string(argv[1]) == "VIII")
    {
        //xml_address = "/usr/Robots/resource/Robot_Type_I/Robot_VIII/Robot_VIII.xml";
        xml_address = _XML_PATH;
        xml_address.append("Robot_VIII.xml");
    }
    else
    {
        throw std::runtime_error("invalid robot name, please type in III or VIII");
    }

    auto &rs = aris::server::ControlServer::instance();
    rs.createModel<Robots::RobotTypeI>();
    rs.loadXml(xml_address.c_str());

    rs.addCmd("en", Robots::basicParse, nullptr);
    rs.addCmd("ds", Robots::basicParse, nullptr);
    rs.addCmd("hm", Robots::basicParse, nullptr);
    rs.addCmd("rc", Robots::recoverParse, Robots::recoverGait);
    rs.addCmd("wk", Robots::walkParse, Robots::walkGait);
    rs.addCmd("ro", Robots::resetOriginParse, Robots::resetOriginGait);

    rs.addCmd("climb",rofoParse,rofoGait);
    rs.addCmd("edcl", rofoEndParse,rofoEndGait);
    rs.addCmd("ay",ayParse,ayGait);

    //rs.addCmd("ad",Robots::adjust,parseAdjust);
    //rs.addCmd("aw",Robots::adjust,parseAdjust2Climb);
    //rs.addCmd("ay",adjustY,parseAdjustY);
    //rs.addCmd("wj",parseDig,Dig);

    //init rofo
    RofoWalkInit();

    rs.open();

    rs.setOnExit([&]()
    {
        aris::core::XmlDocument xml_doc;
        xml_doc.LoadFile(xml_address.c_str());
        auto model_xml_ele = xml_doc.RootElement()->FirstChildElement("Model");
        if (!model_xml_ele)throw std::runtime_error("can't find Model element in xml file");
        rs.model().saveXml(*model_xml_ele);

        aris::core::stopMsgLoop();
    });

	/**/
	std::cout<<"finished"<<std::endl;



    aris::core::runMsgLoop();

    robot_log.stop();

	return 0;
}
