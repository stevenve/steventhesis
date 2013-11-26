//#include <argos2/common/utility/logging/argos_log.h>
#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/utility/math/angles.h>

#define BEHAVIOR_NAME "[CBTFootbotOdometry]: "

#include "bt_footbot_odometry.h"

/****************************************/
/****************************************/

CBTFootbotOdometry::CBTFootbotOdometry(CCI_RobotData<CCI_FootBotState>* c_robot_data) :
	CCI_Behavior<CCI_FootBotState> (c_robot_data, "bt_footbot_odometry"),
	angle(0){
}

/****************************************/
/****************************************/

CBTFootbotOdometry::~CBTFootbotOdometry() {

}

/****************************************/
/****************************************/

void CBTFootbotOdometry::Init(CCI_FootBotState& c_robot_state) {
	angle = new CRadians(0);
}

/****************************************/
/****************************************/

void CBTFootbotOdometry::Step(CCI_FootBotState& c_robot_state) {


}

void CBTFootbotOdometry::Destroy(CCI_FootBotState& c_robot_state) {

}

/****************************************/
/****************************************/

void CBTFootbotOdometry::Reset(CCI_FootBotState& c_robot_state) {
	Destroy(c_robot_state);
	Init(c_robot_state);
}

/****************************************/
/****************************************/

