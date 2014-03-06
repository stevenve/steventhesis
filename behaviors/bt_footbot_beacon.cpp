//#include <argos2/common/utility/logging/argos_log.h>

#include <argos3/core/utility/math/angles.h>

#define BEHAVIOR_NAME "[CBTFootbotBeacon]: "

#include "bt_footbot_beacon.h"


/****************************************/
/****************************************/

CBTFootbotBeacon::CBTFootbotBeacon(CCI_RobotData<CCI_FootBotState>* c_robot_data) :
    CCI_Behavior<CCI_FootBotState> (c_robot_data, "bt_footbot_control_leds") {

}

/****************************************/
/****************************************/

CBTFootbotBeacon::~CBTFootbotBeacon() {
}

/****************************************/
/****************************************/

void CBTFootbotBeacon::Init(CCI_FootBotState& c_robot_state) {

}

/****************************************/
/****************************************/

void CBTFootbotBeacon::Step(CCI_FootBotState& c_robot_state) {

}



/****************************************/
/****************************************/

void CBTFootbotBeacon::Destroy(CCI_FootBotState& c_robot_state) {
}

/****************************************/
/****************************************/

void CBTFootbotBeacon::Reset(CCI_FootBotState& c_robot_state) {
    Destroy(c_robot_state);
    Init(c_robot_state);
}

/**
 * @brief Sets the color for the beacon
 * notice that the actual color will be set in the ApplyState() method
 *
 * @param c_color color to be set
 * @see CColor
 **/
void CBTFootbotBeacon::SetSingleLedColor(CCI_FootBotState& c_robot_state, const CColor& c_color)
{
	c_robot_state.setSetSingleLedColor(un_index, c_color);
}

/****************************************/
/****************************************/
