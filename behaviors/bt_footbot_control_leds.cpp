//#include <argos2/common/utility/logging/argos_log.h>

#include <argos3/core/utility/math/angles.h>

#define BEHAVIOR_NAME "[CBTFootbotControlLeds]: "

#include "bt_footbot_control_leds.h"


/****************************************/
/****************************************/

CBTFootbotControlLeds::CBTFootbotControlLeds(CCI_RobotData<CCI_FootBotState>* c_robot_data) :
    CCI_Behavior<CCI_FootBotState> (c_robot_data, "bt_footbot_control_leds") {

}

/****************************************/
/****************************************/

CBTFootbotControlLeds::~CBTFootbotControlLeds() {
}

/****************************************/
/****************************************/

void CBTFootbotControlLeds::Init(CCI_FootBotState& c_robot_state) {

}

/****************************************/
/****************************************/

void CBTFootbotControlLeds::Step(CCI_FootBotState& c_robot_state) {

}



/****************************************/
/****************************************/

void CBTFootbotControlLeds::Destroy(CCI_FootBotState& c_robot_state) {
}

/****************************************/
/****************************************/

void CBTFootbotControlLeds::Reset(CCI_FootBotState& c_robot_state) {
    Destroy(c_robot_state);
    Init(c_robot_state);
}

/**
 *
 * @brief Sets the color for the specified LED
 * notice that the actual color will be set in the ApplyState() method
 *
 * @param un_index index of the LED to be set
 * @param c_color color to be set
 * @see CColor
 *
 **/
void CBTFootbotControlLeds::SetSingleLedColor(CCI_FootBotState& c_robot_state, UInt8 un_index, const CColor& c_color)
{
	c_robot_state.SetSingleLedColor(un_index, c_color);
}

/**
 *
 * @brief Sets the color for all the LEDs
 * notice that the actual color will be set in the ApplyState() method
 *
 * @param c_color color to be set
 * @see CColor
 *
 **/
void CBTFootbotControlLeds::SetAllLedsColor(CCI_FootBotState& c_robot_state, const CColor& c_color)
{
	c_robot_state.SetAllLedsColor(c_color);
}

/****************************************/
/****************************************/
