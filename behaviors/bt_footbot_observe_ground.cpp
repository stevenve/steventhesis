//#include <argos2/common/utility/logging/argos_log.h>
#include <argos3/core/utility/math/angles.h>

#define BEHAVIOR_NAME "[CBTFootbotObserveGround]: "

#include "bt_footbot_observe_ground.h"



/****************************************/
/****************************************/

CBTFootbotObserveGround::CBTFootbotObserveGround(CCI_RobotData<CCI_FootBotState>* c_robot_data) :
    CCI_Behavior<CCI_FootBotState> (c_robot_data, "bt_footbot_observer_ground") {
    m_color = BLACK;
    tReadings.clear();
}

/****************************************/
/****************************************/

CBTFootbotObserveGround::~CBTFootbotObserveGround() {
}

/****************************************/
/****************************************/

void CBTFootbotObserveGround::Init(CCI_FootBotState& c_robot_state) {

}

/****************************************/
/****************************************/

void CBTFootbotObserveGround::Step(CCI_FootBotState& c_robot_state) {
	tReadings = c_robot_state.GetMotorGroundSensorReadings();
}

/****************************************/
/****************************************/

void CBTFootbotObserveGround::Destroy(CCI_FootBotState& c_robot_state) {
}

/****************************************/
/****************************************/

void CBTFootbotObserveGround::Reset(CCI_FootBotState& c_robot_state) {
    Destroy(c_robot_state);
    Init(c_robot_state);
}

/****************************************/
/****************************************/

bool CBTFootbotObserveGround::OnColor(){
	switch(m_color) {


	      case BLACK: {
	    	  if(tReadings[2].Value < 0.25f && tReadings[3].Value < 0.25f) {
	    		  return true;
	    	  }
	         break;
	      }

	      case GRAY: {
	    	  if(tReadings[2].Value > 0.25f &&
	    			  tReadings[2].Value < 0.75f &&
	    			  tReadings[3].Value > 0.25f &&
	    			  tReadings[3].Value < 0.75f) {
	    	  	      return true;
	    	  	   }
	         break;
	      }
	      case WHITE: {
	      	if(tReadings[2].Value > 0.75f &&tReadings[3].Value > 0.75f) {
	      		return true;
	      	}
	      	break;
	     }
	   }
	return false;
}

/*std::vector<SReading> CBTFootbotObserveGround::GetReadings(){
	return tReadings;
}*/
