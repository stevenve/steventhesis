//#include <argos2/common/utility/logging/argos_log.h>
#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/utility/math/angles.h>

#define BEHAVIOR_NAME "[CBTFootbotMotionControl]: "

#include "bt_footbot_motion_control.h"

//#define DEBUG(message) { LOG << BEHAVIOR_NAME << message << std::endl; }
#define DEBUG(message)
//#define DEBUG_STATE() { LOG << BEHAVIOR_NAME << "[STATE] " << GetStateName() << std::endl; }
#define DEBUG_STATE()


//const UInt8 CBTFootbotMotionControl::MOTION_CONTROL_EROL = 0;
//const UInt8 CBTFootbotMotionControl::MOTION_CONTROL_CMC = 1;
//const UInt8 CBTFootbotMotionControl::MOTION_CONTROL_VMC = 2;

/****************************************/
/****************************************/

CBTFootbotMotionControl::CBTFootbotMotionControl(CCI_RobotData<CCI_FootBotState>* c_robot_data) :
CCI_Behavior<CCI_FootBotState> (c_robot_data, "bt_footbot_motion_control"){

	m_fLeftSpeed = 0.0;
	m_fRightSpeed = 0.0;
}

/****************************************/
/****************************************/

CBTFootbotMotionControl::~CBTFootbotMotionControl() {

}

/****************************************/
/****************************************/

void CBTFootbotMotionControl::Init(CCI_FootBotState& c_robot_state) {

}

/****************************************/
/****************************************/

void CBTFootbotMotionControl::Step(CCI_FootBotState& c_robot_state) {
	c_robot_state.SetWheelsLinearVelocity(m_fLeftSpeed, m_fRightSpeed);
}


/****************************************/
/****************************************/

void CBTFootbotMotionControl::ComputeSpeedFromForce(CVector2& cForce){

	/* Get the heading angle */
	CRadians cHeadingAngle = cForce.Angle().SignedNormalize();
	/* Get the length of the heading vector */
	Real fHeadingLength = cForce.Length();
	/* Clamp the speed so that it's not greater than MaxSpeed */
	Real fBaseSpeed = 5.0;

	Real fSpeed1, fSpeed2;
	/* Turning state switching conditions */
	if(Abs(cHeadingAngle) <= ToRadians(CDegrees(10))) {
		/* No Turn, heading angle very small */
		fSpeed1 = fBaseSpeed;
		fSpeed2 = fBaseSpeed;
	}
	else if(Abs(cHeadingAngle) > ToRadians(CDegrees(90))) {
		/* Hard Turn, heading angle very large */
		fSpeed1 = -fBaseSpeed;
		fSpeed2 =  fBaseSpeed;
	}
	else{
		/* Soft Turn, heading angle in between the two cases */
		Real fSpeedFactor = (ToRadians(CDegrees(90))- Abs(cHeadingAngle)) / ToRadians(CDegrees(90));
		fSpeed1 = fBaseSpeed - fBaseSpeed * (1.0 - fSpeedFactor);
		fSpeed2 = fBaseSpeed + fBaseSpeed * (1.0 - fSpeedFactor);
	}

	/* Apply the calculated speeds to the appropriate wheels */
	if(cHeadingAngle > CRadians::ZERO) {
		/* Turn Left */
		m_fLeftSpeed  = fSpeed1;
		m_fRightSpeed = fSpeed2;
	}
	else {
		/* Turn Right */
		m_fLeftSpeed  = fSpeed2;
		m_fRightSpeed = fSpeed1;
	}

}


/****************************************/
/****************************************/

void CBTFootbotMotionControl::GetComputedWheelsSpeed(Real* fLeftSpeed, Real* fRightSpeed) {
	*fLeftSpeed = m_fLeftSpeed;
	*fRightSpeed = m_fRightSpeed;
}

/****************************************/
/****************************************/

void CBTFootbotMotionControl::Destroy(CCI_FootBotState& c_robot_state) {

}

/****************************************/
/****************************************/

void CBTFootbotMotionControl::Reset(CCI_FootBotState& c_robot_state) {
	Destroy(c_robot_state);
	Init(c_robot_state);
}

/****************************************/
/****************************************/

