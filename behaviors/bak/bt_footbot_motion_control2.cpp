
#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/utility/math/angles.h>
#include <argos3/core/utility/configuration/argos_configuration.h>

#define BEHAVIOR_NAME "[CBTFootbotMotionControl]: "

#include "bt_footbot_motion_control.h"


/****************************************/
/****************************************/

CBTFootbotMotionControl::CBTFootbotMotionControl(CCI_RobotData<CCI_FootBotState>* c_robot_data) :
	CCI_Behavior<CCI_FootBotState> (c_robot_data, "bt_footbot_motion_control"),
	   m_cAlpha(5.0f),
	   m_fDelta(0.01f),
	   m_fWheelVelocity(5.0f),
	   m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
	                           ToRadians(m_cAlpha)) {}

/****************************************/
/****************************************/

CBTFootbotMotionControl::~CBTFootbotMotionControl() {

}

/****************************************/
/****************************************/

void CBTFootbotMotionControl::Init(CCI_FootBotState& c_robot_state) {

	m_sWheelTurningParams.Init();

	    /* reset all */
	    //Reset(c_robot_state);
}

/****************************************/
/****************************************/

void CBTFootbotMotionControl::Step(CCI_FootBotState& c_robot_state) {
	c_robot_state.SetWheelsLinearVelocity(fLeftWheelSpeed +2.0f, fRightWheelSpeed +2.0f);
}

/****************************************/
/****************************************/

void CBTFootbotMotionControl::ComputeSpeedFromForce(CVector2& c_heading){

	/* Get the heading angle */
	   CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
	   /* Get the length of the heading vector */
	   Real fHeadingLength = c_heading.Length();
	   /* Clamp the speed so that it's not greater than MaxSpeed */
	   Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);

	   /* Turning state switching conditions */
	   if(Abs(cHeadingAngle) <= m_sWheelTurningParams.NoTurnAngleThreshold) {
	      /* No Turn, heading angle very small */
	      m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
	   }
	   else if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
	      /* Hard Turn, heading angle very large */
	      m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
	   }
	   else if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::NO_TURN &&
	           Abs(cHeadingAngle) > m_sWheelTurningParams.SoftTurnOnAngleThreshold) {
	      /* Soft Turn, heading angle in between the two cases */
	      m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
	   }

	   /* Wheel speeds based on current turning state */
	   Real fSpeed1, fSpeed2;
	   switch(m_sWheelTurningParams.TurningMechanism) {
	      case SWheelTurningParams::NO_TURN: {
	         /* Just go straight */
	         fSpeed1 = fBaseAngularWheelSpeed;
	         fSpeed2 = fBaseAngularWheelSpeed;
	         break;
	      }

	      case SWheelTurningParams::SOFT_TURN: {
	         /* Both wheels go straight, but one is faster than the other */
	         Real fSpeedFactor = (m_sWheelTurningParams.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) / m_sWheelTurningParams.HardTurnOnAngleThreshold;
	         fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
	         fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
	         break;
	      }

	      default: case SWheelTurningParams::HARD_TURN: {
	         /* Opposite wheel speeds */
	         fSpeed1 = -m_sWheelTurningParams.MaxSpeed;
	         fSpeed2 =  m_sWheelTurningParams.MaxSpeed;
	         break;
	      }
	   }

	   /* Apply the calculated speeds to the appropriate wheels */
	   if(cHeadingAngle > CRadians::ZERO) {
	      /* Turn Left */
	      fLeftWheelSpeed  = fSpeed1;
	      fRightWheelSpeed = fSpeed2;
	   }
	   else {
	      /* Turn Right */
	      fLeftWheelSpeed  = fSpeed2;
	      fRightWheelSpeed = fSpeed1;
	   }
}


/****************************************/
/****************************************/

void CBTFootbotMotionControl::GetComputedWheelsSpeed(Real* m_fLeftSpeed, Real* m_fRightSpeed) {
	*m_fLeftSpeed = fLeftWheelSpeed;
	*m_fRightSpeed = fRightWheelSpeed;
}



void CBTFootbotMotionControl::Destroy(CCI_FootBotState& c_robot_state) {
	//delete m_pcRNG;
}

/****************************************/
/****************************************/

void CBTFootbotMotionControl::Reset(CCI_FootBotState& c_robot_state) {
	Destroy(c_robot_state);
	Init(c_robot_state);
}

void CBTFootbotMotionControl::SWheelTurningParams::Init() {
      CDegrees cAngle;
      cAngle.SetValue(90);
      HardTurnOnAngleThreshold = ToRadians(cAngle);
      cAngle.SetValue(70);
      SoftTurnOnAngleThreshold = ToRadians(cAngle);
      cAngle.SetValue(10);
      NoTurnAngleThreshold = ToRadians(cAngle);
      MaxSpeed = 25.0f;
}

/****************************************/
/****************************************/

