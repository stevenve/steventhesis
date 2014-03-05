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
	CCI_Behavior<CCI_FootBotState> (c_robot_data, "bt_footbot_motion_control"),
	m_cRandomNoiseRange(-0.0000001, 0.0000001) {
	//m_pcRNG = CRandom::CreateRNG("argos");
	m_fLeftSpeed = 0.0;
	m_fRightSpeed = 0.0;

	m_fWheelsDistance = 14.0;

	m_fBiasForwardSpeed = 5.0;

	m_fForwardSpeed = 0.001;
	m_fAngularSpeed = 0.0;
	m_fLinearKVMC = 25.0;
	m_fAngularKVMC = 0.5;
	m_fForwardSaturation = 25.0;
	//m_fForwardSaturation = 0.5;
	m_fAngularSaturation = CRadians::PI_OVER_TWO.GetValue();

	m_bSaturateSpeed = true;
	m_bForwardOnly = true;

	m_fNoiseFactor = 0.0000001;
	m_cRandomNoiseRange.Set(-m_fNoiseFactor * CRadians::PI.GetValue(), m_fNoiseFactor * CRadians::PI.GetValue());
}

/****************************************/
/****************************************/

CBTFootbotMotionControl::~CBTFootbotMotionControl() {

}

/****************************************/
/****************************************/

void CBTFootbotMotionControl::Init(CCI_FootBotState& c_robot_state) {
	DEBUG("Init()");
	/* create random generator */
	//m_pcRNG = CRandom::CreateRNG("argos");
}

/****************************************/
/****************************************/

void CBTFootbotMotionControl::Step(CCI_FootBotState& c_robot_state) {

	c_robot_state.SetWheelsLinearVelocity(m_fLeftSpeed, m_fRightSpeed);
}

/****************************************/
/****************************************/

Real CBTFootbotMotionControl::SaturateSpeed(Real f_original_speed, Real f_saturation_value) {
	Real fResult = f_original_speed;
	if (f_original_speed > f_saturation_value) {
		fResult = f_saturation_value;
	}
	if (f_original_speed < -f_saturation_value) {
		fResult = -f_saturation_value;
	}
	return fResult;
}

/****************************************/
/****************************************/

void CBTFootbotMotionControl::ComputeSpeedFromForce(CVector2& cForce){

	VMC(cForce);

	//UpdateWheelSpeeds();
}

/****************************************/
/****************************************/

void CBTFootbotMotionControl::VMC(CVector2& c_force) {

	/*m_fForwardSpeed = m_fLinearKVMC * c_force.GetX();
	m_fAngularSpeed = m_fAngularKVMC * c_force.GetY();

	if(m_fForwardSpeed < 0.0 && m_bForwardOnly){
		m_fForwardSpeed = 0.0;
	}

	m_fForwardSpeed+=m_fBiasForwardSpeed;*/


	// Get the heading angle
	   CRadians cHeadingAngle = c_force.Angle().SignedNormalize();
	   //std::cout << "normalized angle: " << c_force.Angle() << "\n";
	   // Clamp the speed so that it's not greater than MaxSpeed
	   Real fBaseAngularWheelSpeed = 10.0f;
	   CRadians noturn = CRadians(0);
	   noturn.FromValueInDegrees(5);
	   CRadians hardturn = CRadians(0);
	   hardturn.FromValueInDegrees(90);

	   Real fSpeed1, fSpeed2;
	   //Turning state switching conditions
	   if(cHeadingAngle.GetAbsoluteValue() <= noturn.GetAbsoluteValue()) {
		   //std::cout << "no turn\n";
	      // No Turn, heading angle very small
		   fSpeed1 = m_fBiasForwardSpeed + fBaseAngularWheelSpeed;
		   fSpeed2 = m_fBiasForwardSpeed + fBaseAngularWheelSpeed;
	   }
	   else if(cHeadingAngle.GetAbsoluteValue() >= hardturn.GetAbsoluteValue()) {
	      // Hard Turn, heading angle very large
		   fSpeed1 = m_fBiasForwardSpeed + -10.0f;
		   fSpeed2 = m_fBiasForwardSpeed + 10.0f;
		   //std::cout << "hard turn\n";
	   }
	   else {
	      // soft Turn, heading angle in between the two cases
		   Real fSpeedFactor = (CRadians(90) - Abs(cHeadingAngle)) / CRadians(90)+ 0.5;
		   //std::cout << "speedfactor: " << fSpeedFactor << "\n";
		   fSpeed1 = m_fBiasForwardSpeed + fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
		   fSpeed2 = m_fBiasForwardSpeed + fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
		   fSpeed1 = m_fBiasForwardSpeed + -5.0f;
		   fSpeed2 = m_fBiasForwardSpeed + 15.0f;
		   //std::cout << " soft turn " << fSpeed1 << " " << fSpeed2 << "\n";
	   }


	   // Apply the calculated speeds to the appropriate wheels
	   Real fLeftWheelSpeed, fRightWheelSpeed;
	   if(cHeadingAngle > CRadians::ZERO) {
	      // Turn Left
	      fLeftWheelSpeed  = fSpeed1;
	      fRightWheelSpeed = fSpeed2;
	   }
	   else {
	      // Turn Right
	      fLeftWheelSpeed  = fSpeed2;
	      fRightWheelSpeed = fSpeed1;
	   }
	   m_fLeftSpeed = fLeftWheelSpeed;
	   m_fRightSpeed = fRightWheelSpeed;
	   /* Finally, set the wheel speeds
	  // m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);*/

}

/****************************************/
/****************************************/

/*void CBTFootbotMotionControl::CMC(CVector2& c_force) {
	m_fForwardSpeed = m_fCMCForwardSpeed;
	m_fAngularSpeed = m_fKCMCErol * c_force.Angle().GetValue();

	if (m_bSaturateSpeed) {
		m_fAngularSpeed = SaturateSpeed(m_fAngularSpeed, m_fAngularSaturation);
	}

	if (m_fForwardSpeed < 0.0 && m_bForwardOnly) {
		m_fForwardSpeed = 0.0;
	}
}*/

/****************************************/
/****************************************/

/*void CBTFootbotMotionControl::ErolMotionControl(CVector2& c_force) {
	CRadians c_target_angle = c_force.Angle();
	Real fDotProduct = 0.0;

	if (c_target_angle > CRadians::PI_OVER_TWO || c_target_angle < -CRadians::PI_OVER_TWO) {
		fDotProduct = 0.0;
	}
	else {
		CVector2 cForwardVector(1.0, CRadians::ZERO);
		CVector2 cTargetVector(1.0, c_target_angle);
		fDotProduct = cForwardVector.DotProduct(cTargetVector);
	}

	m_fAngularSpeed = m_fKCMCErol * c_target_angle.GetValue();
	m_fForwardSpeed = fDotProduct * m_fCMCForwardSpeed;
}*

/****************************************/
/****************************************/

void CBTFootbotMotionControl::UpdateWheelSpeeds(){
	//Real fNoise = m_pcRNG->Uniform(m_cRandomNoiseRange);
	//LOG << "Noise: " << fNoise << std::endl;
	//m_fAngularSpeed+=fNoise;

	if (m_bSaturateSpeed) {
		m_fForwardSpeed = SaturateSpeed(m_fForwardSpeed,m_fForwardSaturation);
		m_fAngularSpeed = SaturateSpeed(m_fAngularSpeed,m_fAngularSaturation);
	}

	m_fLeftSpeed = (m_fForwardSpeed - m_fAngularSpeed * m_fWheelsDistance);
	m_fRightSpeed = (m_fForwardSpeed + m_fAngularSpeed * m_fWheelsDistance);
}


/****************************************/
/****************************************/

void CBTFootbotMotionControl::GetComputedWheelsSpeed(Real* fLeftSpeed, Real* fRightSpeed) {
	*fLeftSpeed = m_fLeftSpeed;
	*fRightSpeed = m_fRightSpeed;
}

/****************************************/
/****************************************/

/*void CBTFootbotMotionControl::SetForwardVelocity(Real fForwardVelocity){
	m_fCMCForwardSpeed = fForwardVelocity;
}*/

/****************************************/
/****************************************/

/*void CBTFootbotMotionControl::SetWheelsDistance(Real fWheelsDistance){
	m_fWheelsDistance = fWheelsDistance;
}*/

/****************************************/
/****************************************/

/*void CBTFootbotMotionControl::SetProportionalFactor(Real fKProp){
	m_fKCMCErol = fKProp;
}*/

/****************************************/
/****************************************/

/*void CBTFootbotMotionControl::SetNoiseFactor(Real fNoise){
	m_fNoiseFactor = fNoise;
	m_cRandomNoiseRange.Set(-m_fNoiseFactor * CRadians::PI.GetValue(), m_fNoiseFactor * CRadians::PI.GetValue());
}*/

/****************************************/
/****************************************/

void CBTFootbotMotionControl::Destroy(CCI_FootBotState& c_robot_state) {
	//delete m_pcRNG;
}

/****************************************/
/****************************************/

void CBTFootbotMotionControl::Reset(CCI_FootBotState& c_robot_state) {
	Destroy(c_robot_state);
	Init(c_robot_state);
}

/****************************************/
/****************************************/

