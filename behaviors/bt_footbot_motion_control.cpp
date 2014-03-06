//#include <argos2/common/utility/logging/argos_log.h>
#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/utility/math/angles.h>

#define BEHAVIOR_NAME "[CBTFootbotMotionControl]: "

#include "bt_footbot_motion_control.h"


const UInt8 CBTFootbotMotionControl::MOTION_CONTROL_EROL = 0;
const UInt8 CBTFootbotMotionControl::MOTION_CONTROL_CMC = 1;
const UInt8 CBTFootbotMotionControl::MOTION_CONTROL_VMC = 2;
const UInt8 CBTFootbotMotionControl::MOTION_CONTROL_BASIC = 3;

/****************************************/
/****************************************/

CBTFootbotMotionControl::CBTFootbotMotionControl(CCI_RobotData<CCI_FootBotState>* c_robot_data) :
															CCI_Behavior<CCI_FootBotState> (c_robot_data, "bt_footbot_motion_control"),
															m_cRandomNoiseRange(-0.0000001, 0.0000001) {

	m_unMotionControlType = MOTION_CONTROL_VMC;//MOTION_CONTROL_EROL;

	m_fLeftSpeed = 0.0;
	m_fRightSpeed = 0.0;

	m_fWheelsDistance = 14.0;

	// CMC
	m_fKCMCErol = 0.1;
	m_fCMCForwardSpeed = 5.0;

	// OTHERS

	m_fBiasForwardSpeed = 1.5; //5.0

	m_fForwardSpeed = 0.001;
	m_fAngularSpeed = 0.0;
	m_fLinearKVMC = 1.0; //25.0
	m_fAngularKVMC = 0.12; //0.5
	m_fForwardSaturation = 25.0;
	//m_fForwardSaturation = 0.5;
	m_fAngularSaturation = CRadians::PI_OVER_TWO.GetValue();

	m_bSaturateSpeed = true;
	m_bForwardOnly = true;
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
	//LOG << "Angle: " << cForce.Angle().GetValue() << "\n";
	//LOG << "Length: " << cForce.Length() << "\n";
	//cForce.Normalize();
	if(cForce.Length() == 0){
		m_fLeftSpeed = 0;
		m_fRightSpeed = 0;
	}else{
		switch(m_unMotionControlType){
		case MOTION_CONTROL_EROL:{
			ErolMotionControl(cForce);
			break;
		}
		case MOTION_CONTROL_CMC: {
			CMC(cForce);
			break;
		}
		case MOTION_CONTROL_VMC: {
			VMC(cForce);
			break;
		}
		case MOTION_CONTROL_BASIC:{
			BASIC(cForce);
			break;
		}
		}
		UpdateWheelSpeeds();
	}
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

void CBTFootbotMotionControl::BASIC(CVector2& c_force) {

	// Get the heading angle
	CRadians cHeadingAngle = c_force.Angle().SignedNormalize();
	// Clamp the speed so that it's not greater than MaxSpeed
	Real fBaseAngularWheelSpeed = 10.0f;
	CRadians noturn = CRadians(0);
	noturn.FromValueInDegrees(5);
	CRadians hardturn = CRadians(0);
	hardturn.FromValueInDegrees(90);

	Real fSpeed1, fSpeed2;
	//Turning state switching conditions
	if(cHeadingAngle.GetAbsoluteValue() <= noturn.GetAbsoluteValue()) {
		// No Turn, heading angle very small
		fSpeed1 = m_fBiasForwardSpeed + fBaseAngularWheelSpeed;
		fSpeed2 = m_fBiasForwardSpeed + fBaseAngularWheelSpeed;
	}
	else if(cHeadingAngle.GetAbsoluteValue() >= hardturn.GetAbsoluteValue()) {
		// Hard Turn, heading angle very large
		fSpeed1 = m_fBiasForwardSpeed + -10.0f;
		fSpeed2 = m_fBiasForwardSpeed + 10.0f;
	}
	else {
		// soft Turn, heading angle in between the two cases
		Real fSpeedFactor = (CRadians(90) - Abs(cHeadingAngle)) / CRadians(90)+ 0.5;
		fSpeed1 = m_fBiasForwardSpeed + fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
		fSpeed2 = m_fBiasForwardSpeed + fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
		fSpeed1 = m_fBiasForwardSpeed + -5.0f;
		fSpeed2 = m_fBiasForwardSpeed + 15.0f;
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

}

void CBTFootbotMotionControl::CMC(CVector2& c_force) {
	m_fForwardSpeed = m_fCMCForwardSpeed;
	m_fAngularSpeed = m_fKCMCErol * c_force.Angle().GetValue();

	if (m_bSaturateSpeed) {
		m_fAngularSpeed = SaturateSpeed(m_fAngularSpeed, m_fAngularSaturation);
	}

	if (m_fForwardSpeed < 0.0 && m_bForwardOnly) {
		m_fForwardSpeed = 0.0;
	}

}

void CBTFootbotMotionControl::VMC(CVector2& c_force) {
	m_fForwardSpeed = m_fLinearKVMC * c_force.GetX();
	m_fAngularSpeed = m_fAngularKVMC * c_force.GetY();

	if(m_fForwardSpeed < 0.0 && m_bForwardOnly){
		m_fForwardSpeed = 0.0;
	}

	m_fForwardSpeed+=m_fBiasForwardSpeed;

	//LOG << "Fw: " << m_fForwardSpeed << "\n";
	//LOG << "Ang: " << m_fAngularSpeed << "\n";
}

void CBTFootbotMotionControl::ErolMotionControl(CVector2& c_force) {
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

	m_fForwardSpeed = fDotProduct * m_fCMCForwardSpeed;
	m_fAngularSpeed = m_fKCMCErol * c_target_angle.GetValue();
}

/****************************************/
/****************************************/

void CBTFootbotMotionControl::UpdateWheelSpeeds(){

	if (m_bSaturateSpeed) {
		m_fForwardSpeed = SaturateSpeed(m_fForwardSpeed,m_fForwardSaturation);
		m_fAngularSpeed = SaturateSpeed(m_fAngularSpeed,m_fAngularSaturation);
	}

	m_fLeftSpeed = (m_fForwardSpeed - m_fAngularSpeed * m_fWheelsDistance);
	m_fRightSpeed = (m_fForwardSpeed + m_fAngularSpeed * m_fWheelsDistance);

	//LOG << "Left: " << m_fLeftSpeed << "\n";
	//LOG << "Right: " << m_fRightSpeed << "\n";
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
	Init(c_robot_state);
}

/****************************************/
/****************************************/

