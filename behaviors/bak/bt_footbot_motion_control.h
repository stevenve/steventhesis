#ifndef BT_FOOTBOT_MOTION_CONTROL_H
#define BT_FOOTBOT_MOTION_CONTROL_H

#include <argos3/core/utility/math/rng.h>
#include "../BTSimple/ci_behavior.h"
#include "../BTSimple/swarmanoid/footbot/ci_footbot_state.h"
#include "../BTSimple/utility_classes/fsm/fsm.h"
#include <argos3/core/utility/math/vector2.h>

//#include <argos2/user/eliseofe/common/ct_footbot_utility.h>

using namespace argos;
using namespace btfsm;

class CBTFootbotMotionControl: public CCI_Behavior<CCI_FootBotState> , public FSM<std::string> {

private:

	Real m_fLeftSpeed;
	Real m_fRightSpeed;
	Real m_fForwardSpeed;
	Real m_fAngularSpeed;

	Real m_fBiasForwardSpeed;

	Real m_fWheelsDistance;
	Real m_fForwardSaturation;
	Real m_fAngularSaturation;
	bool m_bSaturateSpeed;
	bool m_bForwardOnly;

	//Real m_fKCMCErol;
	//Real m_fCMCForwardSpeed;

	Real m_fLinearKVMC;
	Real m_fAngularKVMC;

	//UInt32 m_unMotionControlType;

	virtual Real SaturateSpeed(Real f_original_speed, Real f_saturation_value);

	/** Random Numbers Generator */
	//CRandom::CRNG* m_pcRNG;

	Real m_fNoiseFactor;

	/** Range for the random noise */
	CRange<Real> m_cRandomNoiseRange;

public:

		CBTFootbotMotionControl(CCI_RobotData<CCI_FootBotState>* c_robot_data);
    virtual ~CBTFootbotMotionControl();

    virtual void Init(CCI_FootBotState& c_robot_state);
    virtual void Step(CCI_FootBotState& c_robot_state);

    virtual void Destroy(CCI_FootBotState& c_robot_state);
    virtual void Reset(CCI_FootBotState& c_robot_state);

    virtual void GetComputedWheelsSpeed(Real* fLeftSpeed, Real* fRightSpeed);

    virtual void ComputeSpeedFromForce(CVector2& cForce);

    //virtual void SetForwardVelocity(Real fForwardVelocity);
    virtual inline void SetBiasForwardSpeed(Real fBiasForwardSpeed){
    	m_fBiasForwardSpeed = fBiasForwardSpeed;
    }
    //virtual void SetWheelsDistance(Real fWheelsDistance);
    //virtual void SetProportionalFactor(Real fKProp);
    virtual inline void SetPhysicsBasedParameters(bool b_saturate_speed, bool b_forward_only, Real f_k_cmc, Real f_cmc_forward_speed, Real f_linear_k_vmc, Real f_angular_k_vmc){
    	m_bSaturateSpeed = b_saturate_speed;
    	m_bForwardOnly = b_forward_only;
    	//m_fKCMCErol = f_k_cmc;
    	//m_fCMCForwardSpeed = f_cmc_forward_speed;
    	m_fLinearKVMC = f_linear_k_vmc;
    	m_fAngularKVMC = f_angular_k_vmc;
    }

    /*virtual inline void SetMotionControlType(UInt32 un_type){
    	m_unMotionControlType = un_type;
    }*/

    //virtual void SetNoiseFactor(Real fNoise);

private:
    //virtual void ErolMotionControl(CVector2& cForce);
    //virtual void CMC(CVector2& cForce);
    virtual void VMC(CVector2& cForce);
    virtual void UpdateWheelSpeeds();

		//static const UInt8 MOTION_CONTROL_EROL;
		//static const UInt8 MOTION_CONTROL_CMC;
		//static const UInt8 MOTION_CONTROL_VMC;


};

#endif /* CBTFootbotMotionControl_H_ */

