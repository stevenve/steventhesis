#ifndef CBTFOOTBOTTEMPLATEROOTBEHAVIOR_H_
#define CBTFOOTBOTTEMPLATEROOTBEHAVIOR_H_

#include <argos2/common/utility/argos_random.h>
#include <argos2/common/control_interface/behavioral_toolkit/ci_behavior.h>
#include <argos2/common/control_interface/behavioral_toolkit/swarmanoid/footbot/ci_footbot_state.h>
#include <argos2/common/control_interface/behavioral_toolkit/utility_classes/fsm/fsm.h>

#include <argos2/user/eliseofe/behaviors/generic/bt_footbot_motion_control.h>
#include <argos2/user/eliseofe/behaviors/generic/bt_footbot_get_light_direction.h>

using namespace argos;
using namespace btfsm;

class CBTFootbotTemplateRootBehavior: public CCI_Behavior<CCI_FootBotState> , public FSM<std::string> {

	private:

		/* **************** */
		/* 	 SUB-BEHAVIORS  */
		/* **************** */

		//CBTFootbotSingleRobotObstacleAvoidance* m_pcObstacleAvoidance;
		CBTFootbotMotionControl* m_pcMotionControl;
		CBTFootbotGetLightDirection* m_pcGetLight;

		CCtFootbotUtility* m_pcUtility;

		/* reference vector r */
		CVector2 m_vReferenceVector;

		bool m_bInformed;

	public:

		CBTFootbotTemplateRootBehavior(CCI_RobotData<CCI_FootBotState>* c_robot_data);
		virtual ~CBTFootbotTemplateRootBehavior();

		virtual void Init(CCI_FootBotState& c_robot_state);
		virtual void Step(CCI_FootBotState& c_robot_state);

		virtual void Destroy(CCI_FootBotState& c_robot_state);
		virtual void Reset(CCI_FootBotState& c_robot_state);

		virtual inline void SetMotionControlPhysicsBasedParameters(bool b_saturate_speed, bool b_forward_only, Real f_k_cmc, Real f_cmc_forward_speed, Real f_linear_k_vmc, Real f_angular_k_vmc){
			m_pcMotionControl->SetPhysicsBasedParameters(b_saturate_speed,b_forward_only, f_k_cmc, f_cmc_forward_speed, f_linear_k_vmc, f_angular_k_vmc);
		}
		virtual CRadians GetOrientation();

		virtual void SetInformed(bool b_informed);
};

#endif /* CBTFootbotTemplateRootBehavior_H_ */
