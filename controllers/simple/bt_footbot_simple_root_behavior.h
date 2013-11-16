#ifndef CBTFOOTBOTTEMPLATEROOTBEHAVIOR_H_
#define CBTFOOTBOTTEMPLATEROOTBEHAVIOR_H_

#include <argos3/core/utility/math/rng.h>
#include "../../BTSimple/ci_behavior.h"
#include "../../BTSimple/swarmanoid/footbot/ci_footbot_state.h"
#include "../../BTSimple/utility_classes/fsm/fsm.h"

#include "../../behaviors/bt_footbot_randomwalk.h"
#include "../../behaviors/bt_footbot_go_to_led.h"
#include "../../behaviors/bt_footbot_phototaxis.h"
#include "../../behaviors/bt_footbot_obstacle_avoidance.h"

using namespace argos;
using namespace btfsm;

class CBTFootbotTemplateRootBehavior: public CCI_Behavior<CCI_FootBotState> , public FSM<std::string> {

	private:

		/* **************** */
		/* 	 SUB-BEHAVIORS  */
		/* **************** */

	CBTFootbotRandomWalk* m_pcRandomWalk;
	CBTFootbotGoToLED* m_pcGoToLED;
	CBTFootbotPhototaxis* m_pcPhototaxis;
	CBTFootbotObstacleAvoidance* m_pcObstacleAvoidance;
	CBTFootbotMotionControl* m_pcMotionControl;

	public:

		CBTFootbotTemplateRootBehavior(CCI_RobotData<CCI_FootBotState>* c_robot_data);
		virtual ~CBTFootbotTemplateRootBehavior();

		virtual void Init(CCI_FootBotState& c_robot_state);
		virtual void Step(CCI_FootBotState& c_robot_state);

		virtual void Destroy(CCI_FootBotState& c_robot_state);
		virtual void Reset(CCI_FootBotState& c_robot_state);



};

#endif /* CBTFootbotTemplateRootBehavior_H_ */
