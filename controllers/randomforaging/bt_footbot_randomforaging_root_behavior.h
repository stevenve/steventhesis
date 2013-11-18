#ifndef CBTFootbotRandomForagingRootBehavior_H_
#define CBTFootbotRandomForagingRootBehavior_H_

#include <argos3/core/utility/math/rng.h>
#include "../../BTSimple/ci_behavior.h"
#include "../../BTSimple/swarmanoid/footbot/ci_footbot_state.h"
#include "../../BTSimple/utility_classes/fsm/fsm.h"

#include "../../behaviors/bt_footbot_randomwalk.h"
#include "../../behaviors/bt_footbot_go_to_led.h"
#include "../../behaviors/bt_footbot_phototaxis.h"
#include "../../behaviors/bt_footbot_obstacle_avoidance.h"
#include "../../behaviors/bt_footbot_observe_ground.h"

using namespace argos;
using namespace btfsm;

class CBTFootbotRandomForagingRootBehavior: public CCI_Behavior<CCI_FootBotState> , public FSM<std::string> {

	private:

		/* **************** */
		/* 	 SUB-BEHAVIORS  */
		/* **************** */

	CBTFootbotRandomWalk* m_pcRandomWalk;
	CBTFootbotGoToLED* m_pcGoToLED;
	CBTFootbotPhototaxis* m_pcPhototaxis;
	CBTFootbotObstacleAvoidance* m_pcObstacleAvoidance;
	CBTFootbotMotionControl* m_pcMotionControl;
	CBTFootbotObserveGround* m_pcObserveGround;

	CCI_FootBotState* c_robot_state;

	public:

		CBTFootbotRandomForagingRootBehavior(CCI_RobotData<CCI_FootBotState>* c_robot_data);
		virtual ~CBTFootbotRandomForagingRootBehavior();

		virtual void Init(CCI_FootBotState& c_robot_state);
		virtual void Step(CCI_FootBotState& c_robot_state);

		virtual void Destroy(CCI_FootBotState& c_robot_state);
		virtual void Reset(CCI_FootBotState& c_robot_state);
		virtual void ReturnToNest();
		virtual void Explore();
		virtual void ExitNest();
		virtual bool InNest();



};

#endif /* CBTFootbotRandomForagingRootBehavior_H_ */
