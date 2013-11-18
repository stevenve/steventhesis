#ifndef BT_FOOTBOT_RANDOM_WALK_H_
#define BT_FOOTBOT_RANDOM_WALK_H_

#include "../BTSimple/ci_behavior.h"
#include "../BTSimple/swarmanoid/footbot/ci_footbot_state.h"
#include "../BTSimple/utility_classes/fsm/fsm.h"
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/rng.h>
#include "bt_footbot_motion_control.h"

using namespace argos;
using namespace btfsm;

class CBTFootbotRandomWalk: public CCI_Behavior<CCI_FootBotState> , public FSM<std::string> {
	public:

	CBTFootbotRandomWalk(CCI_RobotData<CCI_FootBotState>* c_robot_data);
	virtual ~CBTFootbotRandomWalk();

	virtual void Init(CCI_FootBotState& cRobotState);

	virtual void Step(CCI_FootBotState& cRobotState);

	virtual void StepAndGo(CCI_FootBotState& cRobotState);

	virtual CVector2 GetVector();

	virtual void Destroy(CCI_FootBotState& cRobotState);

	virtual void Reset(CCI_FootBotState& cRobotState);

	private:
	/** Random Numbers Generator */
	CRange<Real> m_cRandomAngle;
	CRandom::CRNG* m_pcRNG;

	CVector2 m_cRandomWalkDirection;
	Real m_fNoiseFactor;
	CBTFootbotMotionControl* m_pcMotionControl;

};

#endif /* BT_FOOTBOT_RANDOM_WALK_H_ */
