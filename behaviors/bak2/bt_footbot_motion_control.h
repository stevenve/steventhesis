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

public:

		CBTFootbotMotionControl(CCI_RobotData<CCI_FootBotState>* c_robot_data);
    virtual ~CBTFootbotMotionControl();

    virtual void Init(CCI_FootBotState& c_robot_state);
    virtual void Step(CCI_FootBotState& c_robot_state);

    virtual void Destroy(CCI_FootBotState& c_robot_state);
    virtual void Reset(CCI_FootBotState& c_robot_state);

    virtual void GetComputedWheelsSpeed(Real* fLeftSpeed, Real* fRightSpeed);

    virtual void ComputeSpeedFromForce(CVector2& cForce);



};

#endif /* CBTFootbotMotionControl_H_ */

