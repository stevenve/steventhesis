#ifndef CBT_FOOTBOT_BEACON_H
#define CBT_FOOTBOT_BEACON_H

#include <argos3/core/utility/math/rng.h>
#include "../BTSimple/ci_behavior.h"
#include "../BTSimple/swarmanoid/footbot/ci_footbot_state.h"
#include "../BTSimple/utility_classes/fsm/fsm.h"
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/datatypes/color.h>

using namespace argos;
using namespace btfsm;

class CBTFootbotBeacon: public CCI_Behavior<CCI_FootBotState> , public FSM<std::string> {

private:


public:

	CBTFootbotBeacon(CCI_RobotData<CCI_FootBotState>* c_robot_data);
    virtual ~CBTFootbotBeacon();

    virtual void Init(CCI_FootBotState& c_robot_state);
    virtual void Step(CCI_FootBotState& c_robot_state);

    virtual void Destroy(CCI_FootBotState& c_robot_state);
    virtual void Reset(CCI_FootBotState& c_robot_state);

    virtual void SetBeaconColor(CCI_FootBotState& c_robot_state, const CColor& c_color);
};

#endif

