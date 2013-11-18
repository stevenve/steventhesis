#ifndef CBT_FOOTBOT_OBSERVE_GROUND_H
#define CBT_FOOTBOT_OBSERVE_GROUND_H

#include <argos3/core/utility/math/rng.h>
#include "../BTSimple/ci_behavior.h"
#include "../BTSimple/swarmanoid/footbot/ci_footbot_state.h"
#include "../BTSimple/utility_classes/fsm/fsm.h"
#include <argos3/core/utility/math/vector2.h>

#include "bt_footbot_motion_control.h"

using namespace argos;
using namespace btfsm;

class CBTFootbotObserveGround : public CCI_Behavior<CCI_FootBotState> , public FSM<std::string> {



public:

    enum color { WHITE = 1, GRAY = 2, BLACK = 3 };

    CBTFootbotObserveGround(CCI_RobotData<CCI_FootBotState>* c_robot_data);
    virtual ~CBTFootbotObserveGround();

    virtual void Init(CCI_FootBotState& c_robot_state);
    virtual void Step(CCI_FootBotState& c_robot_state);

    virtual void Destroy(CCI_FootBotState& c_robot_state);
    virtual void Reset(CCI_FootBotState& c_robot_state);

    inline void SetColor(color c_color){ m_color = c_color; }
    virtual bool OnColor();
    //inline std::vector<SReading> GetReadings();

private:


    color m_color;
    CCI_FootBotMotorGroundSensor::TReadings tReadings;

};

#endif

