#ifndef CBT_FOOTBOT_GO_TO_LED_H
#define CBT_FOOTBOT_GO_TO_LED_H

#include <argos3/core/utility/math/rng.h>
#include "../BTSimple/ci_behavior.h"
#include "../BTSimple/swarmanoid/footbot/ci_footbot_state.h"
#include "../BTSimple/utility_classes/fsm/fsm.h"
#include <argos3/core/utility/math/vector2.h>

#include "bt_footbot_motion_control.h"

using namespace argos;
using namespace btfsm;

class CBTFootbotGoToLED: public CCI_Behavior<CCI_FootBotState> , public FSM<std::string> {

private:

    CColor m_color;
    CCI_ColoredBlobOmnidirectionalCameraSensor::TBlobList m_cCameraBlobs;
    CBTFootbotMotionControl* m_pcMotionControl;

    Real m_fDistanceFromLED;
    CRadians m_cAngleFromSource;
    CVector2 vector;

public:

    CBTFootbotGoToLED(CCI_RobotData<CCI_FootBotState>* c_robot_data);
    virtual ~CBTFootbotGoToLED();

    virtual void Init(CCI_FootBotState& c_robot_state);
    virtual void Step(CCI_FootBotState& c_robot_state);
    virtual void StepAndGo(CCI_FootBotState& c_robot_state);

    virtual void Destroy(CCI_FootBotState& c_robot_state);
    virtual void Reset(CCI_FootBotState& c_robot_state);

    inline void SetColor(CColor& c_color){ m_color = c_color; }
    inline Real GetDistanceFromLED(){ return m_fDistanceFromLED; }
    inline CVector2 GetVector(){return vector;}

};

#endif

