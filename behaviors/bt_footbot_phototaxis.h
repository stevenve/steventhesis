#ifndef BT_FOOTBOT_GET_LIGHT_DIRECTION_H
#define BT_FOOTBOT_GET_LIGHT_DIRECTION_H

#include <argos3/core/utility/math/rng.h>
#include "../BTSimple/ci_behavior.h"
#include "../BTSimple/swarmanoid/footbot/ci_footbot_state.h"
#include "../BTSimple/utility_classes/fsm/fsm.h"
#include <argos3/core/utility/math/vector2.h>
#include "bt_footbot_motion_control.h"

//#include "ct_footbot_utility.h"

using namespace argos;
using namespace btfsm;

class CBTFootbotPhototaxis: public CCI_Behavior<CCI_FootBotState> , public FSM<std::string> {

private:

	CVector2 m_cLightDirection;
	//CRadians m_cInformationDirection;
	//CVector2 m_cInformationVector;

	/** Random Numbers Generator */
	//CRandom::CRNG* m_pcRNG;

	//Real m_fNoiseFactor;
	//Real m_fNoiseGoalFactor;

	/** Range for the random noise */
	//CRange<Real> m_cRandomNoiseRange;

	/** Range for the random noise */
	//CRange<Real> m_cRandomNoiseGoalRange;

	CBTFootbotMotionControl* m_pcMotionControl;

	bool anti;

public:

	CBTFootbotPhototaxis(CCI_RobotData<CCI_FootBotState>* c_robot_data);
    virtual ~CBTFootbotPhototaxis();

    virtual void Init(CCI_FootBotState& c_robot_state);
    virtual void Step(CCI_FootBotState& c_robot_state);
    virtual void StepAndGo(CCI_FootBotState& c_robot_state);

    virtual void Destroy(CCI_FootBotState& c_robot_state);
    virtual void Reset(CCI_FootBotState& c_robot_state);

    virtual CVector2& GetVector();
    //virtual void SetInformationDirection(CRadians& c_information_direction);
    //virtual CVector2& GetInformationVector();

    virtual CRadians GetOrientation();

    //virtual void SetNoiseFactor(Real fNoise);
    //virtual void SetNoiseFactorGoal(Real fNoise);
    virtual void SetAntiPhototaxis(bool anti);


};

#endif /* CBTFootbotPhototaxis_H_ */

