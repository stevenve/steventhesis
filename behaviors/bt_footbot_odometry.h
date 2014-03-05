#ifndef BT_FOOTBOT_ODOMETRY_H
#define BT_FOOTBOT_ODOMETRY_H

#include <argos3/core/utility/math/rng.h>
#include "../BTSimple/ci_behavior.h"
#include "../BTSimple/swarmanoid/footbot/ci_footbot_state.h"
#include "../BTSimple/utility_classes/fsm/fsm.h"
#include <argos3/core/utility/math/vector2.h>

//#include <argos2/user/eliseofe/common/ct_footbot_utility.h>

using namespace argos;
using namespace btfsm;

class CBTFootbotOdometry: public CCI_Behavior<CCI_FootBotState> , public FSM<std::string> {

private:

	CRadians theta;
	bool started;
	Real delta_s;
	Real x;
	Real y;


public:

		CBTFootbotOdometry(CCI_RobotData<CCI_FootBotState>* c_robot_data);
    virtual ~CBTFootbotOdometry();

    virtual void Init(CCI_FootBotState& c_robot_state);
    virtual void Step(CCI_FootBotState& c_robot_state);

    virtual void Destroy(CCI_FootBotState& c_robot_state);
    virtual void Reset(CCI_FootBotState& c_robot_state);

    virtual CRadians GetAngle();
    virtual void Start();
    virtual void Stop();
    virtual Real GetDistance();
    //virtual CVector2 GetSavedLocation();
    virtual CVector2 GetReversedLocationVector();
    //virtual void ReverseStep(CCI_FootBotState& c_robot_state);

};

#endif /* CBTFootbotOdometry_H_ */

