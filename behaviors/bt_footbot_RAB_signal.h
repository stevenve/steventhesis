#ifndef BT_FOOTBOT_RAB_SIGNAL_H_
#define BT_FOOTBOT_RAB_SIGNAL_H_

#include "../BTSimple/ci_behavior.h"
#include "../BTSimple/swarmanoid/footbot/ci_footbot_state.h"
#include "../BTSimple/utility_classes/fsm/fsm.h"
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>

using namespace argos;
using namespace btfsm;

class CBTFootbotRABSignal: public CCI_Behavior<CCI_FootBotState> , public FSM<std::string> {
	public:

		CBTFootbotRABSignal(CCI_RobotData<CCI_FootBotState>* c_robot_data);
		virtual ~CBTFootbotRABSignal();

		virtual void Init(CCI_FootBotState& c_robot_state);

		virtual void Step(CCI_FootBotState& cRobotState);

		virtual void StartSignal(CCI_FootBotState& cRobotState);
		virtual void StopSignal(CCI_FootBotState& cRobotState);
		virtual CVector2& GetVectorToClosestSignal(CCI_FootBotState& cRobotState);
		virtual bool SignalFound(CCI_FootBotState& cRobotState);
		virtual void SetMaxDistance(Real dist);

		virtual void Destroy(CCI_FootBotState& cRobotState);

		virtual void Reset(CCI_FootBotState& cRobotState);

	protected:

		Real m_fDistanceMax;
		CVector2 vectorToClosestSignal;
		bool started;
		CByteArray signal;
		bool signalFound;

};

#endif /* BT_FOOTBOT_RAB_SIGNAL_H_ */
