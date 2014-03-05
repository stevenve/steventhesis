#ifndef _CBTFOOTBOT_RECRUITMENT_CONTROLLER_H_
#define _CBTFOOTBOT_RECRUITMENT_CONTROLLER_H_

#include "../../BTSimple/ci_behavior.h"
#include "../../BTSimple/swarmanoid/footbot/ci_footbot_state.h"

#include "bt_footbot_exp2_root_behavior.h"



class CBTFootbotExp2Controller: public CCI_BehaviorController<CCI_FootBotState>
{



public:

	virtual void Init ( TConfigurationNode& t_tree);

	virtual void ControlStep (  );

	virtual void Destroy (  );

	virtual UInt32 GetRootBehaviorStateID();

	virtual std::string GetRootBehaviorStateName();

	virtual CBTFootbotExp2RootBehavior* GetRootBehavior(){
		return (CBTFootbotExp2RootBehavior*) m_pcRootBehavior;
	}

};

#endif
