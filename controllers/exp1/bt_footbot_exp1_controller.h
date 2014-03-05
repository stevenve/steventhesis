#ifndef _CBTFOOTBOT_RECRUITMENT_CONTROLLER_H_
#define _CBTFOOTBOT_RECRUITMENT_CONTROLLER_H_

#include "bt_footbot_exp1_root_behavior.h"

class CBTFootbotExp1Controller: public CCI_BehaviorController<CCI_FootBotState>
{



public:

	virtual void Init ( TConfigurationNode& t_tree);

	virtual void ControlStep (  );

	virtual void Destroy (  );

	virtual UInt32 GetRootBehaviorStateID();

	virtual std::string GetRootBehaviorStateName();

	virtual CBTFootbotExp1RootBehavior* GetRootBehavior(){
		return (CBTFootbotExp1RootBehavior*) m_pcRootBehavior;
	}

};

#endif
