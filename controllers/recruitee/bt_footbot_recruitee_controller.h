#ifndef _CBTFOOTBOT_RECRUITEE_CONTROLLER_H_
#define _CBTFOOTBOT_RECRUITEE_CONTROLLER_H_

#include "bt_footbot_recruitee_root_behavior.h"

class CBTFootbotRecruiteeController: public CCI_BehaviorController<CCI_FootBotState>
{



public:

	virtual void Init ( TConfigurationNode& t_tree);

	virtual void ControlStep (  );

	virtual void Destroy (  );

	virtual UInt32 GetRootBehaviorStateID();

	virtual std::string GetRootBehaviorStateName();

	virtual CBTFootbotRecruiteeRootBehavior* GetRootBehavior(){
		return (CBTFootbotRecruiteeRootBehavior*) m_pcRootBehavior;
	}

};

#endif
