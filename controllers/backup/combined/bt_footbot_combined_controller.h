#ifndef _CBTFOOTBOT_COMBINED_CONTROLLER_H_
#define _CBTFOOTBOT_COMBINED_CONTROLLER_H_

#include "bt_footbot_combined_root_behavior.h"

class CBTFootbotCombinedController: public CCI_BehaviorController<CCI_FootBotState>
{



public:

	virtual void Init ( TConfigurationNode& t_tree);

	virtual void ControlStep (  );

	virtual void Destroy (  );

	virtual UInt32 GetRootBehaviorStateID();

	virtual std::string GetRootBehaviorStateName();

	virtual CBTFootbotCombinedRootBehavior* GetRootBehavior(){
		return (CBTFootbotCombinedRootBehavior*) m_pcRootBehavior;
	}

};

#endif
