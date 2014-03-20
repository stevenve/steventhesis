#ifndef _CBTFOOTBOT_COMBINED_NOVIS_CONTROLLER_H_
#define _CBTFOOTBOT_COMBINED_NOVIS_CONTROLLER_H_

#include "bt_footbot_combined_novis_root_behavior.h"

class CBTFootbotCombinedNoVisController: public CCI_BehaviorController<CCI_FootBotState>
{



public:

	virtual void Init ( TConfigurationNode& t_tree);

	virtual void ControlStep (  );

	virtual void Destroy (  );

	virtual UInt32 GetRootBehaviorStateID();

	virtual std::string GetRootBehaviorStateName();

	virtual CBTFootbotCombinedNoVisRootBehavior* GetRootBehavior(){
		return (CBTFootbotCombinedNoVisRootBehavior*) m_pcRootBehavior;
	}

};

#endif
