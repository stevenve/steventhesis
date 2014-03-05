#ifndef _CBTFOOTBOT_RECRUITER_CONTROLLER_H_
#define _CBTFOOTBOT_RECRUITER_CONTROLLER_H_

#include "bt_footbot_recruiter_root_behavior.h"

class CBTFootbotRecruiterController: public CCI_BehaviorController<CCI_FootBotState>
{



public:

	virtual void Init ( TConfigurationNode& t_tree);

	virtual void ControlStep (  );

	virtual void Destroy (  );

	virtual UInt32 GetRootBehaviorStateID();

	virtual std::string GetRootBehaviorStateName();

	virtual CBTFootbotRecruiterRootBehavior* GetRootBehavior(){
		return (CBTFootbotRecruiterRootBehavior*) m_pcRootBehavior;
	}

};

#endif
