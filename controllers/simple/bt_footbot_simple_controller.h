#ifndef _CBTFOOTBOT_FLOCKING_CONTROLLER_H_
#define _CBTFOOTBOT_FLOCKING_CONTROLLER_H_

#include "../../BTSimple/ci_behavior.h"
#include "../../BTSimple/swarmanoid/footbot/ci_footbot_state.h"

#include "bt_footbot_simple_root_behavior.h"



class CBTFootbotTemplateController: public CCI_BehaviorController<CCI_FootBotState>
{
private:


public:

    virtual void Init ( TConfigurationNode& t_tree);

    virtual void ControlStep (  );

    virtual void Destroy (  );

    virtual UInt32 GetRootBehaviorStateID();

    virtual std::string GetRootBehaviorStateName();

    virtual CBTFootbotTemplateRootBehavior* GetRootBehavior(){
        return (CBTFootbotTemplateRootBehavior*)m_pcRootBehavior;
    }
};

#endif
