#ifndef _CBTFOOTBOT_FLOCKING_CONTROLLER_H_
#define _CBTFOOTBOT_FLOCKING_CONTROLLER_H_

#include <argos2/common/control_interface/behavioral_toolkit/ci_behavior.h>
#include <argos2/common/control_interface/behavioral_toolkit/swarmanoid/footbot/ci_footbot_state.h>

#include <argos2/user/eliseofe/controllers/bt_footbot_template_controller/bt_footbot_template_root_behavior.h>



class CBTFootbotTemplateController: public CCI_BehaviorController<CCI_FootBotState>
{
private:


public:

    virtual void Init ( TConfigurationNode& t_tree );

    virtual void ControlStep (  );

    virtual void Destroy (  );

    virtual UInt32 GetRootBehaviorStateID();

    virtual std::string GetRootBehaviorStateName();

    virtual CBTFootbotTemplateRootBehavior* GetRootBehavior(){
        return (CBTFootbotTemplateRootBehavior*)m_pcRootBehavior;
    }
};

#endif
