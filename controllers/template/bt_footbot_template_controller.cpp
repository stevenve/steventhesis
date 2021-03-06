#include <argos2/common/utility/logging/argos_log.h>
#include <argos2/common/control_interface/ci_controller.h>

#include <argos2/user/eliseofe/controllers/bt_footbot_template_controller/bt_footbot_template_controller.h>


using namespace argos;


void CBTFootbotTemplateController::Init ( TConfigurationNode& t_tree )
{
    // Get the robot data, state and the root behavior
    m_pcRobotData = new CCI_RobotData<CCI_FootBotState>( GetRobot(), (CCI_BehaviorController<CCI_FootBotState>*) this, t_tree );
    m_pcState = new CCI_FootBotState(GetRobot());
    m_pcState->Init();
    m_pcRootBehavior = new CBTFootbotTemplateRootBehavior(m_pcRobotData);
    m_pcRobotData->SetRootBehavior( (CCI_Behavior<CCI_FootBotState>*) m_pcRootBehavior );

    m_pcRootBehavior->Init( *m_pcState );
}

void CBTFootbotTemplateController::ControlStep (  )
{

    // Call the control step in of the interface
    CCI_BehaviorController<CCI_FootBotState>::ControlStep();
}

UInt32 CBTFootbotTemplateController::GetRootBehaviorStateID()
{
	CBTFootbotTemplateRootBehavior* pcRootBehavior = (CBTFootbotTemplateRootBehavior*) m_pcRootBehavior;
    return pcRootBehavior->GetStateID();
}

std::string CBTFootbotTemplateController::GetRootBehaviorStateName()
{
	CBTFootbotTemplateRootBehavior* pcRootBehavior = (CBTFootbotTemplateRootBehavior*) m_pcRootBehavior;
    return pcRootBehavior->GetStateName();
}

void CBTFootbotTemplateController::Destroy (  )
{
    // Clean up in the root behavior
    m_pcRootBehavior->Destroy(*m_pcState);

    // Release memory
    delete m_pcRobotData;
    delete m_pcState;
    delete m_pcRootBehavior;
}


/*
 * To allow dynamic loading of this controller
 */
REGISTER_CONTROLLER( CBTFootbotTemplateController, "bt_footbot_template_controller" );

