//#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/control_interface/ci_controller.h>

#include "bt_footbot_recruiter_controller.h"


using namespace argos;


void CBTFootbotRecruiterController::Init ( TConfigurationNode& t_tree)
{
	// Get the robot data, state and the root behavior
	m_pcRobotData = new CCI_RobotData<CCI_FootBotState>( (CCI_Controller*) this, (CCI_BehaviorController<CCI_FootBotState>*) this, t_tree );
	m_pcState = new CCI_FootBotState((CCI_Controller*) this);
	m_pcState->Init();
	m_pcRootBehavior = new CBTFootbotRecruiterRootBehavior(m_pcRobotData);
	m_pcRobotData->SetRootBehavior( (CCI_Behavior<CCI_FootBotState>*) m_pcRootBehavior );

	m_pcRootBehavior->Init( *m_pcState );

	/* reset all */
	Reset();
}

void CBTFootbotRecruiterController::ControlStep (  )
{
	// Call the control step in of the interface
	CCI_BehaviorController<CCI_FootBotState>::ControlStep();
}

UInt32 CBTFootbotRecruiterController::GetRootBehaviorStateID()
{
	CBTFootbotRecruiterRootBehavior* pcRootBehavior = (CBTFootbotRecruiterRootBehavior*) m_pcRootBehavior;
	return pcRootBehavior->GetStateID();
}

std::string CBTFootbotRecruiterController::GetRootBehaviorStateName()
{
	CBTFootbotRecruiterRootBehavior* pcRootBehavior = (CBTFootbotRecruiterRootBehavior*) m_pcRootBehavior;
	return pcRootBehavior->GetStateName();
}

void CBTFootbotRecruiterController::Destroy (  )
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
REGISTER_CONTROLLER( CBTFootbotRecruiterController, "footbot_recruiter_controller" );

