//#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/utility/math/angles.h>
#include "bt_footbot_simple_controller.h"


/****************************************/
/****************************************/

CBTFootbotTemplateRootBehavior::CBTFootbotTemplateRootBehavior(CCI_RobotData<CCI_FootBotState>* c_robot_data) :
	CCI_Behavior<CCI_FootBotState> (c_robot_data, "bt_footbot_template_root_behavior") {

	m_pcRandomWalk = new CBTFootbotRandomWalk(c_robot_data);
	m_pcGoToLED = new CBTFootbotGoToLED(c_robot_data);
	m_pcPhototaxis = new CBTFootbotPhototaxis(c_robot_data);
	m_pcObstacleAvoidance = new CBTFootbotObstacleAvoidance(c_robot_data);
	m_pcMotionControl = new CBTFootbotMotionControl(c_robot_data);

}

/****************************************/
/****************************************/

CBTFootbotTemplateRootBehavior::~CBTFootbotTemplateRootBehavior() {

}

/****************************************/
/****************************************/

void CBTFootbotTemplateRootBehavior::Init(CCI_FootBotState& c_robot_state) {

	m_pcRandomWalk->Init(c_robot_state);
	m_pcGoToLED->Init(c_robot_state);
	m_pcPhototaxis->Init(c_robot_state);
	m_pcObstacleAvoidance->Init(c_robot_state);
	m_pcMotionControl->Init(c_robot_state);
}

/****************************************/
/****************************************/

void CBTFootbotTemplateRootBehavior::Step(CCI_FootBotState& c_robot_state) {

	//m_pcRandomWalk->Step(c_robot_state);
	//m_pcGoToLED->Step(c_robot_state);
	//m_pcPhototaxis->StepAndGo(c_robot_state);
	//m_pcPhototaxis->SetAntiPhototaxis(true);
	//m_pcPhototaxis->StepAndGo(c_robot_state);
	//m_pcObstacleAvoidance->Step(c_robot_state);

	m_pcObstacleAvoidance->Step(c_robot_state);
	m_pcPhototaxis->SetAntiPhototaxis(true);
	m_pcPhototaxis->StepAndGo(c_robot_state);

	CVector2 tmp = m_pcObstacleAvoidance->GetVector();
	tmp += m_pcPhototaxis->GetVector();
	m_pcMotionControl->ComputeSpeedFromForce(tmp);
	m_pcMotionControl->Step(c_robot_state);

}

/****************************************/
/****************************************/

void CBTFootbotTemplateRootBehavior::Destroy(CCI_FootBotState& c_robot_state) {
	m_pcObstacleAvoidance->Destroy(c_robot_state);
	m_pcRandomWalk->Destroy(c_robot_state);
	m_pcGoToLED->Destroy(c_robot_state);
	m_pcPhototaxis->Destroy(c_robot_state);
	m_pcMotionControl->Destroy(c_robot_state);
}

/****************************************/
/****************************************/

void CBTFootbotTemplateRootBehavior::Reset(CCI_FootBotState& c_robot_state) {
	Destroy(c_robot_state);
	Init(c_robot_state);
}

/****************************************/
/****************************************/
