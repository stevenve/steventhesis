//#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/utility/math/angles.h>
#include "bt_footbot_recruitment_controller.h"


/****************************************/
/****************************************/

CBTFootbotRecruitmentRootBehavior::CBTFootbotRecruitmentRootBehavior(CCI_RobotData<CCI_FootBotState>* c_robot_data) :
	CCI_Behavior<CCI_FootBotState> (c_robot_data, "bt_footbot_template_root_behavior") {
	CCI_RobotData<CCI_FootBotState>* c_robot_state = c_robot_data;

	m_pcWalk = new CBTFootbotRandomWalk(c_robot_data);
	m_pcGoToLED = new CBTFootbotGoToLED(c_robot_data);
	m_pcPhototaxis = new CBTFootbotPhototaxis(c_robot_data);
	m_pcObstacleAvoidance = new CBTFootbotObstacleAvoidance(c_robot_data);
	m_pcMotionControl = new CBTFootbotMotionControl(c_robot_data);
	m_pcObserveGround = new CBTFootbotObserveGround(c_robot_data);
	m_pcControlLeds = new CBTFootbotControlLeds(c_robot_data);
	m_pcOdometry = new CBTFootbotOdometry(c_robot_data);
}

/****************************************/
/****************************************/

CBTFootbotRecruitmentRootBehavior::~CBTFootbotRecruitmentRootBehavior() {

}

/****************************************/
/****************************************/

void CBTFootbotRecruitmentRootBehavior::Init(CCI_FootBotState& state) {
	c_robot_state = &state;

	m_pcWalk->Init(state);
	m_pcGoToLED->Init(state);
	m_pcPhototaxis->Init(state);
	m_pcObstacleAvoidance->Init(state);
	m_pcMotionControl->Init(state);
	m_pcObserveGround->Init(state);
	m_pcControlLeds->Init(state);
	m_pcOdometry->Init(state);
}

/****************************************/
/****************************************/

void CBTFootbotRecruitmentRootBehavior::Step(CCI_FootBotState& c_robot_state) {

	//m_pcWalk->Step(c_robot_state);
	//m_pcGoToLED->Step(c_robot_state);
	//m_pcPhototaxis->StepAndGo(c_robot_state);
	//m_pcPhototaxis->SetAntiPhototaxis(true);
	//m_pcPhototaxis->StepAndGo(c_robot_state);
	//m_pcObstacleAvoidance->Step(c_robot_state);

	/*m_pcObstacleAvoidance->Step(c_robot_state);
	m_pcPhototaxis->SetAntiPhototaxis(true);
	m_pcPhototaxis->StepAndGo(c_robot_state);

	CVector2 tmp = m_pcObstacleAvoidance->GetVector();
	tmp += m_pcPhototaxis->GetVector();
	m_pcMotionControl->ComputeSpeedFromForce(tmp);
	m_pcMotionControl->Step(c_robot_state);*/

}

/****************************************/
/****************************************/

void CBTFootbotRecruitmentRootBehavior::Destroy(CCI_FootBotState& c_robot_state) {
	m_pcObstacleAvoidance->Destroy(c_robot_state);
	m_pcWalk->Destroy(c_robot_state);
	m_pcGoToLED->Destroy(c_robot_state);
	m_pcPhototaxis->Destroy(c_robot_state);
	m_pcMotionControl->Destroy(c_robot_state);
	m_pcOdometry->Destroy(c_robot_state);
}

/****************************************/
/****************************************/

void CBTFootbotRecruitmentRootBehavior::Reset(CCI_FootBotState& c_robot_state) {
	Destroy(c_robot_state);
	Init(c_robot_state);
}

/****************************************/
/****************************************/



void CBTFootbotRecruitmentRootBehavior::StartOdometry(){
	m_pcOdometry->Reset(*c_robot_state);
	m_pcOdometry->Start();
}

void CBTFootbotRecruitmentRootBehavior::StopOdometry(){
	m_pcOdometry->Stop();
}

void CBTFootbotRecruitmentRootBehavior::Explore() {
	m_pcControlLeds->SetAllLedsColor(*c_robot_state, CColor::BLACK);
	m_pcOdometry->Reset(*c_robot_state);
			//std::cout << "explore\n";

	m_pcObstacleAvoidance->Step(*c_robot_state);
	m_pcWalk->Step(*c_robot_state);
	//m_pcPhototaxis->SetAntiPhototaxis(true);
	//m_pcPhototaxis->Step(*c_robot_state);

	CVector2 tmp = m_pcObstacleAvoidance->GetVector();
    tmp += m_pcWalk->GetVector();
    //tmp += 0.1f * m_pcPhototaxis->GetVector();

    m_pcMotionControl->ComputeSpeedFromForce(tmp);
    m_pcMotionControl->Step(*c_robot_state);
}

void CBTFootbotRecruitmentRootBehavior::ExitNest() {
	std::cout << "UIT DA NEST\n";
	m_pcControlLeds->SetAllLedsColor(*c_robot_state, CColor::BLACK);

	m_pcObstacleAvoidance->Step(*c_robot_state);
	m_pcPhototaxis->SetAntiPhototaxis(true);
	m_pcPhototaxis->Step(*c_robot_state);

	CVector2 tmp = m_pcObstacleAvoidance->GetVector();
    tmp += m_pcPhototaxis->GetVector();

    m_pcMotionControl->ComputeSpeedFromForce(tmp);
    m_pcMotionControl->Step(*c_robot_state);
}

bool CBTFootbotRecruitmentRootBehavior::InNest(){
	m_pcObserveGround->SetColor(CBTFootbotObserveGround::GRAY);
	m_pcObserveGround->Step(*c_robot_state);
	return m_pcObserveGround->OnColor();
}

void CBTFootbotRecruitmentRootBehavior::GoToVector(CVector2 vec){
	m_pcMotionControl->ComputeSpeedFromForce(vec);
	m_pcMotionControl->Step(*c_robot_state);
}

void CBTFootbotRecruitmentRootBehavior::GoToFood(){
	//m_pcOdometry->Stop();
	CVector2 tmp = m_pcObstacleAvoidance->GetVector();
	tmp += m_pcOdometry->GetReversedLocationVector();
	//std::cout << m_pcOdometry->GetReversedLocationVector().Angle() << "   " << m_pcOdometry->GetReversedLocationVector().Length() <<"\n";
	//std::cout << tmp.Angle() << "   " << tmp.Length() <<"\n";
	std::cout << "gotofood\n";
	GoToVector(tmp);
	m_pcOdometry->Step(*c_robot_state);
}

bool CBTFootbotRecruitmentRootBehavior::IsDoneLookingForFood(){
	CVector2 tmp = m_pcOdometry->GetReversedLocationVector();
	return tmp.Length() <= 7;
}

void CBTFootbotRecruitmentRootBehavior::ResetOdometry(){
	m_pcOdometry->Reset(*c_robot_state);
}

void CBTFootbotRecruitmentRootBehavior::ReturnToNest() {
	//std::cout << "return to nest\n";
	m_pcOdometry->Start();

	//std::cout << "normal: " << m_pcOdometry->GetDistance() << " " << m_pcOdometry->GetAngle() << "\n";
	//std::cout << m_pcOdometry->GetReversedLocationVector().Angle() << "\n";

	m_pcControlLeds->SetAllLedsColor(*c_robot_state, CColor::GREEN);

	m_pcPhototaxis->SetAntiPhototaxis(false);
	m_pcPhototaxis->Step(*c_robot_state);
	m_pcObstacleAvoidance->Step(*c_robot_state);

	CVector2 tmp = m_pcObstacleAvoidance->GetVector();
	tmp += 2000.0f * m_pcPhototaxis->GetVector();
	//std::cout << tmp.Angle() << "   " << tmp.Length() <<"\n";
	GoToVector(tmp);
	m_pcOdometry->Step(*c_robot_state);
	//m_pcMotionControl->ComputeSpeedFromForce(tmp);
	//m_pcMotionControl->Step(*c_robot_state);
}
