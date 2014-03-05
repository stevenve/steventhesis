//#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/utility/math/angles.h>
#include "bt_footbot_recruiter_controller.h"


/****************************************/

CBTFootbotRecruiterRootBehavior::CBTFootbotRecruiterRootBehavior(CCI_RobotData<CCI_FootBotState>* c_robot_data) :
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
	m_pcSignalling = new CBTFootbotRABSignal(c_robot_data);
	pickedUp = false;
	timer = 0;
}

/****************************************/

CBTFootbotRecruiterRootBehavior::~CBTFootbotRecruiterRootBehavior() {

}

/****************************************/

void CBTFootbotRecruiterRootBehavior::Init(CCI_FootBotState& state) {
	c_robot_state = &state;

	m_pcWalk->Init(state);
	m_pcGoToLED->Init(state);
	m_pcPhototaxis->Init(state);
	m_pcObstacleAvoidance->Init(state);
	m_pcMotionControl->Init(state);
	m_pcObserveGround->Init(state);
	m_pcControlLeds->Init(state);
	m_pcOdometry->Init(state);
	m_pcSignalling->Init(state);

	m_sStateData.Init();
	m_sFoodData.Init();

	timer = 0;
	pickedUp = false;
}


/****************************************/

void CBTFootbotRecruiterRootBehavior::Destroy(CCI_FootBotState& c_robot_state) {
	m_pcObstacleAvoidance->Destroy(c_robot_state);
	m_pcWalk->Destroy(c_robot_state);
	m_pcGoToLED->Destroy(c_robot_state);
	m_pcPhototaxis->Destroy(c_robot_state);
	m_pcMotionControl->Destroy(c_robot_state);
	m_pcObserveGround->Destroy(c_robot_state);
	m_pcControlLeds->Destroy(c_robot_state);
	m_pcOdometry->Destroy(c_robot_state);
	m_pcSignalling->Destroy(c_robot_state);
}


/****************************************/

void CBTFootbotRecruiterRootBehavior::Reset(CCI_FootBotState& c_robot_state) {
	//Destroy(c_robot_state);
	Init(c_robot_state);
}

/****************************************/

void CBTFootbotRecruiterRootBehavior::Step(CCI_FootBotState& c_robot_state) {

	// Update Data
	UpdateFoodData();
	UpdateStateData();

	// Decide which behaviour to execute
	switch(m_sStateData.State) {
	case SStateData::STATE_EXPLORING: {
		Explore();
		break;
	}
	case SStateData::STATE_SIGNAL_AND_PICK_UP: {
		SignalPickUp();
		break;
	}
	case SStateData::STATE_DROP: {
		Drop();
		break;
	}
	case SStateData::STATE_RETURN_TO_NEST: {
		ReturnToNest();
		break;
	}
	case SStateData::STATE_GO_TO_FOOD: {
		GoToFood();
		break;
	}
	default: {
		THROW_ARGOSEXCEPTION("Invalid State");
	}
	}
}


/****************************************/
/** Behavior methods				   **/
/****************************************/

void CBTFootbotRecruiterRootBehavior::SignalPickUp(){
	CVector2 vect = CVector2(0,0);
	m_pcMotionControl->ComputeSpeedFromForce(vect);
	m_pcMotionControl->Step(*c_robot_state);

	if(--timer == 0){
		pickedUp = true;
	}
}

void CBTFootbotRecruiterRootBehavior::Drop(){
	CVector2 vect = CVector2(0,0);
	m_pcMotionControl->ComputeSpeedFromForce(vect);
	m_pcMotionControl->Step(*c_robot_state);

	if(--timer == 0){
		pickedUp = false;
	}
}

void CBTFootbotRecruiterRootBehavior::Explore() {

	if(m_sStateData.InNest) {
		ExitNest();
	}
	else {

		// Get vectors for obstacle avoidance and random walk
		m_pcObstacleAvoidance->Step(*c_robot_state);
		m_pcWalk->Step(*c_robot_state);


		// Calculate vector
		CVector2 targetVector = m_pcObstacleAvoidance->GetVector();
		targetVector += m_pcWalk->GetVector();

		// Apply vector
		GoToVector(targetVector);
	}
}

void CBTFootbotRecruiterRootBehavior::ExitNest() {

	m_pcObstacleAvoidance->Step(*c_robot_state);
	m_pcPhototaxis->SetAntiPhototaxis(true);
	m_pcPhototaxis->Step(*c_robot_state);

	CVector2 tmp = m_pcObstacleAvoidance->GetVector();
	tmp += m_pcPhototaxis->GetVector();

	GoToVector(tmp);
}

void CBTFootbotRecruiterRootBehavior::GoToFood(){

	CVector2 tmp = m_pcObstacleAvoidance->GetVector();
	tmp += m_pcOdometry->GetReversedLocationVector();
	tmp.Normalize();
	GoToVector(tmp);
	m_pcOdometry->Step(*c_robot_state);
}

void CBTFootbotRecruiterRootBehavior::ReturnToNest() {

	m_pcPhototaxis->SetAntiPhototaxis(false);
	m_pcPhototaxis->Step(*c_robot_state);
	m_pcObstacleAvoidance->Step(*c_robot_state);

	CVector2 tmp = m_pcObstacleAvoidance->GetVector();
	tmp += 2000.0f * m_pcPhototaxis->GetVector();

	GoToVector(tmp);
	m_pcOdometry->Step(*c_robot_state);

}

/****************************************/
/** Helper methods					   **/
/****************************************/

void CBTFootbotRecruiterRootBehavior::StartOdometry(){
	m_pcOdometry->Reset(*c_robot_state);
	m_pcOdometry->Start();
}

void CBTFootbotRecruiterRootBehavior::StopOdometry(){
	m_pcOdometry->Stop();
}

bool CBTFootbotRecruiterRootBehavior::InNest(){
	m_pcObserveGround->SetColor(CBTFootbotObserveGround::GRAY);
	m_pcObserveGround->Step(*c_robot_state);
	return m_pcObserveGround->OnColor();
}

void CBTFootbotRecruiterRootBehavior::GoToVector(CVector2 vec){
	m_pcMotionControl->ComputeSpeedFromForce(vec);
	m_pcMotionControl->Step(*c_robot_state);
}

bool CBTFootbotRecruiterRootBehavior::IsDoneLookingForFood(){
	CVector2 tmp = m_pcOdometry->GetReversedLocationVector();
	return tmp.Length() <= 7;
}

void CBTFootbotRecruiterRootBehavior::ResetOdometry(){
	m_pcOdometry->Reset(*c_robot_state);
}




/****************************************/
/** state related methods
/****************************************/

CBTFootbotRecruiterRootBehavior::SFoodData::SFoodData() :
																						   HasFoodItem(false),
																						   FoodItemIdx(0),
																						   TotalFoodItems(0),
																						   LastFoodPosition(0,0){
}

void CBTFootbotRecruiterRootBehavior::SFoodData::Init(){
	Reset();
}

void CBTFootbotRecruiterRootBehavior::SFoodData::Reset() {
	HasFoodItem = false;
	FoodItemIdx = 0;
	TotalFoodItems = 0;
	FoodPatchIdx = 0;
}

void CBTFootbotRecruiterRootBehavior::UpdateFoodData(){
	//TODO statistics
}

/******/

CBTFootbotRecruiterRootBehavior::SStateData::SStateData() : CurrentPosition(0,0){
	State = STATE_EXPLORING;
	InNest = true;
}

void CBTFootbotRecruiterRootBehavior::SStateData::Init(/*TConfigurationNode& t_node*/) {
	Reset();
}

void CBTFootbotRecruiterRootBehavior::SStateData::Reset() {
	State = STATE_EXPLORING;
	InNest = false;
}

void CBTFootbotRecruiterRootBehavior::UpdateStateData(){
	if(InNest())
		m_sStateData.InNest = true;
	else
		m_sStateData.InNest = false;

	if(m_sStateData.State == SStateData::STATE_DROP && pickedUp == false){
		m_pcControlLeds->SetAllLedsColor(*c_robot_state, CColor::RED);
		m_sStateData.State = SStateData::STATE_GO_TO_FOOD;
	}else if(m_sStateData.State == SStateData::STATE_RETURN_TO_NEST && m_sStateData.InNest) {
		m_pcControlLeds->SetAllLedsColor(*c_robot_state, CColor::WHITE);
		timer = PICKUP_TIME;
		m_sStateData.State = SStateData::STATE_DROP;
	}else if((m_sStateData.State == SStateData::STATE_EXPLORING || m_sStateData.State == SStateData::STATE_GO_TO_FOOD) && m_sFoodData.HasFoodItem) {
		m_pcControlLeds->SetAllLedsColor(*c_robot_state, CColor::WHITE);
		timer = PICKUP_TIME;
		m_pcSignalling->StartSignal(*c_robot_state);
		m_sStateData.State = SStateData::STATE_SIGNAL_AND_PICK_UP;
	}else if(m_sStateData.State == SStateData::STATE_SIGNAL_AND_PICK_UP && pickedUp){
		m_pcControlLeds->SetAllLedsColor(*c_robot_state, CColor::GREEN);
		m_pcOdometry->Reset(*c_robot_state);
		m_pcOdometry->Start();
		m_pcSignalling->StopSignal(*c_robot_state);
		m_sStateData.State = SStateData::STATE_RETURN_TO_NEST;
	}else if(m_sStateData.State == SStateData::STATE_GO_TO_FOOD && IsDoneLookingForFood()){
		m_pcControlLeds->SetAllLedsColor(*c_robot_state, CColor::BLACK);
		m_sStateData.State = SStateData::STATE_EXPLORING;
	}
}




