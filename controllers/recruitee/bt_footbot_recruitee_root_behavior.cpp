//#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/utility/math/angles.h>
#include "bt_footbot_recruitee_controller.h"


/****************************************/

CBTFootbotRecruiteeRootBehavior::CBTFootbotRecruiteeRootBehavior(CCI_RobotData<CCI_FootBotState>* c_robot_data) :
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
	signalFound = false;
	closeToSignal = false;
	foodHandlingTimer = 0;
	signalExploreTimer = 0;
}

/****************************************/

CBTFootbotRecruiteeRootBehavior::~CBTFootbotRecruiteeRootBehavior() {

}

/****************************************/

void CBTFootbotRecruiteeRootBehavior::Init(CCI_FootBotState& state) {
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

	foodHandlingTimer = 0;
	signalExploreTimer = 0;
	pickedUp = false;
	signalFound = false;
	closeToSignal = false;
}


/****************************************/

void CBTFootbotRecruiteeRootBehavior::Destroy(CCI_FootBotState& c_robot_state) {
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

void CBTFootbotRecruiteeRootBehavior::Reset(CCI_FootBotState& c_robot_state) {
	//Destroy(c_robot_state);
	Init(c_robot_state);
}

/****************************************/

void CBTFootbotRecruiteeRootBehavior::Step(CCI_FootBotState& c_robot_state) {

	// Update Data
	UpdateFoodData();
	UpdateStateData();

	// Decide which behaviour to execute
	switch(m_sStateData.State) {
	case SStateData::STATE_EXPLORING: {
		Explore();
		break;
	}
	case SStateData::STATE_PICK_UP: {
		PickUp();
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
	case SStateData::STATE_FOLLOW_SIGNAL: {
		FollowSignal();
		break;
	}
	case SStateData::STATE_EXPLORING_SIGNAL_AREA: {
		ExploreSignalArea();
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

void CBTFootbotRecruiteeRootBehavior::FollowSignal(){
	m_pcSignalling->Step(*c_robot_state);
	CVector2 tmp = m_pcSignalling->GetVectorToClosestSignal(*c_robot_state);
	GoToVector(tmp);
	m_pcOdometry->Step(*c_robot_state);

	if(tmp.Length() <= SIGNAL_CLOSE_RANGE)
		closeToSignal = true;
}

void CBTFootbotRecruiteeRootBehavior::ExploreSignalArea(){
	--signalExploreTimer;

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

void CBTFootbotRecruiteeRootBehavior::PickUp(){
	CVector2 vect = CVector2(0,0);
	m_pcMotionControl->ComputeSpeedFromForce(vect);
	m_pcMotionControl->Step(*c_robot_state);

	if(--foodHandlingTimer == 0){
		pickedUp = true;
	}
}

void CBTFootbotRecruiteeRootBehavior::Drop(){
	CVector2 vect = CVector2(0,0);
	m_pcMotionControl->ComputeSpeedFromForce(vect);
	m_pcMotionControl->Step(*c_robot_state);

	if(--foodHandlingTimer == 0){
		pickedUp = false;
	}
}

void CBTFootbotRecruiteeRootBehavior::Explore() {
	m_pcSignalling->Step(*c_robot_state);
	if(m_pcSignalling->SignalFound(*c_robot_state))
		signalFound = true;

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

void CBTFootbotRecruiteeRootBehavior::ExitNest() {

	m_pcObstacleAvoidance->Step(*c_robot_state);
	m_pcPhototaxis->SetAntiPhototaxis(true);
	m_pcPhototaxis->Step(*c_robot_state);

	CVector2 tmp = m_pcObstacleAvoidance->GetVector();
	tmp += m_pcPhototaxis->GetVector();

	GoToVector(tmp);
}

void CBTFootbotRecruiteeRootBehavior::GoToFood(){

	CVector2 tmp = m_pcObstacleAvoidance->GetVector();
	tmp += m_pcOdometry->GetReversedLocationVector();
	tmp.Normalize();
	GoToVector(tmp);
	m_pcOdometry->Step(*c_robot_state);
}

void CBTFootbotRecruiteeRootBehavior::ReturnToNest() {

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

void CBTFootbotRecruiteeRootBehavior::StartOdometry(){
	m_pcOdometry->Reset(*c_robot_state);
	m_pcOdometry->Start();
}

void CBTFootbotRecruiteeRootBehavior::StopOdometry(){
	m_pcOdometry->Stop();
}

bool CBTFootbotRecruiteeRootBehavior::InNest(){
	m_pcObserveGround->SetColor(CBTFootbotObserveGround::GRAY);
	m_pcObserveGround->Step(*c_robot_state);
	return m_pcObserveGround->OnColor();
}

void CBTFootbotRecruiteeRootBehavior::GoToVector(CVector2 vec){
	m_pcMotionControl->ComputeSpeedFromForce(vec);
	m_pcMotionControl->Step(*c_robot_state);
}

bool CBTFootbotRecruiteeRootBehavior::IsDoneLookingForFood(){
	CVector2 tmp = m_pcOdometry->GetReversedLocationVector();
	return tmp.Length() <= 7;
}

void CBTFootbotRecruiteeRootBehavior::ResetOdometry(){
	m_pcOdometry->Reset(*c_robot_state);
}




/****************************************/
/** state related methods
/****************************************/

CBTFootbotRecruiteeRootBehavior::SFoodData::SFoodData() :
																																												   HasFoodItem(false),
																																												   FoodItemIdx(0),
																																												   TotalFoodItems(0),
																																												   LastFoodPosition(0,0){
}

void CBTFootbotRecruiteeRootBehavior::SFoodData::Init(){
	Reset();
}

void CBTFootbotRecruiteeRootBehavior::SFoodData::Reset() {
	HasFoodItem = false;
	FoodItemIdx = 0;
	TotalFoodItems = 0;
	FoodPatchIdx = 0;
}

void CBTFootbotRecruiteeRootBehavior::UpdateFoodData(){
	//TODO statistics
}

/******/

CBTFootbotRecruiteeRootBehavior::SStateData::SStateData() : CurrentPosition(0,0){
	State = STATE_EXPLORING;
	InNest = true;
}

void CBTFootbotRecruiteeRootBehavior::SStateData::Init(/*TConfigurationNode& t_node*/) {
	Reset();
}

void CBTFootbotRecruiteeRootBehavior::SStateData::Reset() {
	State = STATE_EXPLORING;
	InNest = false;
}

void CBTFootbotRecruiteeRootBehavior::UpdateStateData(){
	if(InNest())
		m_sStateData.InNest = true;
	else
		m_sStateData.InNest = false;

	if(m_sStateData.State == SStateData::STATE_EXPLORING && signalFound){
		m_pcControlLeds->SetAllLedsColor(*c_robot_state, CColor::RED);
		m_sStateData.State = SStateData::STATE_FOLLOW_SIGNAL;
	}else if(m_sStateData.State == SStateData::STATE_EXPLORING && m_sFoodData.HasFoodItem){
		m_pcControlLeds->SetAllLedsColor(*c_robot_state, CColor::WHITE);
		foodHandlingTimer = FOOD_HANDLING_TIME;
		m_sStateData.State = SStateData::STATE_PICK_UP;
	}else if(m_sStateData.State == SStateData::STATE_FOLLOW_SIGNAL){
		if(m_sFoodData.HasFoodItem){
			m_pcControlLeds->SetAllLedsColor(*c_robot_state, CColor::WHITE);
			foodHandlingTimer = FOOD_HANDLING_TIME;
			m_sStateData.State = SStateData::STATE_PICK_UP;
		}else if(closeToSignal){
			m_pcControlLeds->SetAllLedsColor(*c_robot_state, CColor::RED);
			signalExploreTimer = SIGNAL_EXPLORE_TIME;
			m_sStateData.State = SStateData::STATE_EXPLORING_SIGNAL_AREA;
		}
	}else if(m_sStateData.State == SStateData::STATE_EXPLORING_SIGNAL_AREA){
		if(m_sFoodData.HasFoodItem){
			m_pcControlLeds->SetAllLedsColor(*c_robot_state, CColor::WHITE);
			foodHandlingTimer = FOOD_HANDLING_TIME;
			m_sStateData.State = SStateData::STATE_PICK_UP;
		} else if(signalExploreTimer == 0){
			m_pcControlLeds->SetAllLedsColor(*c_robot_state, CColor::BLACK);
			m_sStateData.State = SStateData::STATE_EXPLORING;
		}
	}else if(m_sStateData.State == SStateData::STATE_PICK_UP && pickedUp){
		m_pcControlLeds->SetAllLedsColor(*c_robot_state, CColor::GREEN);
		m_sStateData.State = SStateData::STATE_RETURN_TO_NEST;
	}else if(m_sStateData.State == SStateData::STATE_RETURN_TO_NEST && m_sStateData.InNest){
		m_pcControlLeds->SetAllLedsColor(*c_robot_state, CColor::WHITE);
		foodHandlingTimer = FOOD_HANDLING_TIME;
		m_sStateData.State = SStateData::STATE_DROP;
	}
}




