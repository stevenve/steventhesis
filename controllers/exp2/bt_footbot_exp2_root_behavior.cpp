//#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/utility/math/angles.h>
#include "bt_footbot_exp2_controller.h"


/****************************************/

CBTFootbotExp2RootBehavior::CBTFootbotExp2RootBehavior(CCI_RobotData<CCI_FootBotState>* c_robot_data) :
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

CBTFootbotExp2RootBehavior::~CBTFootbotExp2RootBehavior() {

}

/****************************************/

void CBTFootbotExp2RootBehavior::Init(CCI_FootBotState& state) {
	c_robot_state = &state;

	m_pcWalk->Init(state);
	m_pcGoToLED->Init(state);
	m_pcPhototaxis->Init(state);
	m_pcObstacleAvoidance->Init(state);
	m_pcMotionControl->Init(state);
	m_pcObserveGround->Init(state);
	m_pcControlLeds->Init(state);
	m_pcOdometry->Init(state);

	m_sStateData.Init();
	m_sFoodData.Init();
}


/****************************************/

void CBTFootbotExp2RootBehavior::Destroy(CCI_FootBotState& c_robot_state) {
	m_pcObstacleAvoidance->Destroy(c_robot_state);
	m_pcWalk->Destroy(c_robot_state);
	m_pcGoToLED->Destroy(c_robot_state);
	m_pcPhototaxis->Destroy(c_robot_state);
	m_pcMotionControl->Destroy(c_robot_state);
	m_pcObserveGround->Destroy(c_robot_state);
	m_pcControlLeds->Destroy(c_robot_state);
	m_pcOdometry->Destroy(c_robot_state);
}


/****************************************/

void CBTFootbotExp2RootBehavior::Reset(CCI_FootBotState& c_robot_state) {
	Destroy(c_robot_state);
	Init(c_robot_state);
}

/****************************************/

void CBTFootbotExp2RootBehavior::Step(CCI_FootBotState& c_robot_state) {

	// Update Data
	UpdateFoodData();
	UpdateStateData();

	// Decide which behaviour to execute
	switch(m_sStateData.State) {
	case SStateData::STATE_EXPLORING: {
		Explore(c_robot_state);
		break;
	}
	case SStateData::STATE_RETURN_TO_NEST: {
		ReturnToNest(c_robot_state);
		break;
	}
	case SStateData::STATE_GO_TO_FOOD: {
		//GoToFood(c_robot_state);
		Explore(c_robot_state);
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

void CBTFootbotExp2RootBehavior::Explore() {

	if(m_sStateData.InNest) {
		ExitNest();
	}
	else {
		//m_pcOdometry->Reset(*c_robot_state);

		// Get vectors for obstacle avoidance and random walk
		m_pcObstacleAvoidance->Step(*c_robot_state);
		m_pcWalk->Step(*c_robot_state);


		// Choose correct vector
		CVector2 targetVector;
		targetVector.Set(0.0f, 0.0f);

		if(m_pcObstacleAvoidance->GetVector().Length() != 0.0f){
			targetVector = m_pcObstacleAvoidance->GetVector(); LOG << "avoidance\n";}
		else{
			targetVector = m_pcWalk->GetVector();LOG << "random walk\n";}

		// Apply vector
		GoToVector(targetVector);
	}
}

void CBTFootbotExp2RootBehavior::ExitNest() {
	//std::cout << "UIT DA NEST\n";
	//m_pcControlLeds->SetAllLedsColor(*c_robot_state, CColor::BLACK);

	m_pcObstacleAvoidance->Step(*c_robot_state);
	m_pcPhototaxis->SetAntiPhototaxis(true);
	m_pcPhototaxis->Step(*c_robot_state);

	CVector2 tmp = m_pcObstacleAvoidance->GetVector();
	tmp += m_pcPhototaxis->GetVector();

	GoToVector(tmp);
}

void CBTFootbotExp2RootBehavior::GoToFood(){


	//m_pcOdometry->Stop();
	CVector2 tmp = m_pcObstacleAvoidance->GetVector();
	tmp += m_pcOdometry->GetReversedLocationVector();
	//std::cout << m_pcOdometry->GetReversedLocationVector().Angle() << "   " << m_pcOdometry->GetReversedLocationVector().Length() <<"\n";
	//std::cout << tmp.Angle() << "   " << tmp.Length() <<"\n";
	std::cout << "gotofood\n";
	GoToVector(tmp);
	m_pcOdometry->Step(*c_robot_state);
}

void CBTFootbotExp2RootBehavior::ReturnToNest() {
	//std::cout << "return to nest\n";
	//m_pcOdometry->Start();

	//std::cout << "normal: " << m_pcOdometry->GetDistance() << " " << m_pcOdometry->GetAngle() << "\n";
	//std::cout << m_pcOdometry->GetReversedLocationVector().Angle() << "\n";

	//m_pcControlLeds->SetAllLedsColor(*c_robot_state, CColor::GREEN);

	m_pcPhototaxis->SetAntiPhototaxis(false);
	m_pcPhototaxis->Step(*c_robot_state);
	m_pcObstacleAvoidance->Step(*c_robot_state);

	CVector2 tmp = m_pcObstacleAvoidance->GetVector();
	tmp += 2000.0f * m_pcPhototaxis->GetVector();

	//std::cout << tmp.Angle() << "   " << tmp.Length() <<"\n";
	GoToVector(tmp);
	//m_pcOdometry->Step(*c_robot_state);

}

/****************************************/
/** Helper methods					   **/
/****************************************/

void CBTFootbotExp2RootBehavior::StartOdometry(){
	m_pcOdometry->Reset(*c_robot_state);
	m_pcOdometry->Start();
}

void CBTFootbotExp2RootBehavior::StopOdometry(){
	m_pcOdometry->Stop();
}

bool CBTFootbotExp2RootBehavior::InNest(){
	m_pcObserveGround->SetColor(CBTFootbotObserveGround::GRAY);
	m_pcObserveGround->Step(*c_robot_state);
	return m_pcObserveGround->OnColor();
}

void CBTFootbotExp2RootBehavior::GoToVector(CVector2 vec){
	m_pcMotionControl->ComputeSpeedFromForce(vec);
	m_pcMotionControl->Step(*c_robot_state);
}

bool CBTFootbotExp2RootBehavior::IsDoneLookingForFood(){
	CVector2 tmp = m_pcOdometry->GetReversedLocationVector();
	return tmp.Length() <= 7;
}

void CBTFootbotExp2RootBehavior::ResetOdometry(){
	m_pcOdometry->Reset(*c_robot_state);
}




/****************************************/
/** state related methods
/****************************************/

CBTFootbotExp2RootBehavior::SFoodData::SFoodData() :
														   HasFoodItem(false),
														   FoodItemIdx(0),
														   TotalFoodItems(0),
														   LastFoodPosition(0,0){
}

void CBTFootbotExp2RootBehavior::SFoodData::Init(){
	Reset();
}

void CBTFootbotExp2RootBehavior::SFoodData::Reset() {
	HasFoodItem = false;
	FoodItemIdx = 0;
	TotalFoodItems = 0;
}

void CBTFootbotExp2RootBehavior::UpdateFoodData(){
	//TODO statistics
}

/******/

CBTFootbotExp2RootBehavior::SStateData::SStateData() : CurrentPosition(0,0){
	State = STATE_EXPLORING;
	InNest = true;
}

void CBTFootbotExp2RootBehavior::SStateData::Init(/*TConfigurationNode& t_node*/) {
	Reset();
}

void CBTFootbotExp2RootBehavior::SStateData::Reset() {
	State = STATE_EXPLORING;
	InNest = false;
}

void CBTFootbotExp2RootBehavior::UpdateStateData(){
	if(InNest())
		m_sStateData.InNest = true;
	else
		m_sStateData.InNest = false;

	if(m_sStateData.InNest)
			m_sStateData.State = SStateData::STATE_GO_TO_FOOD;
	else if(m_sFoodData.HasFoodItem) {
		//StopOdometry();
		//ResetOdometry();
		m_sStateData.State = SStateData::STATE_RETURN_TO_NEST;
	}else if(IsDoneLookingForFood()){
		m_sStateData.State = SStateData::STATE_EXPLORING;
	}
}




