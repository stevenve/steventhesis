//#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/utility/math/angles.h>
#include "bt_footbot_combined_novis_controller.h"

/****************************************/

CBTFootbotCombinedNoVisRootBehavior::CBTFootbotCombinedNoVisRootBehavior(CCI_RobotData<CCI_FootBotState>* c_robot_data, TConfigurationNode& t_node) :
CCI_Behavior<CCI_FootBotState> (c_robot_data, "bt_footbot_template_root_behavior") {
	CCI_RobotData<CCI_FootBotState>* c_robot_state = c_robot_data;

	m_pcWalk = new CBTFootbotRandomWalk(c_robot_data);
	//m_pcGoToLED = new CBTFootbotGoToLED(c_robot_data);
	m_pcPhototaxis = new CBTFootbotPhototaxis(c_robot_data);
	m_pcObstacleAvoidance = new CBTFootbotObstacleAvoidance(c_robot_data);
	m_pcMotionControl = new CBTFootbotMotionControl(c_robot_data);
	m_pcObserveGround = new CBTFootbotObserveGround(c_robot_data);
	m_pcControlLeds = new CBTFootbotControlLeds(c_robot_data);
	m_pcOdometry = new CBTFootbotOdometry(c_robot_data);
	pickedUp = false;
	timer = 0;

	//Recruitee
	m_pcSignalling = new CBTFootbotRABSignal(c_robot_data);
	signalFound = false;
	closeToSignal = false;
	foodHandlingTimer = 0;
	signalExploreTimer = 0;

	CRandom::CRNG* m_pcRNG = CRandom::CreateRNG("argos");
	robotType = m_pcRNG->Uniform(CRange<int>(0,3));
	node = t_node;

}

/****************************************/

CBTFootbotCombinedNoVisRootBehavior::~CBTFootbotCombinedNoVisRootBehavior() {

}

/****************************************/

void CBTFootbotCombinedNoVisRootBehavior::Init(CCI_FootBotState& state) {
	c_robot_state = &state;

	m_pcWalk->Init(state);
	//m_pcGoToLED->Init(state);
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
	foodHandlingTimer = 0;
	signalExploreTimer = 0;
	pickedUp = false;
	signalFound = false;
	closeToSignal = false;

	CRandom::CRNG* m_pcRNG = CRandom::CreateRNG("argos");
	robotType = m_pcRNG->Uniform(CRange<int>(0,3));
	LOG << robotType << "\n";

	TConfigurationNode& tForaging = GetNode(node, "parameters");
	GetNodeAttribute(tForaging, "useOdometry", USE_ODOMETRY);
	GetNodeAttribute(tForaging, "exploreTime", SIGNAL_EXPLORE_TIME);
	GetNodeAttribute(tForaging, "signalTime", SIGNAL_TIME);
	GetNodeAttribute(tForaging, "dropTime", DROP_TIME);
	GetNodeAttribute(tForaging, "pickupTime", PICKUP_TIME);
	GetNodeAttribute(tForaging, "signalCloseRange", SIGNAL_CLOSE_RANGE);
	GetNodeAttribute(tForaging, "avoidanceFactor", AVOIDANCE_FACTOR);
	Real dist;
	GetNodeAttribute(tForaging, "signalDistance", dist);
	m_pcSignalling->SetMaxDistance(dist);
	GetNodeAttribute(tForaging, "obstacleAvoidanceDistance", dist);
	m_pcObstacleAvoidance->SetMaxDistance(dist);

	m_pcControlLeds->SetAllLedsColor(*c_robot_state, CColor::RED);
}


/****************************************/

void CBTFootbotCombinedNoVisRootBehavior::Destroy(CCI_FootBotState& c_robot_state) {
	m_pcObstacleAvoidance->Destroy(c_robot_state);
	m_pcWalk->Destroy(c_robot_state);
	//m_pcGoToLED->Destroy(c_robot_state);
	m_pcPhototaxis->Destroy(c_robot_state);
	m_pcMotionControl->Destroy(c_robot_state);
	m_pcObserveGround->Destroy(c_robot_state);
	m_pcControlLeds->Destroy(c_robot_state);
	m_pcOdometry->Destroy(c_robot_state);
}


/****************************************/

void CBTFootbotCombinedNoVisRootBehavior::Reset(CCI_FootBotState& c_robot_state) {
	//Destroy(c_robot_state);
	Init(c_robot_state);
}

/****************************************/

void CBTFootbotCombinedNoVisRootBehavior::SetRobotType(int type) {
	robotType = type;
}

int CBTFootbotCombinedNoVisRootBehavior::GetRobotType() {
	return robotType;
}

void CBTFootbotCombinedNoVisRootBehavior::Step(CCI_FootBotState& c_robot_state) {
	switch(robotType){
	case SOLITARY:{
		StepSolitary(c_robot_state);
		break;
	}
	case RECRUITEE:{
		StepRecruitee(c_robot_state);
		break;
	}
	case RECRUITER:{
		StepRecruiter(c_robot_state);
		break;
	}
	default: {
		THROW_ARGOSEXCEPTION("Unknown robot type");
	}
	}
}

void CBTFootbotCombinedNoVisRootBehavior::StepSolitary(CCI_FootBotState& c_robot_state) {

	// Update Data
	//UpdateFoodData();
	UpdateStateDataSolitary();

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
	case SStateData::STATE_GO_TO_FOOD: {
		GoToFood();
		break;
	}
	default: {
		THROW_ARGOSEXCEPTION("Invalid State");
	}
	}
}

void CBTFootbotCombinedNoVisRootBehavior::StepRecruitee(CCI_FootBotState& c_robot_state) {

	// Update Data
	//UpdateFoodData();
	UpdateStateDataRecruitee();

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

void CBTFootbotCombinedNoVisRootBehavior::StepRecruiter(CCI_FootBotState& c_robot_state) {

	// Update Data
	//UpdateFoodData();
	UpdateStateDataRecruiter();

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

void CBTFootbotCombinedNoVisRootBehavior::SignalPickUp(){
	CVector2 vect = CVector2(0,0);
	m_pcMotionControl->ComputeSpeedFromForce(vect);
	m_pcMotionControl->Step(*c_robot_state);

	if(--timer == 0){
		pickedUp = true;
	}
}

void CBTFootbotCombinedNoVisRootBehavior::FollowSignal(){
	m_pcObstacleAvoidance->Step(*c_robot_state);
	m_pcSignalling->Step(*c_robot_state);

	CVector2 tmp = m_pcSignalling->GetVectorToClosestSignal(*c_robot_state);

	if(tmp.Length() <= SIGNAL_CLOSE_RANGE)
		closeToSignal = true;

	if(tmp.Length() != 0)
		tmp = tmp.Normalize();
	CVector2 tmp2 = m_pcObstacleAvoidance->GetVector();
	if(tmp2.Length() != 0)
		tmp2 = AVOIDANCE_FACTOR*tmp2.Normalize();
	tmp += tmp2;

	//CVector2 tmp = m_pcSignalling->GetVectorToClosestSignal(*c_robot_state);

	GoToVector(tmp);
	m_pcOdometry->Step(*c_robot_state);
}

void CBTFootbotCombinedNoVisRootBehavior::ExploreSignalArea(){
	--signalExploreTimer;

	if(m_sStateData.InNest) {
		ExitNest();
	}
	else {
		// Step for obstacle avoidance and random walk
		m_pcObstacleAvoidance->Step(*c_robot_state);
		m_pcWalk->Step(*c_robot_state);

		// Calculate vector
		CVector2 tmp = m_pcObstacleAvoidance->GetVector();
		if(tmp.Length() != 0)
			tmp = AVOIDANCE_FACTOR *tmp.Normalize();
		CVector2 tmp2 = m_pcWalk->GetVector();
		if(tmp2.Length() != 0)
			tmp += tmp2.Normalize();
		if(tmp.Length() != 0)
			tmp.Normalize();

		// Apply vector
		GoToVector(tmp);
	}
}

void CBTFootbotCombinedNoVisRootBehavior::PickUp(){
	CVector2 vect = CVector2(0,0);
	m_pcMotionControl->ComputeSpeedFromForce(vect);
	m_pcMotionControl->Step(*c_robot_state);

	if(--timer == 0){
		pickedUp = true;
	}
}

void CBTFootbotCombinedNoVisRootBehavior::Drop(){
	CVector2 vect = CVector2(0,0);
	m_pcMotionControl->ComputeSpeedFromForce(vect);
	m_pcMotionControl->Step(*c_robot_state);

	if(--timer == 0){
		pickedUp = false;
	}
}

void CBTFootbotCombinedNoVisRootBehavior::Explore() {

	if(m_sStateData.InNest) {
		ExitNest();
	}
	else {

		// Get vectors for obstacle avoidance and random walk
		m_pcObstacleAvoidance->Step(*c_robot_state);
		m_pcWalk->Step(*c_robot_state);


		// Calculate vector
		CVector2 tmp = m_pcObstacleAvoidance->GetVector();
		if(tmp.Length() != 0)
			tmp = AVOIDANCE_FACTOR *tmp.Normalize();
		CVector2 tmp2 = m_pcWalk->GetVector();
		if(tmp2.Length() != 0)
			tmp += tmp2.Normalize();
		if(tmp.Length() != 0)
			tmp.Normalize();

		// Apply vector
		GoToVector(tmp);
	}
}

void CBTFootbotCombinedNoVisRootBehavior::ExitNest() {

	m_pcObstacleAvoidance->Step(*c_robot_state);
	m_pcPhototaxis->SetAntiPhototaxis(true);
	m_pcPhototaxis->Step(*c_robot_state);

	CVector2 tmp = m_pcObstacleAvoidance->GetVector();
	if(tmp.Length() != 0)
		tmp = AVOIDANCE_FACTOR *tmp.Normalize();
	CVector2 tmp2 = m_pcPhototaxis->GetVector();
	if(tmp2.Length() != 0)
		tmp += tmp2.Normalize();
	if(tmp.Length() != 0)
		tmp.Normalize();

	GoToVector(tmp);
}

void CBTFootbotCombinedNoVisRootBehavior::GoToFood(){

	m_pcObstacleAvoidance->Step(*c_robot_state);

	CVector2 tmp = m_pcObstacleAvoidance->GetVector();
	if(tmp.Length() != 0)
		tmp = AVOIDANCE_FACTOR *tmp.Normalize();
	CVector2 tmp2 = m_pcOdometry->GetReversedLocationVector();
	if(tmp2.Length() != 0)
		tmp += tmp2.Normalize();
	if(tmp.Length() != 0)
		tmp.Normalize();

	GoToVector(tmp);
	m_pcOdometry->Step(*c_robot_state);
}

void CBTFootbotCombinedNoVisRootBehavior::ReturnToNest() {

	m_pcPhototaxis->SetAntiPhototaxis(false);
	m_pcPhototaxis->Step(*c_robot_state);
	m_pcObstacleAvoidance->Step(*c_robot_state);

	CVector2 tmp = m_pcObstacleAvoidance->GetVector();
	if(tmp.Length() != 0)
		tmp = AVOIDANCE_FACTOR *tmp.Normalize();
	CVector2 tmp2 = m_pcPhototaxis->GetVector();
	if(tmp2.Length() != 0)
		tmp += tmp2.Normalize();
	if(tmp.Length() != 0)
		tmp.Normalize();
	if(tmp.Length() == 0) // TODO fix beter ipv hackish (met stop methode in motion control)
		tmp += CVector2(1,CRadians(0));

	GoToVector(tmp);
	m_pcOdometry->Step(*c_robot_state);

}

/****************************************/
/** Helper methods					   **/
/****************************************/

void CBTFootbotCombinedNoVisRootBehavior::StartOdometry(){
	m_pcOdometry->Reset(*c_robot_state);
	m_pcOdometry->Start();
}

void CBTFootbotCombinedNoVisRootBehavior::StopOdometry(){
	m_pcOdometry->Stop();
}

bool CBTFootbotCombinedNoVisRootBehavior::InNest(){
	m_pcObserveGround->SetColor(CBTFootbotObserveGround::GRAY);
	m_pcObserveGround->Step(*c_robot_state);
	return m_pcObserveGround->OnColor();
}

void CBTFootbotCombinedNoVisRootBehavior::GoToVector(CVector2 vec){
	m_pcMotionControl->ComputeSpeedFromForce(vec);
	m_pcMotionControl->Step(*c_robot_state);
}

bool CBTFootbotCombinedNoVisRootBehavior::IsDoneLookingForFood(){
	CVector2 tmp = m_pcOdometry->GetReversedLocationVector();
	return tmp.Length() <= 7;
}

void CBTFootbotCombinedNoVisRootBehavior::ResetOdometry(){
	m_pcOdometry->Reset(*c_robot_state);
}




/****************************************/
/** state related methods
/****************************************/

CBTFootbotCombinedNoVisRootBehavior::SFoodData::SFoodData() :
												HasFoodItem(false),
												FoodItemIdx(0),
												TotalFoodItems(0),
												LastFoodPosition(0,0){
}

void CBTFootbotCombinedNoVisRootBehavior::SFoodData::Init(){
	Reset();
}

void CBTFootbotCombinedNoVisRootBehavior::SFoodData::Reset() {
	HasFoodItem = false;
	FoodItemIdx = 0;
	TotalFoodItems = 0;
	FoodPatchIdx = 0;
}

/*void CBTFootbotCombinedNoVisRootBehavior::UpdateFoodData(){
	//TODO statistics
}*/

/******/

CBTFootbotCombinedNoVisRootBehavior::SStateData::SStateData() : CurrentPosition(0,0){
	State = STATE_EXPLORING;
	InNest = true;
}

void CBTFootbotCombinedNoVisRootBehavior::SStateData::Init(/*TConfigurationNode& t_node*/) {
	Reset();
}

void CBTFootbotCombinedNoVisRootBehavior::SStateData::Reset() {
	State = STATE_EXPLORING;
	InNest = false;
}

void CBTFootbotCombinedNoVisRootBehavior::UpdateStateDataSolitary(){
	if(InNest())
		m_sStateData.InNest = true;
	else
		m_sStateData.InNest = false;

	if(m_sStateData.State == SStateData::STATE_DROP && pickedUp == false){
		if(USE_ODOMETRY){
			//m_pcControlLeds->SetAllLedsColor(*c_robot_state, CColor::RED);
			m_sStateData.State = SStateData::STATE_GO_TO_FOOD;
		}else{
			//m_pcControlLeds->SetAllLedsColor(*c_robot_state, CColor::BLACK);
			m_sStateData.State = SStateData::STATE_EXPLORING;
		}
	}else if(m_sStateData.State == SStateData::STATE_RETURN_TO_NEST && m_sStateData.InNest) {
		//m_pcControlLeds->SetAllLedsColor(*c_robot_state, CColor::WHITE);
		timer = PICKUP_TIME;
		m_sStateData.State = SStateData::STATE_DROP;
	}else if((m_sStateData.State == SStateData::STATE_EXPLORING || m_sStateData.State == SStateData::STATE_GO_TO_FOOD) && m_sFoodData.HasFoodItem) {
		//m_pcControlLeds->SetAllLedsColor(*c_robot_state, CColor::WHITE);
		timer = PICKUP_TIME;
		m_sStateData.State = SStateData::STATE_PICK_UP;
	}else if(m_sStateData.State == SStateData::STATE_PICK_UP && pickedUp){
		//m_pcControlLeds->SetAllLedsColor(*c_robot_state, CColor::GREEN);
		m_pcOdometry->Reset(*c_robot_state);
		m_pcOdometry->Start();
		m_sStateData.State = SStateData::STATE_RETURN_TO_NEST;
	}else if(m_sStateData.State == SStateData::STATE_GO_TO_FOOD && IsDoneLookingForFood()){
		//m_pcControlLeds->SetAllLedsColor(*c_robot_state, CColor::BLACK);
		m_sStateData.State = SStateData::STATE_EXPLORING;
	}
}

void CBTFootbotCombinedNoVisRootBehavior::UpdateStateDataRecruitee(){
	m_pcSignalling->Step(*c_robot_state);
	if(m_pcSignalling->SignalFound(*c_robot_state))
		signalFound = true;
	else
		signalFound = false;

	if(InNest())
		m_sStateData.InNest = true;
	else
		m_sStateData.InNest = false;

	if(m_sStateData.State == SStateData::STATE_EXPLORING && signalFound){
		//m_pcControlLeds->SetAllLedsColor(*c_robot_state, CColor::YELLOW);
		m_sStateData.State = SStateData::STATE_FOLLOW_SIGNAL;
	}else if(m_sStateData.State == SStateData::STATE_EXPLORING && m_sFoodData.HasFoodItem){
		//m_pcControlLeds->SetAllLedsColor(*c_robot_state, CColor::WHITE);
		timer = PICKUP_TIME;
		m_sStateData.State = SStateData::STATE_PICK_UP;
	}else if(m_sStateData.State == SStateData::STATE_FOLLOW_SIGNAL){
		if(m_sFoodData.HasFoodItem){
			//m_pcControlLeds->SetAllLedsColor(*c_robot_state, CColor::WHITE);
			timer = PICKUP_TIME;
			m_sStateData.State = SStateData::STATE_PICK_UP;
		}else if(closeToSignal){
			//m_pcControlLeds->SetAllLedsColor(*c_robot_state, CColor::BLUE);
			signalExploreTimer = SIGNAL_EXPLORE_TIME;
			m_sStateData.State = SStateData::STATE_EXPLORING_SIGNAL_AREA;
		}else if(!signalFound){
			//m_pcControlLeds->SetAllLedsColor(*c_robot_state, CColor::BLACK);
			m_sStateData.State = SStateData::STATE_EXPLORING;
		}
	}else if(m_sStateData.State == SStateData::STATE_EXPLORING_SIGNAL_AREA){
		if(m_sFoodData.HasFoodItem){
			//m_pcControlLeds->SetAllLedsColor(*c_robot_state, CColor::WHITE);
			closeToSignal = false;
			timer = PICKUP_TIME;
			m_sStateData.State = SStateData::STATE_PICK_UP;
		} else if(signalExploreTimer == 0){
			//m_pcControlLeds->SetAllLedsColor(*c_robot_state, CColor::BLACK);
			closeToSignal = false;
			m_sStateData.State = SStateData::STATE_EXPLORING;
		}
	}else if(m_sStateData.State == SStateData::STATE_PICK_UP && pickedUp){
		//m_pcControlLeds->SetAllLedsColor(*c_robot_state, CColor::GREEN);
		m_sStateData.State = SStateData::STATE_RETURN_TO_NEST;
	}else if(m_sStateData.State == SStateData::STATE_RETURN_TO_NEST && m_sStateData.InNest){
		//m_pcControlLeds->SetAllLedsColor(*c_robot_state, CColor::WHITE);
		timer = DROP_TIME;
		m_sStateData.State = SStateData::STATE_DROP;
	}else if(m_sStateData.State == SStateData::STATE_DROP && pickedUp == false){
		//m_pcControlLeds->SetAllLedsColor(*c_robot_state, CColor::BLACK);
		m_sStateData.State = SStateData::STATE_EXPLORING;
	}
}

void CBTFootbotCombinedNoVisRootBehavior::UpdateStateDataRecruiter(){
	if(InNest())
		m_sStateData.InNest = true;
	else
		m_sStateData.InNest = false;

	if(m_sStateData.State == SStateData::STATE_DROP && pickedUp == false){
		//m_pcControlLeds->SetAllLedsColor(*c_robot_state, CColor::RED);
		m_sStateData.State = SStateData::STATE_GO_TO_FOOD;
	}else if(m_sStateData.State == SStateData::STATE_RETURN_TO_NEST && m_sStateData.InNest) {
		//m_pcControlLeds->SetAllLedsColor(*c_robot_state, CColor::WHITE);
		timer = DROP_TIME;
		m_sStateData.State = SStateData::STATE_DROP;
	}else if((m_sStateData.State == SStateData::STATE_EXPLORING || m_sStateData.State == SStateData::STATE_GO_TO_FOOD) && m_sFoodData.HasFoodItem) {
		//m_pcControlLeds->SetAllLedsColor(*c_robot_state, CColor::WHITE);
		timer = SIGNAL_TIME;
		m_pcSignalling->StartSignal(*c_robot_state);
		m_sStateData.State = SStateData::STATE_SIGNAL_AND_PICK_UP;
	}else if(m_sStateData.State == SStateData::STATE_SIGNAL_AND_PICK_UP && pickedUp){
		//m_pcControlLeds->SetAllLedsColor(*c_robot_state, CColor::GREEN);
		m_pcOdometry->Reset(*c_robot_state);
		m_pcOdometry->Start();
		m_pcSignalling->StopSignal(*c_robot_state);
		m_sStateData.State = SStateData::STATE_RETURN_TO_NEST;
	}else if(m_sStateData.State == SStateData::STATE_GO_TO_FOOD && IsDoneLookingForFood()){
		//m_pcControlLeds->SetAllLedsColor(*c_robot_state, CColor::BLACK);
		m_sStateData.State = SStateData::STATE_EXPLORING;
	}
}




