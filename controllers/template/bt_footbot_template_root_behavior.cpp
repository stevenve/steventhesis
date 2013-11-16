#include <argos2/common/utility/logging/argos_log.h>
#include <argos2/common/utility/datatypes/color.h>
#include <argos2/common/utility/math/angles.h>
#include <argos2/user/eliseofe/controllers/bt_footbot_template_controller/bt_footbot_template_controller.h>


/****************************************/
/****************************************/

//#define DEBUG_FSM(format, ...) fprintf( stderr, "[DEBUG-FSM] [robot id=\"%s\"] " format, cRobotState.GetRobotId().c_str(), ## __VA_ARGS__ )
#define DEBUG_FSM(format, ...)

//#define DEBUG_RAB(message) { LOG << message << std::endl; }
#define DEBUG_RAB(message)

//#define DEBUG(logger,message) { logger << "[CBTFootbotOpinionDynamicsRootBehavior]: " << message << std::endl; }
#define DEBUG(logger,message)

/****************************************/
/****************************************/

CBTFootbotTemplateRootBehavior::CBTFootbotTemplateRootBehavior(CCI_RobotData<CCI_FootBotState>* c_robot_data) :
	CCI_Behavior<CCI_FootBotState> (c_robot_data, "bt_footbot_template_root_behavior") {
	//m_pcObstacleAvoidance = new CBTFootbotSingleRobotObstacleAvoidance(c_robot_data);
	m_pcMotionControl = new CBTFootbotMotionControl(c_robot_data);
	m_pcGetLight = new CBTFootbotGetLightDirection(c_robot_data);

	m_pcMotionControl->SetForwardVelocity(5.0);
	m_pcMotionControl->SetProportionalFactor(5.0);

	m_pcUtility = new CCtFootbotUtility();

	m_bInformed = false;

}

/****************************************/
/****************************************/

CBTFootbotTemplateRootBehavior::~CBTFootbotTemplateRootBehavior() {

}

/****************************************/
/****************************************/

void CBTFootbotTemplateRootBehavior::Init(CCI_FootBotState& c_robot_state) {
	//m_pcObstacleAvoidance->Init(cRobotState);
	m_pcMotionControl->Init(c_robot_state);
	m_pcGetLight->Init(c_robot_state);

}

/****************************************/
/****************************************/

void CBTFootbotTemplateRootBehavior::Step(CCI_FootBotState& c_robot_state) {

	c_robot_state.SetAllLedsColor(CColor::BLACK);
	c_robot_state.SetBeaconColor(CColor::BLACK);

	//LOG << "Robot " << c_robot_state.GetRobotId() << std::endl;

	c_robot_state.ClearRABPacketData();
	m_vReferenceVector = m_pcGetLight->GetLightDirection();


	/* compute speed from force */
	//m_pcMotionControl->ComputeSpeedFromVector(m_vForceVector);
	//m_pcMotionControl->ComputeSpeedFromForce(c_robot_state, m_vForceVector);


	/* apply speed */
	m_pcMotionControl->Step(c_robot_state);

	c_robot_state.ClearRABReceivedPackets();

}

/****************************************/
/****************************************/

CRadians CBTFootbotTemplateRootBehavior::GetOrientation(){
	return -m_vReferenceVector.Angle(); // The orientation of the robot is the
																			// negate of the light measurement (because it's
																			// from axis to robot and the measurement is from robot to axis)
}

/****************************************/
/****************************************/

void CBTFootbotTemplateRootBehavior::SetInformed(bool b_informed){
	m_bInformed = b_informed;
}

/****************************************/
/****************************************/

void CBTFootbotTemplateRootBehavior::Destroy(CCI_FootBotState& c_robot_state) {
	//m_pcObstacleAvoidance->Destroy(c_robot_state);
	m_pcMotionControl->Destroy(c_robot_state);
	m_pcGetLight->Destroy(c_robot_state);

	//delete m_pcObstacleAvoidance;
	delete m_pcMotionControl;
	delete m_pcGetLight;
}

/****************************************/
/****************************************/

void CBTFootbotTemplateRootBehavior::Reset(CCI_FootBotState& c_robot_state) {
	Destroy(c_robot_state);
	Init(c_robot_state);
}

/****************************************/
/****************************************/
