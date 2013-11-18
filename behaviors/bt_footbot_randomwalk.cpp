#include "bt_footbot_randomwalk.h"
#define BEHAVIOR_NAME "[CBTFootbotRandomWalk]: "

/****************************************/
/****************************************/

CBTFootbotRandomWalk::CBTFootbotRandomWalk(CCI_RobotData<CCI_FootBotState>* c_robot_data) :
			CCI_Behavior<CCI_FootBotState> (c_robot_data, "bt_footbot_random_walk"),
			m_cRandomAngle(-0.0000001f, 0.0000001f){

	m_pcRNG = NULL;

	m_pcRobotData= c_robot_data;
	m_fNoiseFactor = 0.6f;

	m_cRandomAngle.Set(-m_fNoiseFactor * CRadians::PI.GetValue(), m_fNoiseFactor * CRadians::PI.GetValue());
	m_pcMotionControl = new CBTFootbotMotionControl(c_robot_data);
	//m_pcMotionControl->SetBiasForwardSpeed(2.0f);

}

/****************************************/
/****************************************/

CBTFootbotRandomWalk::~CBTFootbotRandomWalk() {
	delete m_pcMotionControl;
}

/****************************************/
/****************************************/

void CBTFootbotRandomWalk::Init(CCI_FootBotState& cRobotState) {
	// Reset the FSM
	m_pcRNG = CRandom::CreateRNG("argos");
	ResetFSM();
	m_pcMotionControl->Init(cRobotState);
}

/****************************************/
/****************************************/

void CBTFootbotRandomWalk::Destroy(CCI_FootBotState& cRobotState) {
	m_pcMotionControl->Destroy(cRobotState);
}

/****************************************/
/****************************************/

void CBTFootbotRandomWalk::Reset(CCI_FootBotState& cRobotState) {

	Destroy(cRobotState);
	Init(cRobotState);
}

/****************************************/
/****************************************/

void CBTFootbotRandomWalk::Step(CCI_FootBotState& cRobotState) {
	//CCI_FootBotState cRobotStateCopy = cRobotState;

	CRadians cRandomDirection;
	Real fLength = 1.0f;

	Real fRandomDirection = m_pcRNG->Uniform(m_cRandomAngle);
	cRandomDirection.SetValue(fRandomDirection);

	//m_cRandomWalkDirection.SetFromAngleAndLength(cRandomDirection, fLength);
	m_cRandomWalkDirection.FromPolarCoordinates(fLength, cRandomDirection);
}

void CBTFootbotRandomWalk::StepAndGo(CCI_FootBotState& cRobotState) {
	Step(cRobotState);

	m_pcMotionControl->ComputeSpeedFromForce(m_cRandomWalkDirection);
	m_pcMotionControl->Step(cRobotState);
}

/****************************************/
/****************************************/
CVector2 CBTFootbotRandomWalk::GetVector(){
	return m_cRandomWalkDirection;
}



/****************************************/
/****************************************/
