#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/utility/math/angles.h>

#define BEHAVIOR_NAME "[CBTFootbotPhototaxis]: "

#include "bt_footbot_phototaxis.h"

//#define DEBUG(message) { LOG << BEHAVIOR_NAME << message << std::endl; }
#define DEBUG(message)
//#define DEBUG_STATE() { LOG << BEHAVIOR_NAME << "[STATE] " << GetStateName() << std::endl; }
#define DEBUG_STATE()


/****************************************/
/****************************************/

CBTFootbotPhototaxis::CBTFootbotPhototaxis(CCI_RobotData<CCI_FootBotState>* c_robot_data) :
	CCI_Behavior<CCI_FootBotState> (c_robot_data, "bt_footbot_get_light_direction")
	//m_cRandomNoiseRange(-0.0000001, 0.0000001), m_cRandomNoiseGoalRange(-0.0000001, 0.0000001)
	{

	m_cLightDirection.Set(0.0,0.0);
	//m_cInformationDirection = CRadians::ZERO;

	//m_fNoiseFactor = 0.0000001;
	//m_fNoiseGoalFactor = 0.0000001;
	//m_cRandomNoiseRange.Set(-m_fNoiseFactor * CRadians::PI.GetValue(), m_fNoiseFactor * CRadians::PI.GetValue());
	//m_cRandomNoiseGoalRange.Set(-m_fNoiseFactor * CRadians::PI.GetValue(), m_fNoiseFactor * CRadians::PI.GetValue());
	//m_pcRNG = CRandom::CreateRNG("argos");
	m_pcMotionControl = new CBTFootbotMotionControl(c_robot_data);
	//m_pcMotionControl->SetBiasForwardSpeed(2.0f);

	anti = false;
}

/****************************************/
/****************************************/

CBTFootbotPhototaxis::~CBTFootbotPhototaxis() {
	delete m_pcMotionControl;
}

/****************************************/
/****************************************/

void CBTFootbotPhototaxis::Init(CCI_FootBotState& c_robot_state) {
	/* create random generator */
	//m_pcRNG = CRandom::CreateRNG("argos");

	DEBUG("Init()");
	m_pcMotionControl->Init(c_robot_state);
}

/****************************************/
/****************************************/

void CBTFootbotPhototaxis::Step(CCI_FootBotState& c_robot_state) {
	m_cLightDirection.Set(0.0,0.0);
	CCI_FootBotLightSensor::TReadings tReadings = c_robot_state.GetLightSensorReadings();
	for(UInt32 i=0; i < tReadings.size();i++){ // ipv size wasset CCI_FootBotLightSensor::NUM_READINGS
		CVector2 vPointReading;
		if (tReadings[i].Value > 0.0) {
			vPointReading.FromPolarCoordinates(tReadings[i].Value, tReadings[i].Angle);
			m_cLightDirection += vPointReading;
			//LOGERR << "Angle: " << tAngles[i] << " , reading: " << tReadings[i] << std::endl;
		}
	}

	if(anti)
		m_cLightDirection = -m_cLightDirection;
	/*Real fNoise = m_pcRNG->Uniform(m_cRandomNoiseRange);
	m_cLightDirection.FromPolarCoordinates(m_cLightDirection.Length(), m_cLightDirection.Angle() + CRadians(fNoise));

	Real fNoiseGoal = m_pcRNG->Uniform(m_cRandomNoiseGoalRange);
	m_cInformationVector.FromPolarCoordinates(1.0, ( m_cInformationDirection - GetOrientation() + CRadians(fNoiseGoal) ));*/
}

void CBTFootbotPhototaxis::StepAndGo(CCI_FootBotState& cRobotState) {
	Step(cRobotState);

	m_pcMotionControl->ComputeSpeedFromForce(m_cLightDirection);
	m_pcMotionControl->Step(cRobotState);
}

/****************************************/
/****************************************/

CVector2& CBTFootbotPhototaxis::GetVector(){
	return m_cLightDirection;
}

/****************************************/
/****************************************/

void CBTFootbotPhototaxis::Destroy(CCI_FootBotState& c_robot_state) {
	//delete m_pcRNG;
	m_pcMotionControl->Destroy(c_robot_state);
}

/****************************************/
/****************************************/

void CBTFootbotPhototaxis::Reset(CCI_FootBotState& c_robot_state) {
	Destroy(c_robot_state);
	Init(c_robot_state);
}

/****************************************/
/****************************************/

/*void CBTFootbotPhototaxis::SetInformationDirection(CRadians& c_information_direction){
	m_cInformationDirection = c_information_direction;
}*/

/****************************************/
/****************************************/

/*CVector2& CBTFootbotPhototaxis::GetInformationVector() {
	return m_cInformationVector;
}*/

/****************************************/
/****************************************/

CRadians CBTFootbotPhototaxis::GetOrientation() {
	return -m_cLightDirection.Angle();
}

/****************************************/
/****************************************/

/*void CBTFootbotPhototaxis::SetNoiseFactor(Real fNoise){
	m_fNoiseFactor = fNoise;
	m_cRandomNoiseRange.Set(-m_fNoiseFactor * CRadians::PI.GetValue(), m_fNoiseFactor * CRadians::PI.GetValue());
}*/

/****************************************/
/****************************************/

/*void CBTFootbotPhototaxis::SetNoiseFactorGoal(Real fNoise){
	m_fNoiseGoalFactor = fNoise;
	m_cRandomNoiseGoalRange.Set(-m_fNoiseGoalFactor* CRadians::PI.GetValue(), m_fNoiseGoalFactor * CRadians::PI.GetValue());
}*/

/****************************************/
/****************************************/

void CBTFootbotPhototaxis::SetAntiPhototaxis(bool anti2){
	anti = anti2;
}
