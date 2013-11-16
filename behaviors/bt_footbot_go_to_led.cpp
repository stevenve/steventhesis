//#include <argos2/common/utility/logging/argos_log.h>
#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/utility/math/angles.h>

#define BEHAVIOR_NAME "[CBTFootbotGoToSource]: "

#include "bt_footbot_go_to_led.h"

//#define DEBUG(message) { LOG << BEHAVIOR_NAME << message << std::endl; }
#define DEBUG(message)
//#define DEBUG_STATE() { LOG << BEHAVIOR_NAME << "[STATE] " << GetStateName() << std::endl; }
#define DEBUG_STATE()


/****************************************/
/****************************************/

CBTFootbotGoToLED::CBTFootbotGoToLED(CCI_RobotData<CCI_FootBotState>* c_robot_data) :
    CCI_Behavior<CCI_FootBotState> (c_robot_data, "bt_footbot_go_to_source") {
    m_cAngleFromSource = CRadians::ZERO;
    m_fDistanceFromLED = 100000.0;
    m_color = CColor::YELLOW;
    m_cCameraBlobs.clear();
    m_pcMotionControl = new CBTFootbotMotionControl(c_robot_data);
    vector = CVector2();
    //m_pcMotionControl->SetBiasForwardSpeed(3.0f);
}

/****************************************/
/****************************************/

CBTFootbotGoToLED::~CBTFootbotGoToLED() {
    delete m_pcMotionControl;
}

/****************************************/
/****************************************/

void CBTFootbotGoToLED::Init(CCI_FootBotState& c_robot_state) {
    m_pcMotionControl->Init(c_robot_state);
}

/****************************************/
/****************************************/

void CBTFootbotGoToLED::Step(CCI_FootBotState& c_robot_state) {

	m_cCameraBlobs = c_robot_state.GetOmnidirectionalCameraReadings().BlobList;

	for (unsigned int i = 0; i < m_cCameraBlobs.size(); i++) {
		if (m_cCameraBlobs[i]->Color == m_color) {
			m_cAngleFromSource = m_cCameraBlobs[i]->Angle;
			m_fDistanceFromLED = m_cCameraBlobs[i]->Distance;
		}
	}

	m_cAngleFromSource.SignedNormalize();

	//LOGERR << "Angle is " << m_cAngleFromSource << std::endl;

	vector = CVector2(1.0,m_cAngleFromSource);
}

void CBTFootbotGoToLED::StepAndGo(CCI_FootBotState& c_robot_state) {

	Step(c_robot_state);

	m_pcMotionControl->ComputeSpeedFromForce(vector);
	m_pcMotionControl->Step(c_robot_state);
}



/****************************************/
/****************************************/

void CBTFootbotGoToLED::Destroy(CCI_FootBotState& c_robot_state) {
    m_pcMotionControl->Destroy(c_robot_state);
}

/****************************************/
/****************************************/

void CBTFootbotGoToLED::Reset(CCI_FootBotState& c_robot_state) {
    Destroy(c_robot_state);
    Init(c_robot_state);
}

/****************************************/
/****************************************/
