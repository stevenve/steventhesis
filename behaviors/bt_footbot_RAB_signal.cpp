#include "bt_footbot_RAB_signal.h"
#include "float.h"

#define BEHAVIOR_NAME "[CBTFootbotRABSignal]: "

/****************************************/
/****************************************/

CBTFootbotRABSignal::CBTFootbotRABSignal(CCI_RobotData<CCI_FootBotState>* c_robot_data) :
CCI_Behavior<CCI_FootBotState> (c_robot_data, "bt_footbot_RAB_signal"){
	// Get data:
	m_pcRobotData = c_robot_data;
	m_fDistanceMax = 5000000.0f; // for RAB in cm's
	started = false;
	vectorToClosestSignal = CVector2(DBL_MAX,CRadians(0));
	signal = CByteArray(10,4);
	signalFound = false;
}

/****************************************/
/****************************************/

CBTFootbotRABSignal::~CBTFootbotRABSignal() {
}

/****************************************/
/****************************************/

void CBTFootbotRABSignal::Init(CCI_FootBotState& cRobotState) {
	m_fDistanceMax = 5000000.0f; // for RAB in cm's
	started = false;
	vectorToClosestSignal = CVector2(DBL_MAX,CRadians(0));
	signal = CByteArray(10,4);
	signalFound = false;
	ResetFSM();
}

/****************************************/
/****************************************/

void CBTFootbotRABSignal::Destroy(CCI_FootBotState& cRobotState) {

}

/****************************************/
/****************************************/

void CBTFootbotRABSignal::Reset(CCI_FootBotState& cRobotState) {
	Init(cRobotState);
}

/****************************************/
/****************************************/

void CBTFootbotRABSignal::Step(CCI_FootBotState& c_robot_state) {

	/* received packets*/
	std::vector<CCI_RangeAndBearingSensor::SPacket> cMyPackets;
	vectorToClosestSignal = CVector2(DBL_MAX,CRadians(0)); // positive infinity vector
	signalFound = false;

	/* store received heading angles, if any */
	cMyPackets = c_robot_state.GetRABLastReceivedPackets();
	if (cMyPackets.size() != 0) {
		for (std::vector<CCI_RangeAndBearingSensor::SPacket>::iterator it = cMyPackets.begin(); it != cMyPackets.end(); it++) {
			if ((*it).Range < m_fDistanceMax && (*it).Range < vectorToClosestSignal.Length() && (*it).Data[0] == signal[0]) {
				signalFound = true;
				vectorToClosestSignal = CVector2((*it).Range, (*it).HorizontalBearing);
			}
		}
	}
}

void CBTFootbotRABSignal::StartSignal(CCI_FootBotState& c_robot_state) {
	c_robot_state.SetRABPacketDataToSend(signal);
}

void CBTFootbotRABSignal::StopSignal(CCI_FootBotState& c_robot_state) {
	c_robot_state.ClearRABPacketData();
}

CVector2& CBTFootbotRABSignal::GetVectorToClosestSignal(CCI_FootBotState& cRobotState) {
	return vectorToClosestSignal;
}

bool CBTFootbotRABSignal::SignalFound(CCI_FootBotState& c_robot_state){
	return signalFound;
}

void CBTFootbotRABSignal::SetMaxDistance(Real dist){
	m_fDistanceMax = dist;
}



