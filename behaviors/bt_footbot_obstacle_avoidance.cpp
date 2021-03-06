#include "bt_footbot_obstacle_avoidance.h"

#define BEHAVIOR_NAME "[CBTFootbotObstacleAvoidance]: "

/****************************************/
/****************************************/

CBTFootbotObstacleAvoidance::CBTFootbotObstacleAvoidance(CCI_RobotData<CCI_FootBotState>* c_robot_data) :
			CCI_Behavior<CCI_FootBotState> (c_robot_data, "bt_footbot_obstacle_avoidance"){
	// Get data:
	m_pcRobotData = c_robot_data;
	m_cObstacleRepulsionVector.Set(0.0f, 0.0f);
	m_cObstacleRepulsionVectorRAB.Set(0.0f, 0.0f);
	m_cObstacleRepulsionVectorTotal.Set(0.0f, 0.0f);
	//RESCALED_CENTER_DISTANCE = CENTER_DISTANCE / DISTANCE_MAX;
	m_fDistanceMax = 100.0f; // for RAB in cm's
	m_fCenterDistance = 1.0f; // set to maximum to have repulsion all the time
}

/****************************************/
/****************************************/

CBTFootbotObstacleAvoidance::~CBTFootbotObstacleAvoidance() {
}

/****************************************/
/****************************************/

void CBTFootbotObstacleAvoidance::Init(CCI_FootBotState& cRobotState) {
	/* create random generator */
	// Reset the FSM
	ResetFSM();
}

/****************************************/
/****************************************/

void CBTFootbotObstacleAvoidance::Destroy(CCI_FootBotState& cRobotState) {

}

/****************************************/
/****************************************/

void CBTFootbotObstacleAvoidance::Reset(CCI_FootBotState& cRobotState) {

	Destroy(cRobotState);
	Init(cRobotState);
}

/****************************************/
/****************************************/

void CBTFootbotObstacleAvoidance::Step(CCI_FootBotState& cRobotState) {

	/* debug */

	m_cObstacleRepulsionVector.Set(0.0f, 0.0f);
	m_cObstacleRepulsionVectorRAB.Set(0.0f, 0.0f);

	/* proximity sensors */
	CCI_FootBotProximitySensor::TReadings tReadings = cRobotState.GetProximitySensorReadings();
	Real fProximityForce;
	CVector2 cProximityForceVector;

	for(UInt32 i=0; i<tReadings.size();i++){ // was CCI_FootBotProximitySensor::NUM_READINGS
		CVector2 cProximityReading;
		if (tReadings[i].Value > 0.0f) {
			Real fRescaledReadings = -(tReadings[i].Value - 1);
			cProximityReading.FromPolarCoordinates(fRescaledReadings, tReadings[i].Angle);

			fProximityForce = CalculateTannerPotentialForce(fRescaledReadings, m_fCenterDistance);
			cProximityForceVector.FromPolarCoordinates(fProximityForce, tReadings[i].Angle);
			m_cObstacleRepulsionVector += cProximityForceVector;
		}
	}

	/* RAB */
	StepRAB(cRobotState);

	//StepCamera(cRobotState);
	//Real fNoise = m_pcRNG->Uniform(m_cRandomNoiseRange);
	//m_cObstacleRepulsionVector.SetFromAngleAndLength(m_cObstacleRepulsionVector.Angle() + CRadians(fNoise),m_cObstacleRepulsionVector.Length());
}

/****************************************/
/****************************************/

void CBTFootbotObstacleAvoidance::StepRAB(CCI_FootBotState& c_robot_state){
	//m_cProximalControlVector.Set(0.0, 0.0);
	/* received packets*/
	std::vector<CCI_RangeAndBearingSensor::SPacket> cMyPackets;
	Real fProximityForce;
	CVector2 cProximityForceVector;

	/* store received heading angles, if any */
	cMyPackets = c_robot_state.GetRABLastReceivedPackets();
	if (cMyPackets.size() != 0) {
		for (std::vector<CCI_RangeAndBearingSensor::SPacket>::iterator it = cMyPackets.begin(); it != cMyPackets.end(); it++) {
			CVector2 cProximityReading;
			if ((*it).Range < m_fDistanceMax) {
				cProximityReading.FromPolarCoordinates((*it).Range / m_fDistanceMax, (*it).HorizontalBearing);
				fProximityForce = CalculateTannerPotentialForce((*it).Range / m_fDistanceMax, m_fCenterDistance);
				cProximityForceVector.FromPolarCoordinates(fProximityForce, (*it).HorizontalBearing);
				m_cObstacleRepulsionVectorRAB += cProximityForceVector;

			}
		}

	}
}

/****************************************/
/****************************************/

Real CBTFootbotObstacleAvoidance::CalculateTannerPotentialForce(Real f_distance, Real f_desired_distance) {
	/* Tanner Lennard - Jones type potential function */
	/* du / dr = -2 * (desired_distance) ^ 2 / r ^ 3 + 2 /r */
	const Real LOWER_DISTANCE = 0.3f; //original (0.35)
	//const Real LOWER_DISTANCE = 0.2;

	//const Real MAXIMUM_FORCE = -15.23; /* should be repulsive! the original value*/
	const Real MAXIMUM_FORCE = -67.4f; /*-52.5f should be repulsive!*/

	if(f_distance < LOWER_DISTANCE){
		return MAXIMUM_FORCE;
	}
	else if(f_distance > f_desired_distance){
		return 0.0;
	}
	else {
		return (-2.0f * f_desired_distance * f_desired_distance / (f_distance * f_distance * f_distance) + 2.0f / f_distance);
	}
}

/****************************************/
/****************************************/

Real CBTFootbotObstacleAvoidance::CalculateLennardJonesPotentialForce(Real f_distance, Real f_desired_distance) {
	/* Lennard - Jones potential function */
	/* u = 4 * 0.3 *  [ (0.6 / r) ^ 12 - (0.6 / r) ^ 6 ]*/
	/* du / dr = 4 * 0.3 * [ -12 * 0.6 ^ 12 / r ^ 13 + 6 * 0.6 ^ 6 / r ^ 7 ] */
	const Real LOWER_DISTANCE = 0.55f;
	const Real MAXIMUM_FORCE = -52.32f; /* should be repulsive!*/
	//const Real SMALL_DIFFERENCE = 0.05;

	if(f_distance <= LOWER_DISTANCE){
		return MAXIMUM_FORCE;
	}
	else if(f_distance >= f_desired_distance){
		return 0.0;
	}
	else{
		return (4 * 0.3 * ((-12 * pow(f_desired_distance, 12.0) / pow(f_distance, 13.0)) + 6 * pow(f_desired_distance, 6.0) / pow(f_distance, 7.0)) );
	}
}

/****************************************/
/****************************************/

CVector2& CBTFootbotObstacleAvoidance::GetVector() {
	m_cObstacleRepulsionVectorTotal = m_cObstacleRepulsionVector + 1.0f * m_cObstacleRepulsionVectorRAB;
	return m_cObstacleRepulsionVectorTotal;
}

CVector2& CBTFootbotObstacleAvoidance::GetObstacleRepulsionVector() {
	return m_cObstacleRepulsionVector;
}

CVector2& CBTFootbotObstacleAvoidance::GetRABRepulsionVector() {
	return m_cObstacleRepulsionVectorRAB;
}

void CBTFootbotObstacleAvoidance::SetMaxDistance(Real dist) {
	m_fDistanceMax = dist;
}

/****************************************/
/****************************************/
