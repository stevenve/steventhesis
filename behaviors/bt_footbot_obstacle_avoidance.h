#ifndef BT_FOOTBOT_OBSTACLE_AVOIDANCE_H_
#define BT_FOOTBOT_OBSTACLE_AVOIDANCE_H_

#include "../BTSimple/ci_behavior.h"
#include "../BTSimple/swarmanoid/footbot/ci_footbot_state.h"
#include "../BTSimple/utility_classes/fsm/fsm.h"
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/rng.h>
//#include "../common/rab_functions.h"
//#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>

using namespace argos;
using namespace btfsm;

class CBTFootbotObstacleAvoidance: public CCI_Behavior<CCI_FootBotState> , public FSM<std::string> {
	public:

		CBTFootbotObstacleAvoidance(CCI_RobotData<CCI_FootBotState>* c_robot_data);
		virtual ~CBTFootbotObstacleAvoidance();

		virtual void Init(CCI_FootBotState& c_robot_state);

		virtual void Step(CCI_FootBotState& cRobotState);

//		virtual void StepCamera(CCI_FootBotState& cRobotState);

//		virtual CRadians& GetObstacleRepulsionAngle();
		virtual CVector2& GetVector();

//		virtual void SetNoiseFactor(Real fNoise);

		virtual void Destroy(CCI_FootBotState& cRobotState);

		virtual void Reset(CCI_FootBotState& cRobotState);

		/* Maximum distance detected by the camera*/
//		static const Real DISTANCE_MAX = 100; //[cm]
//		static const Real CENTER_DISTANCE = 80; // [cm] original value

//		Real RESCALED_CENTER_DISTANCE;

	protected:

		virtual void StepRAB(CCI_FootBotState& c_robot_state);

//		Real m_fNoiseFactor;
		/** Random Numbers Generator */
//		CARGoSRandom::CRNG* m_pcRNG;

		/** Range for the random noise */
//		CRange<Real> m_cRandomNoiseRange;

//		CRadians m_cObstacleRepulsionAngle;
		CVector2 m_cObstacleRepulsionVector;
		CVector2 m_cObstacleRepulsionVectorRAB;
		CVector2 m_cObstacleRepulsionVectorTotal;

		Real m_fDistanceMax;

		/* Camera readings */
//		CCI_CameraSensor::TBlobList m_cCameraBlobs;

		Real m_fCenterDistance;
		virtual Real CalculateTannerPotentialForce(Real f_distance, Real f_desired_distance);
		virtual Real CalculateLennardJonesPotentialForce(Real f_distance, Real f_desired_distance);
//		virtual bool CheckCameraReading(UInt32 unCurrentBlob);

};

#endif /* BT_FOOTBOT_OBSTACLE_AVOIDANCE_H_ */
