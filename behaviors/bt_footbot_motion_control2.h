#ifndef BT_FOOTBOT_MOTION_CONTROL_H
#define BT_FOOTBOT_MOTION_CONTROL_H

#include <argos3/core/utility/math/rng.h>
#include "../BTSimple/ci_behavior.h"
#include "../BTSimple/swarmanoid/footbot/ci_footbot_state.h"
#include "../BTSimple/utility_classes/fsm/fsm.h"
#include <argos3/core/utility/math/vector2.h>

using namespace argos;
using namespace btfsm;

class CBTFootbotMotionControl: public CCI_Behavior<CCI_FootBotState> , public FSM<std::string> {

	   /*
	       * The following variables are used as parameters for
	       * turning during navigation. You can set their value
	       * in the <parameters> section of the XML configuration
	       * file, under the
	       * <controllers><footbot_foraging_controller><parameters><wheel_turning>
	       * section.
	       */
	      struct SWheelTurningParams {
	         /*
	          * The turning mechanism.
	          * The robot can be in three different turning states.
	          */
	         enum ETurningMechanism
	         {
	            NO_TURN = 0, // go straight
	            SOFT_TURN,   // both wheels are turning forwards, but at different speeds
	            HARD_TURN    // wheels are turning with opposite speeds
	         } TurningMechanism;
	         /*
	          * Angular thresholds to change turning state.
	          */
	         CRadians HardTurnOnAngleThreshold;
	         CRadians SoftTurnOnAngleThreshold;
	         CRadians NoTurnAngleThreshold;
	         /* Maximum wheel speed */
	         Real MaxSpeed;

	         void Init();
	      };


public:

		CBTFootbotMotionControl(CCI_RobotData<CCI_FootBotState>* c_robot_data);
    virtual ~CBTFootbotMotionControl();

    virtual void Init(CCI_FootBotState& c_robot_state);
    virtual void Step(CCI_FootBotState& c_robot_state);

    virtual void Destroy(CCI_FootBotState& c_robot_state);
    virtual void Reset(CCI_FootBotState& c_robot_state);

    /*
       * Gets a direction vector as input and transforms it into wheel actuation.
       */
      void ComputeSpeedFromForce(CVector2& c_heading);
      void GetComputedWheelsSpeed(Real* fLeftSpeed, Real* fRightSpeed);

private:

      Real fLeftWheelSpeed, fRightWheelSpeed;

      /* Maximum tolerance for the angle between
          * the robot heading direction and
          * the closest obstacle detected. */
         CDegrees m_cAlpha;
         /* Maximum tolerance for the proximity reading between
          * the robot and the closest obstacle.
          * The proximity reading is 0 when nothing is detected
          * and grows exponentially to 1 when the obstacle is
          * touching the robot.
          */
         Real m_fDelta;
         /* Wheel speed. */
         Real m_fWheelVelocity;
         /* Angle tolerance range to go straight.
          * It is set to [-alpha,alpha]. */
         CRange<CRadians> m_cGoStraightAngleRange;

         /* The turning parameters */
         SWheelTurningParams m_sWheelTurningParams;

};

#endif /* CBTFootbotMotionControl_H_ */

