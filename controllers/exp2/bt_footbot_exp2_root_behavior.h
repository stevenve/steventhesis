#ifndef CBTFootbotExp2RootBehavior_H_
#define CBTFootbotExp2RootBehavior_H_

#include <argos3/core/utility/math/rng.h>
#include "../../BTSimple/ci_behavior.h"
#include "../../BTSimple/swarmanoid/footbot/ci_footbot_state.h"
#include "../../BTSimple/utility_classes/fsm/fsm.h"

#include "../../behaviors/bt_footbot_randomwalk.h"
#include "../../behaviors/bt_footbot_go_to_led.h"
#include "../../behaviors/bt_footbot_phototaxis.h"
#include "../../behaviors/bt_footbot_obstacle_avoidance.h"
#include "../../behaviors/bt_footbot_observe_ground.h"
#include "../../behaviors/bt_footbot_control_leds.h"
#include "../../behaviors/bt_footbot_odometry.h"

using namespace argos;
using namespace btfsm;

class CBTFootbotExp2RootBehavior: public CCI_Behavior<CCI_FootBotState> , public FSM<std::string> {

private:

	/* **************** */
	/* 	 SUB-BEHAVIORS  */
	/* **************** */

	CBTFootbotRandomWalk* m_pcWalk;
	CBTFootbotGoToLED* m_pcGoToLED;
	CBTFootbotPhototaxis* m_pcPhototaxis;
	CBTFootbotObstacleAvoidance* m_pcObstacleAvoidance;
	CBTFootbotMotionControl* m_pcMotionControl;
	CBTFootbotObserveGround* m_pcObserveGround;
	CBTFootbotControlLeds* m_pcControlLeds;
	CBTFootbotOdometry* m_pcOdometry;

	CCI_FootBotState* c_robot_state;

public:

	CBTFootbotExp2RootBehavior(CCI_RobotData<CCI_FootBotState>* c_robot_data);
	virtual ~CBTFootbotExp2RootBehavior();

	virtual void Init(CCI_FootBotState& c_robot_state);
	virtual void Step(CCI_FootBotState& c_robot_state);

	virtual void Destroy(CCI_FootBotState& c_robot_state);
	virtual void Reset(CCI_FootBotState& c_robot_state);
	virtual void ReturnToNest();
	virtual void Explore();
	virtual void ExitNest();
	virtual bool InNest();
	virtual void GoToVector(CVector2);
	virtual void StartOdometry();
	virtual void StopOdometry();
	virtual void GoToFood();
	virtual bool IsDoneLookingForFood();
	virtual void ResetOdometry();

	/*
	 * Contains all the state information about the controller.
	 */
	struct SStateData {
		enum EState {
			STATE_EXPLORING,
			STATE_RETURN_TO_NEST,
			STATE_GO_TO_FOOD
		} State;

		bool InNest;
		CVector2 CurrentPosition;

		SStateData();
		void Init();
		void Reset();
	};

	struct SFoodData {
		bool HasFoodItem;      // true when the robot is carrying a food item
		size_t FoodItemIdx;    // the index of the current food item in the array of available food items
		size_t TotalFoodItems; // the total number of food items carried by this robot during the experiment
		CVector2 LastFoodPosition;

		SFoodData();
		void Init();
		void Reset();
	};


	inline bool IsExploring() const {return m_sStateData.State == SStateData::STATE_EXPLORING;}
	inline bool IsReturningToNest() const {return m_sStateData.State == SStateData::STATE_RETURN_TO_NEST;}
	inline bool IsGoingTofood() const {return m_sStateData.State == SStateData::STATE_GO_TO_FOOD;}

	inline SFoodData& GetFoodData() {return m_sFoodData;}
	inline SStateData& GetStateData() {return m_sStateData;}


	void Rest(CCI_FootBotState& c_robot_state);
	void Explore(CCI_FootBotState& c_robot_state);
	void ReturnToNest(CCI_FootBotState& c_robot_state);
	void GoToFood(CCI_FootBotState& c_robot_state);


private:

	SFoodData m_sFoodData;
	SStateData m_sStateData;

	void UpdateStateData();
	void UpdateFoodData();

};

#endif /* CBTFootbotExp2RootBehavior_H_ */
