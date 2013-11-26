#ifndef _CBTFOOTBOT_RECRUITMENT_CONTROLLER_H_
#define _CBTFOOTBOT_RECRUITMENT_CONTROLLER_H_

#include "../../BTSimple/ci_behavior.h"
#include "../../BTSimple/swarmanoid/footbot/ci_footbot_state.h"

#include "bt_footbot_recruitment_root_behavior.h"



class CBTFootbotRecruitmentController: public CCI_BehaviorController<CCI_FootBotState>
{



public:

    virtual void Init ( TConfigurationNode& t_tree);

    virtual void ControlStep (  );

    virtual void Destroy (  );

    virtual UInt32 GetRootBehaviorStateID();

    virtual std::string GetRootBehaviorStateName();

    virtual CBTFootbotRecruitmentRootBehavior* GetRootBehavior(){
        return (CBTFootbotRecruitmentRootBehavior*)m_pcRootBehavior;
    }

    /*
    	    * Contains all the state information about the controller.
    	    */
    	   struct SStateData {
    	      /* The three possible states in which the controller can be */
    	      enum EState {
    	         STATE_EXPLORING,
    	         STATE_RETURN_TO_NEST,
    	         STATE_GO_TO_FOOD
    	      } State;

    	      /* True when the robot is in the nest */
    	      bool InNest;
    	      CVector2 CurrentPosition;

    	      SStateData();
    	      void Init(TConfigurationNode& t_node);
    	      void Reset();
    	   };

    struct SFoodData {
          bool HasFoodItem;      // true when the robot is carrying a food item
          size_t FoodItemIdx;    // the index of the current food item in the array of available food items
          size_t TotalFoodItems; // the total number of food items carried by this robot during the experiment
          CVector2 LastFoodPosition;

          SFoodData();
          void Reset();
     };

    /*
     * Returns true if the robot is currently exploring.
     */
    inline bool IsExploring() const {
       return m_sStateData.State == SStateData::STATE_EXPLORING;
    }

    /*
     * Returns true if the robot is currently returning to the nest.
     */
    inline bool IsReturningToNest() const {
       return m_sStateData.State == SStateData::STATE_RETURN_TO_NEST;
    }

    /*
     * Returns the food data
     */
    inline SFoodData& GetFoodData() {
       return m_sFoodData;
    }

    /*
         * Returns the food data
         */
        inline SStateData& GetStateData() {
           return m_sStateData;
        }

    /*
     * Executes the resting state.
     */
    void Rest(CCI_FootBotState& c_robot_state);

    /*
     * Executes the exploring state.
     */
    void Explore(CCI_FootBotState& c_robot_state);

    /*
     * Executes the return to nest state.
     */
    void ReturnToNest(CCI_FootBotState& c_robot_state);

    void GoToFood(CCI_FootBotState& c_robot_state);


private:

    /*
        * Updates the state information.
        * In pratice, it sets the SStateData::InNest flag.
        * Future, more complex implementations should add their
        * state update code here.
        */
       void UpdateInNestState();

       /* The food data */
           SFoodData m_sFoodData;

           /* The controller state information */
           SStateData m_sStateData;


};

#endif
