//#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/control_interface/ci_controller.h>

#include "bt_footbot_randomforaging_controller.h"


using namespace argos;


void CBTFootbotRandomForagingController::Init ( TConfigurationNode& t_tree)
{
    // Get the robot data, state and the root behavior
	m_pcRobotData = new CCI_RobotData<CCI_FootBotState>( (CCI_Controller*) this, (CCI_BehaviorController<CCI_FootBotState>*) this, t_tree );
	m_pcState = new CCI_FootBotState((CCI_Controller*) this);
	m_pcState->Init();
	m_pcRootBehavior = new CBTFootbotRandomForagingRootBehavior(m_pcRobotData);
	m_pcRobotData->SetRootBehavior( (CCI_Behavior<CCI_FootBotState>*) m_pcRootBehavior );

	m_pcRootBehavior->Init( *m_pcState );

	/* Controller state */
	     m_sStateData.Init(GetNode(t_tree, "state"));

	    /* reset all */
	    Reset();
}

void CBTFootbotRandomForagingController::ControlStep (  )
{
	switch(m_sStateData.State) {
	      case SStateData::STATE_EXPLORING: {
	         Explore((CCI_FootBotState&) (*m_pcState));
	         break;
	      }
	      case SStateData::STATE_RETURN_TO_NEST: {
	         ReturnToNest((CCI_FootBotState&) (*m_pcState));
	         break;
	      }
	      default: {
	         THROW_ARGOSEXCEPTION("Invalid State");
	      }
	   }

    // Call the control step in of the interface
    CCI_BehaviorController<CCI_FootBotState>::ControlStep();
}

UInt32 CBTFootbotRandomForagingController::GetRootBehaviorStateID()
{
	CBTFootbotRandomForagingRootBehavior* pcRootBehavior = (CBTFootbotRandomForagingRootBehavior*) m_pcRootBehavior;
    return pcRootBehavior->GetStateID();
}

std::string CBTFootbotRandomForagingController::GetRootBehaviorStateName()
{
	CBTFootbotRandomForagingRootBehavior* pcRootBehavior = (CBTFootbotRandomForagingRootBehavior*) m_pcRootBehavior;
    return pcRootBehavior->GetStateName();
}

void CBTFootbotRandomForagingController::Destroy (  )
{
    // Clean up in the root behavior
    m_pcRootBehavior->Destroy(*m_pcState);

    // Release memory
    delete m_pcRobotData;
    delete m_pcState;
    delete m_pcRootBehavior;
}

/****************************************/
/** state related methods
/****************************************/

CBTFootbotRandomForagingController::SFoodData::SFoodData() :
   HasFoodItem(false),
   FoodItemIdx(0),
   TotalFoodItems(0) {}

void CBTFootbotRandomForagingController::SFoodData::Reset() {
   HasFoodItem = false;
   FoodItemIdx = 0;
   TotalFoodItems = 0;
}

CBTFootbotRandomForagingController::SStateData::SStateData() {
	State = STATE_EXPLORING;
	InNest = true;
}

void CBTFootbotRandomForagingController::SStateData::Init(TConfigurationNode& t_node) {
	State = STATE_EXPLORING;
	InNest = false;
}

void CBTFootbotRandomForagingController::SStateData::Reset() {
   State = STATE_EXPLORING;
   InNest = true;
}

void CBTFootbotRandomForagingController::UpdateInNestState() {

	if(((CBTFootbotRandomForagingRootBehavior*) m_pcRootBehavior)->InNest())
		m_sStateData.InNest = true;
	else
		m_sStateData.InNest = false;
}

void CBTFootbotRandomForagingController::ReturnToNest(CCI_FootBotState& c_robot_state) {
   UpdateInNestState();
   if(m_sStateData.InNest)
	   m_sStateData.State = SStateData::STATE_EXPLORING;
   else
	   ((CBTFootbotRandomForagingRootBehavior*) m_pcRootBehavior)->ReturnToNest();
}

void CBTFootbotRandomForagingController::Explore(CCI_FootBotState& c_robot_state) {
   /* We switch to 'return to nest' if we have a food item
    */
   bool bReturnToNest(false);
   /*
    * Have we found a food item?
    * NOTE: the food data is updated by the loop functions, so
    * here we just need to read it
    */
   if(m_sFoodData.HasFoodItem) {
      /* Switch to 'return to nest' and make LEDs RED */
      bReturnToNest = true;
   }
   /* So, do we return to the nest now? */
   if(bReturnToNest) {
      //m_pcLEDs->SetAllColors(CColor::RED);
      m_sStateData.State = SStateData::STATE_RETURN_TO_NEST;
   }
   else {

      /* No, perform the actual exploration */
      UpdateInNestState();
      /* Get the diffusion vector to perform obstacle avoidance */
      //bool bCollision;
      //CVector2 cDiffusion = DiffusionVector(bCollision);
      /*
       * If we are in the nest, we combine antiphototaxis with obstacle avoidance
       * Outside the nest, we just use the diffusion vector
       */
      if(m_sStateData.InNest) {
         /*
          * The vector returned by CalculateVectorToLight() points to
          * the light. Thus, the minus sign is because we want to go away
          * from the light.
          */
         //SetWheelSpeedsFromVector(
           // m_sWheelTurningParams.MaxSpeed * cDiffusion -
            //m_sWheelTurningParams.MaxSpeed * 0.25f * CalculateVectorToLight());
          ((CBTFootbotRandomForagingRootBehavior*) m_pcRootBehavior)->ExitNest();
      }
      else {
    	((CBTFootbotRandomForagingRootBehavior*) m_pcRootBehavior)->Explore();
         /* Use the diffusion vector only */
        // SetWheelSpeedsFromVector(m_sWheelTurningParams.MaxSpeed * cDiffusion);
      }
   }
}

/*
 * To allow dynamic loading of this controller
 */
REGISTER_CONTROLLER( CBTFootbotRandomForagingController, "footbot_simple_controller" );

