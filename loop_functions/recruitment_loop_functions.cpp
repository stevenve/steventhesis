#include "recruitment_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
//#include <argos3/core/utility/datatypes/any.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include "../controllers/randomforaging/bt_footbot_randomforaging_controller.h"

/****************************************/
/****************************************/

CRecruitmentLoopFunctions::CRecruitmentLoopFunctions() :
   m_cForagingArenaSideX(2.0f, 3.0f),
   m_cForagingArenaSideY(1.5f, 2.5f),
   m_cForagingArena2SideX(-3.0f, -2.0f),
   m_cForagingArena2SideY(1.5f, 2.5f),
   m_pcFloor(NULL),
   m_pcRNG(NULL),
   m_nbCollectedFood(0),
   m_fFoodSquareRadius(0.0f){
}

/****************************************/
/****************************************/

void CRecruitmentLoopFunctions::Init(TConfigurationNode& t_node) {
   try {
      /* Get a pointer to the floor entity */
	  m_pcFloor =  &GetSpace().GetFloorEntity();
      //m_pcFloor = &m_cSpace.GetFloorEntity();
      /* Create a new RNG */
      m_pcRNG = CRandom::CreateRNG("argos");

      /* Get the number of food items we want to be scattered from XML */
      TConfigurationNode& tForaging = GetNode(t_node, "foraging");
      UInt32 NbFoodItems;
      GetNodeAttribute(tForaging, "items", NbFoodItems);
      GetNodeAttribute(tForaging, "radius", m_fFoodSquareRadius);
      m_fFoodSquareRadius *= m_fFoodSquareRadius;

      /* Distribute uniformly the items in the environment */
      for(UInt32 i = 0; i < NbFoodItems; ++i) {
         m_cFoodPos.push_back(
            CVector2(m_pcRNG->Uniform(m_cForagingArenaSideX),
                     m_pcRNG->Uniform(m_cForagingArenaSideY)));
      }
      for(UInt32 i = 0; i < NbFoodItems; ++i) {
         m_cFoodPos.push_back(
            CVector2(m_pcRNG->Uniform(m_cForagingArena2SideX),
                     m_pcRNG->Uniform(m_cForagingArena2SideY)));
      }
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
   }

   m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
   m_cOutput << "# clock\tcollected_food" << std::endl;
}

/****************************************/
/****************************************/

void CRecruitmentLoopFunctions::Reset() {
   /* Distribute uniformly the items in the environment */
   for(UInt32 i = 0; i < m_cFoodPos.size(); ++i) {
      m_cFoodPos[i].Set(m_pcRNG->Uniform(m_cForagingArenaSideX),
                        m_pcRNG->Uniform(m_cForagingArenaSideY));
   }

   m_nbCollectedFood = 0;
   /* Close the file */
      m_cOutput.close();
      /* Open the file, erasing its contents */
      m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
      m_cOutput << "# clock\twalking\tresting\tcollected_food\tenergy" << std::endl;
}

/****************************************/
/****************************************/

void CRecruitmentLoopFunctions::Destroy() {
	/* Close the file */
	   m_cOutput.close();
}

/****************************************/
/****************************************/

void CRecruitmentLoopFunctions::PreStep() {
   /* Logic to pick and drop food items */
   /*
    * If a robot is in the nest, drop the food item
    * If a robot is on a food item, pick it
    * Each robot can carry only one food item per time
    */

   /* Check whether a robot is on a food item */
   //CSpace::TMapPerType& m_cFootbots = m_cSpace.GetEntitiesByType("foot-bot");
	CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");

   for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
       it != m_cFootbots.end();
       ++it) {
      /* Get handle to foot-bot entity and controller */
      CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
      CBTFootbotRandomForagingController& cController = dynamic_cast<CBTFootbotRandomForagingController&>(cFootBot.GetControllableEntity().GetController());
      /* Get the position of the foot-bot on the ground as a CVector2 */
      CVector2 cPos;
      cPos.Set(cFootBot.GetEmbodiedEntity().GetPosition().GetX(),
               cFootBot.GetEmbodiedEntity().GetPosition().GetY());
      /* Get food data */
      CBTFootbotRandomForagingController::SFoodData& sFoodData = cController.GetFoodData();
      /* The foot-bot has a food item */
      if(sFoodData.HasFoodItem) {
         /* Check whether the foot-bot is in the nest */
         if(InNest(cPos)) {
            /* Place a new food item on the ground */
            m_cFoodPos[sFoodData.FoodItemIdx].Set(m_pcRNG->Uniform(m_cForagingArenaSideX),
                                                  m_pcRNG->Uniform(m_cForagingArenaSideY));
            /* Drop the food item */
            sFoodData.HasFoodItem = false;
            sFoodData.FoodItemIdx = 0;
            ++sFoodData.TotalFoodItems;
            m_nbCollectedFood++;
            /* The floor texture must be updated */
            m_pcFloor->SetChanged();
         }
      }
      else {
         /* The foot-bot has no food item */
         /* Check whether the foot-bot is out of the nest */
         if(!InNest(cPos)) {
            /* Check whether the foot-bot is on a food item */
            bool bDone = false;
            for(size_t i = 0; i < m_cFoodPos.size() && !bDone; ++i) {
               if((cPos - m_cFoodPos[i]).SquareLength() < m_fFoodSquareRadius) {
                  /* If so, we move that item out of sight */
                  m_cFoodPos[i].Set(100.0f, 100.f);
                  /* The foot-bot is now carrying an item */
                  sFoodData.HasFoodItem = true;
                  sFoodData.FoodItemIdx = i;
                  /* The floor texture must be updated */
                  m_pcFloor->SetChanged();
                  /* We are done */
                  bDone = true;
               }
            }
         }
      }
   }
   m_cOutput << GetSpace().GetSimulationClock() << "\t"
                << m_nbCollectedFood << "\n";
}

/****************************************/
/****************************************/

CColor CRecruitmentLoopFunctions::GetFloorColor(const CVector2& c_position_on_plane) {
   if(InNest(c_position_on_plane)) {
      return CColor::GRAY50;
   }
   for(UInt32 i = 0; i < m_cFoodPos.size(); ++i) {
      if((c_position_on_plane - m_cFoodPos[i]).SquareLength() < m_fFoodSquareRadius) {
         return CColor::BLACK;
      }
   }
   return CColor::WHITE;
}

/****************************************/
/****************************************/

bool CRecruitmentLoopFunctions::InNest(const CVector2& pos){
    if(pos.GetX() > -1.0f && pos.GetX() < 1.0f && pos.GetY() > -5.0f && pos.GetY() < -3.0f) {
      return true;
   }
   return false;
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CRecruitmentLoopFunctions, "recruitment_loop_functions")
