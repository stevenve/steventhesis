#ifndef EXP1_LOOP_FUNCTIONS_H
#define EXP1_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>

using namespace argos;

class CExp1LoopFunctions : public CLoopFunctions {

public:

   CExp1LoopFunctions();
   virtual ~CExp1LoopFunctions() {}

   virtual void Init(TConfigurationNode& t_tree);
   virtual void Reset();
   virtual void Destroy();
   virtual CColor GetFloorColor(const CVector2& c_position_on_plane);
   virtual void PreStep();
   virtual bool InNest(const CVector2& cPos);

private:

   CVector2 arenaSize;
   UInt32 nestSize; // length of a square representing the nest
   Real m_fFoodSquareRadius;
   CRange<Real> m_cForagingArenaSideX, m_cForagingArenaSideY;
   CRange<Real> m_cForagingArena2SideX, m_cForagingArena2SideY;
   CFloorEntity* m_pcFloor;
   CRandom::CRNG* m_pcRNG;
   std::vector<CVector2> m_cFoodPos;
   UInt32 m_nbCollectedFood;

   std::string m_strOutput;
   std::ofstream m_cOutput;

};

#endif
