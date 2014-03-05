#ifndef RECRUITER_LOOP_FUNCTIONS_H
#define RECRUIT_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>

using namespace argos;

class CRecruiterLoopFunctions : public CLoopFunctions {

public:

   CRecruiterLoopFunctions();
   virtual ~CRecruiterLoopFunctions() {}

   virtual void Init(TConfigurationNode& t_tree);
   virtual void Reset();
   virtual void Destroy();
   virtual CColor GetFloorColor(const CVector2& c_position_on_plane);
   virtual void PreStep();
   virtual bool InNest(const CVector2& cPos);
   virtual void FillFood();
   virtual void AddOneFood();
   virtual CVector2 GenerateFoodPatchPosition();
   virtual bool CloseToNest(const CVector2& cPos);
   virtual void generateUniformFoodPatch();
   virtual void generateFoodPatches();

private:

   CVector2 arenaSize;
   float nestSize; // length of a square representing the nest
   UInt32 nbFoodPatches;
   Real m_fFoodSquareRadius;
   CRange<Real> m_cForagingArenaSideX, m_cForagingArenaSideY;
   CRange<Real> m_cForagingArena2SideX, m_cForagingArena2SideY;
   CFloorEntity* m_pcFloor;
   CRandom::CRNG* m_pcRNG;
   std::vector<CVector2> m_cFoodPos;
   std::vector<std::vector<CVector2> > foodPatches;
   UInt32 m_nbCollectedFood;
   UInt32 renewalRate;
   float foodPatchSize;
   std::vector<CVector2> foodPatchCenters;
   std::vector<CVector2> foodPatchSizes;
   UInt32 NbFoodItems;
   UInt32 foodClock;
   std::string type;
   unsigned int randomSeed;

   std::string m_strOutput;
   std::ofstream m_cOutput;

};

#endif
