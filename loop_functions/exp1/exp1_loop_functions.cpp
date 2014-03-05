#include "exp1_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
//#include <argos3/core/utility/datatypes/any.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include "../../controllers/exp1/bt_footbot_exp1_controller.h"

#define CLOSE_TO_NEST_PARAMETER 1
#define CLOSE_TO_ARENA_END_PARAMETER 0.5f

/****************************************/
/****************************************/

CExp1LoopFunctions::CExp1LoopFunctions() :
m_cForagingArenaSideX(2.0f, 3.0f),
m_cForagingArenaSideY(1.5f, 2.5f),
m_cForagingArena2SideX(-3.0f, -2.0f),
m_cForagingArena2SideY(1.5f, 2.5f),
m_pcFloor(NULL),
m_pcRNG(NULL),
m_nbCollectedFood(0),
m_fFoodSquareRadius(0.0f),
arenaSize(0,0),
nestSize(0),
nbFoodPatches(0),
renewalRate(0),
foodClock(0){
}

/****************************************/
/****************************************/

void CExp1LoopFunctions::Init(TConfigurationNode& t_node) {
	try {
		arenaSize = CVector2(GetSpace().GetArenaSize().GetX()-1, GetSpace().GetArenaSize().GetY()-1);

		/* Get a pointer to the floor entity */
		m_pcFloor =  &(GetSpace().GetFloorEntity());
		//m_pcFloor = &m_cSpace.GetFloorEntity();
		/* Create a new RNG */
		m_pcRNG = CRandom::CreateRNG("argos");

		/* Get the number of food items we want to be scattered from XML */
		TConfigurationNode& tForaging = GetNode(t_node, "foraging");
		GetNodeAttribute(tForaging, "nestSize", nestSize);
		GetNodeAttribute(tForaging, "nbFoodPatches", nbFoodPatches);
		GetNodeAttribute(tForaging, "renewalRate", renewalRate);
		foodClock = renewalRate;
		GetNodeAttribute(tForaging, "output", m_strOutput);
		GetNodeAttribute(tForaging, "type", type);
		GetNodeAttribute(tForaging, "foodPatchSize", foodPatchSize);
		GetNodeAttribute(tForaging, "nbFoodItems", NbFoodItems);
		GetNodeAttribute(tForaging, "radius", m_fFoodSquareRadius);
		m_fFoodSquareRadius *= m_fFoodSquareRadius;

		if(type == "uniform")
			generateUniformFoodPatch();
		else if (type == "patched")
			generateFoodPatches();
		else if(type == "mixed"){
			generateUniformFoodPatch();
			generateFoodPatches();
		}
		FillFood();


	}
	catch(CARGoSException& ex) {
		THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
	}

	m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
	m_cOutput << "# clock\tcollected_food\tavarage per 100 steps" << std::endl;
}

void CExp1LoopFunctions::generateFoodPatches(){
	for(UInt32 i = 0; i < nbFoodPatches ; i++){
		foodPatches.push_back(std::vector<CVector2>());
		foodPatchCenters.push_back(GenerateFoodPatchPosition());
		foodPatchSizes.push_back(CVector2(foodPatchSize, foodPatchSize));
	}
}

void CExp1LoopFunctions::generateUniformFoodPatch(){
	foodPatchCenters.push_back(CVector2(0,0));
	foodPatches.push_back(std::vector<CVector2>());
	foodPatchSizes.push_back(arenaSize);
}

/****************************************/
/****************************************/

void CExp1LoopFunctions::Reset() {
	//TODO fix
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
	m_cOutput << "# clock\tcollected_food\tavarage per 100 steps" << std::endl;
}

/****************************************/
/****************************************/

void CExp1LoopFunctions::Destroy() {
	/* Close the file */
	m_cOutput.close();
}

/****************************************/
/****************************************/

void CExp1LoopFunctions::PreStep() {
	if(--foodClock == 0){
		AddOneFood();
		foodClock = renewalRate;
	}

	/* Logic to pick and drop food items */
	/*
	 * If a robot is in the nest, drop the food item
	 * If a robot is on a food item, pick it
	 * Each robot can carry only one food item per time
	 */

	/* Check whether a robot is on a food item */
	CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");

	for(CSpace::TMapPerType::iterator it = m_cFootbots.begin(); it != m_cFootbots.end(); ++it) {
		/* Get handle to foot-bot entity and controller */
		CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
		CBTFootbotExp1Controller& cController = dynamic_cast<CBTFootbotExp1Controller&>(cFootBot.GetControllableEntity().GetController());
		CBTFootbotExp1RootBehavior* pcRootBehavior = cController.GetRootBehavior();

		/* Get the position of the foot-bot on the ground as a CVector2 */
		CVector2 cPos;
		cPos.Set(cFootBot.GetEmbodiedEntity().GetPosition().GetX(),
				cFootBot.GetEmbodiedEntity().GetPosition().GetY());

		/* Get state data */
		CBTFootbotExp1RootBehavior::SStateData* sStateData = &(*pcRootBehavior).GetStateData();
		sStateData->CurrentPosition = cPos;

		/* Get food data */
		CBTFootbotExp1RootBehavior::SFoodData* sFoodData = &(*pcRootBehavior).GetFoodData();
		/* The foot-bot has a food item */
		if(sFoodData->HasFoodItem) {
			/* Check whether the foot-bot is in the nest */
			if(InNest(cPos)) {
				/* Drop the food item */
				//foodPatches[sFoodData->FoodPatchIdx].erase(foodPatches[sFoodData->FoodPatchIdx].begin() + sFoodData->FoodItemIdx);
				sFoodData->HasFoodItem = false;
				sFoodData->FoodItemIdx = 0;
				sFoodData->FoodPatchIdx = 0;
				++sFoodData->TotalFoodItems;
				m_nbCollectedFood++;
			}
		}
		else {
			/* The foot-bot has no food item */
			/* Check whether the foot-bot is out of the nest */
			if(!InNest(cPos)) {
				/* Check whether the foot-bot is on a food item */
				bool bDone = false;
				for(UInt32 j = 0; j < foodPatches.size(); j++)
					for(size_t i = 0; i < foodPatches[j].size() && !bDone; ++i) {
						if((cPos - foodPatches[j][i]).SquareLength() < m_fFoodSquareRadius) {
							/* If so, we move that item out of sight */
							foodPatches[j][i].Set(1000.0f, 1000.f);
							/* The foot-bot is now carrying an item */
							sFoodData->HasFoodItem = true;
							sFoodData->FoodItemIdx = i;
							sFoodData->FoodPatchIdx = j;
							sFoodData->LastFoodPosition = cPos;
							/* We are done */
							bDone = true;
							foodPatches[sFoodData->FoodPatchIdx].erase(foodPatches[j].begin() + i);
							/* The floor texture must be updated */
							m_pcFloor->SetChanged();
						}
					}
			}
		}
	}
	if(GetSpace().GetSimulationClock() % 1000 == 0){
		m_cOutput << GetSpace().GetSimulationClock() << "\t"
				<< m_nbCollectedFood << "\t" << m_nbCollectedFood / (GetSpace().GetSimulationClock()/1000)<< "\n";
	}
}

void CExp1LoopFunctions::FillFood(){
	/*for(UInt32 j = 0; j < foodPatches.size(); j++)
		while(foodPatches[j].size() < NbFoodItems){
			CRange<Real> xRange((foodPatchCenters[j].GetX() - (foodPatchSizes[j].GetX()/2)), (foodPatchCenters[j].GetX() + (foodPatchSizes[j].GetX()/2)));
			CRange<Real> yRange((foodPatchCenters[j].GetY() - (foodPatchSizes[j].GetY()/2)), (foodPatchCenters[j].GetY() + (foodPatchSizes[j].GetY()/2)));
			CVector2 pos(m_pcRNG->Uniform(xRange), m_pcRNG->Uniform(yRange));
			//if(!InNest(pos))
			foodPatches[j].push_back(pos);
		}*/
	for(UInt32 j = 0; j < foodPatches.size(); j++)
		while(foodPatches[j].size() < NbFoodItems){
			CRange<Real> xRange((foodPatchCenters[j].GetX() - (foodPatchSizes[j].GetX()/2)), (foodPatchCenters[j].GetX() + (foodPatchSizes[j].GetX()/2)));
			CRange<Real> yRange((foodPatchCenters[j].GetY() - (foodPatchSizes[j].GetY()/2)), (foodPatchCenters[j].GetY() + (foodPatchSizes[j].GetY()/2)));
			CVector2 pos(m_pcRNG->Uniform(xRange), m_pcRNG->Uniform(yRange));
			if(!CloseToNest(pos) && !CloseToArenaEnd(pos))
				foodPatches[j].push_back(pos);
		}
}

void CExp1LoopFunctions::AddOneFood(){
	/*bool added = false;
	for(UInt32 j = 0; j < foodPatches.size(); j++){
		while(foodPatches[j].size() < NbFoodItems && !added){
			CRange<Real> xRange((foodPatchCenters[j].GetX() - (foodPatchSizes[j].GetX()/2)), (foodPatchCenters[j].GetX() + (foodPatchSizes[j].GetX()/2)));
			CRange<Real> yRange((foodPatchCenters[j].GetY() - (foodPatchSizes[j].GetY()/2)), (foodPatchCenters[j].GetY() + (foodPatchSizes[j].GetY()/2)));CVector2 pos(m_pcRNG->Uniform(xRange), m_pcRNG->Uniform(yRange));
			if(!InNest(pos)){
				foodPatches[j].push_back(pos);
				added = true;
			}
		}
		added = false;
	}*/

	bool added = false;
		for(UInt32 j = 0; j < foodPatches.size(); j++){
			while(foodPatches[j].size() < NbFoodItems && !added){
				CRange<Real> xRange((foodPatchCenters[j].GetX() - (foodPatchSizes[j].GetX()/2)), (foodPatchCenters[j].GetX() + (foodPatchSizes[j].GetX()/2)));
				CRange<Real> yRange((foodPatchCenters[j].GetY() - (foodPatchSizes[j].GetY()/2)), (foodPatchCenters[j].GetY() + (foodPatchSizes[j].GetY()/2)));CVector2 pos(m_pcRNG->Uniform(xRange), m_pcRNG->Uniform(yRange));
				if(!CloseToNest(pos) && !CloseToArenaEnd(pos)){
					foodPatches[j].push_back(pos);
					added = true;
				}
			}
			added = false;
		}
}

CVector2 CExp1LoopFunctions::GenerateFoodPatchPosition(){

	/*CRange<Real> xRange(-(arenaSize.GetX()/2)+foodPatchSize/2, arenaSize.GetX()/2-foodPatchSize/2);
	CRange<Real> yRange(-(arenaSize.GetY()/2)+foodPatchSize/2, arenaSize.GetY()/2-foodPatchSize/2);
	m_pcRNG->Uniform(xRange);m_pcRNG->Uniform(xRange); m_pcRNG->Uniform(xRange);
	CVector2 tmp(0,0);
	while(CloseToNest(tmp)){
		tmp.Set(m_pcRNG->Uniform(xRange), m_pcRNG->Uniform(yRange));
	}

	return tmp;*/

	CRange<Real> xRange(-(arenaSize.GetX()/2), arenaSize.GetX()/2);
	CRange<Real> yRange(-(arenaSize.GetY()/2), arenaSize.GetY()/2);
	m_pcRNG->Uniform(xRange);m_pcRNG->Uniform(xRange); m_pcRNG->Uniform(xRange);
	CVector2 tmp(0,0);
	while(CloseToNest(tmp) || CloseToArenaEnd(tmp)){
		tmp.Set(m_pcRNG->Uniform(xRange), m_pcRNG->Uniform(yRange));
	}

	return tmp;
}
/****************************************/
/****************************************/

CColor CExp1LoopFunctions::GetFloorColor(const CVector2& c_position_on_plane) {
	if(InNest(c_position_on_plane)) {
		return CColor::GRAY50;
	}
	for(UInt32 j = 0; j < foodPatches.size(); j++)
		for(UInt32 i = 0; i < foodPatches[j].size(); ++i) {
			if((c_position_on_plane - foodPatches[j][i]).SquareLength() < m_fFoodSquareRadius) {
				return CColor::BLACK;
			}
		}
	return CColor::WHITE;
}

/****************************************/
/****************************************/

bool CExp1LoopFunctions::InNest(const CVector2& pos){
	if(pos.GetX() > -(nestSize/2.0f) && pos.GetX() < (nestSize/2.0f) && pos.GetY() > -(nestSize/2.0f) && pos.GetY() < (nestSize/2.0f)) {
		return true;
	}
	return false;
}

/**
 * Returns true if the given vector is near the nest, relative to food patch size.
 */
bool CExp1LoopFunctions::CloseToNest(const CVector2& pos){
	/*if(pos.GetX() > -(nestSize/2.0f)-foodPatchSize/1 && pos.GetX() < (nestSize/2.0f) + foodPatchSize/1 && pos.GetY() > -(nestSize/2.0f) - foodPatchSize/1 && pos.GetY() < (nestSize/2.0f) + foodPatchSize/1) {
		return true;
	}
	return false;*/

	if(pos.GetX() > -(nestSize/2.0f) - CLOSE_TO_NEST_PARAMETER && pos.GetX() < (nestSize/2.0f) + CLOSE_TO_NEST_PARAMETER && pos.GetY() > -(nestSize/2.0f) - CLOSE_TO_NEST_PARAMETER && pos.GetY() < (nestSize/2.0f) + CLOSE_TO_NEST_PARAMETER) {
		return true;
	}
	return false;
}


/**
 * Returns true if the given vector is near the arena end.
 */
bool CExp1LoopFunctions::CloseToArenaEnd(const CVector2& pos){

	if(pos.GetX() > arenaSize.GetX()/2.0f - CLOSE_TO_ARENA_END_PARAMETER ||
			pos.GetX() < -arenaSize.GetX()/2.0f + CLOSE_TO_ARENA_END_PARAMETER ||
			pos.GetY() > arenaSize.GetY()/2.0f - CLOSE_TO_ARENA_END_PARAMETER ||
			pos.GetY() < -arenaSize.GetY()/2.0f + CLOSE_TO_ARENA_END_PARAMETER) {
		return true;
	}
	return false;
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CExp1LoopFunctions, "exp1_loop_functions")
