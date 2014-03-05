#include "recruiter_qt_user_functions.h"

#include <argos3/core/simulator/entity/controllable_entity.h>

/****************************************/
/****************************************/

void CRecruitmentQTUserFunctions::Draw(CFootBotEntity& c_entity) {
	CBTFootbotRecruitmentController& cController = dynamic_cast<CBTFootbotRecruitmentController&>(c_entity.GetControllableEntity().GetController());

}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CRecruitmentQTUserFunctions, "recruitment_qt_user_functions")
