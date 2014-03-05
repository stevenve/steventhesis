#include "recruiter_qt_user_functions.h"
#include "../../controllers/recruiter/bt_footbot_recruiter_controller.h"
#include <argos3/core/simulator/entity/controllable_entity.h>

/****************************************/
/****************************************/

void CRecruiterQTUserFunctions::Draw(CFootBotEntity& c_entity) {
   CBTFootbotRecruiterController& cController = dynamic_cast<CBTFootbotRecruiterController&>(c_entity.GetControllableEntity().GetController());

}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CRecruiterQTUserFunctions, "recruiter_qt_user_functions")
