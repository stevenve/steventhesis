#include "combined_qt_user_functions.h"
#include "../../controllers/combined/bt_footbot_combined_controller.h"
#include <argos3/core/simulator/entity/controllable_entity.h>

/****************************************/
/****************************************/

void CCombinedQTUserFunctions::Draw(CFootBotEntity& c_entity) {
   CBTFootbotCombinedController& cController = dynamic_cast<CBTFootbotCombinedController&>(c_entity.GetControllableEntity().GetController());

}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CCombinedQTUserFunctions, "combined_qt_user_functions")
