#include "combined_novis_qt_user_functions.h"
#include "../../controllers/combined_novis/bt_footbot_combined_novis_controller.h"
#include <argos3/core/simulator/entity/controllable_entity.h>

/****************************************/
/****************************************/

void CCombinedNoVisQTUserFunctions::Draw(CFootBotEntity& c_entity) {
   CBTFootbotCombinedNoVisController& cController = dynamic_cast<CBTFootbotCombinedNoVisController&>(c_entity.GetControllableEntity().GetController());

}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CCombinedNoVisQTUserFunctions, "combined_novis_qt_user_functions")
