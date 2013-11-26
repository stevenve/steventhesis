#include "simple_qt_user_functions.h"
#include "../../controllers/randomforaging/bt_footbot_randomforaging_controller.h"
#include <argos3/core/simulator/entity/controllable_entity.h>

/****************************************/
/****************************************/

void CSimpleQTUserFunctions::Draw(CFootBotEntity& c_entity) {
   CBTFootbotRandomForagingController& cController = dynamic_cast<CBTFootbotRandomForagingController&>(c_entity.GetControllableEntity().GetController());

}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CSimpleQTUserFunctions, "simple_qt_user_functions")
