#include "exp1_qt_user_functions.h"
#include "../../controllers/exp1/bt_footbot_exp1_controller.h"
#include <argos3/core/simulator/entity/controllable_entity.h>

/****************************************/
/****************************************/

void CExp1QTUserFunctions::Draw(CFootBotEntity& c_entity) {
   CBTFootbotExp1Controller& cController = dynamic_cast<CBTFootbotExp1Controller&>(c_entity.GetControllableEntity().GetController());

}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CExp1QTUserFunctions, "exp1_qt_user_functions")
