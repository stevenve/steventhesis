#ifndef RECRUITMENT_QT_USER_FUNCTIONS_H
#define RECRUITMENT_QT_USER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include "../../controllers/recruitment/bt_footbot_recruitment_controller.h"

using namespace argos;

class CRecruitmentQTUserFunctions : public CQTOpenGLUserFunctions {

public:
   
   virtual void Draw(CFootBotEntity& c_entity);

private:

};

#endif
