#ifndef EXP1_QT_USER_FUNCTIONS_H
#define EXP1_QT_USER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

using namespace argos;

class CExp1QTUserFunctions : public CQTOpenGLUserFunctions {

public:
   
   virtual void Draw(CFootBotEntity& c_entity);

private:

};

#endif
