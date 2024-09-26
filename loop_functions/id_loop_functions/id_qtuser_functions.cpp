#include "id_qtuser_functions.h"

/****************************************/
/****************************************/

CIDQTUserFunctions::CIDQTUserFunctions() {
   RegisterUserFunction<CIDQTUserFunctions,CFootBotEntity>(&CIDQTUserFunctions::Draw);
}

/****************************************/
/****************************************/

void CIDQTUserFunctions::Draw(CFootBotEntity& c_entity) {
   /* The position of the text is expressed wrt the reference point of the footbot
    * For a foot-bot, the reference point is the center of its base.
    * See also the description in
    * $ argos3 -q foot-bot
    */

   std::string id = c_entity.GetId();
   if (id == "fb_nav") {
      DrawText(CVector3(0.15, 0.0, 0.3),   // position
            "Nav"); // text
   } else if (id == "fb_target") {
      DrawText(CVector3(0.0, 0.0, 0.3),   // position
            "Target"); // text
   }
   
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CIDQTUserFunctions, "id_qtuser_functions")
