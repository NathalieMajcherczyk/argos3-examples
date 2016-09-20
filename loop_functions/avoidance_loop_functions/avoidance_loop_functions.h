#ifndef AVOIDANCE_LOOP_FUNCTIONS_H
#define AVOIDANCE_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <controllers/footbot_avoidance/footbot_avoidance.h>
//#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
//#include <argos3/plugins/robots/generic/control_interface/ci_proximity_sensor.h>

using namespace argos;

class CAvoidanceLoopFunctions : public CLoopFunctions {

public:

   CAvoidanceLoopFunctions();
   virtual ~CAvoidanceLoopFunctions() {}

   //CFootBotAvoidance c;
   //CCI_FootBotProximitySensor* m_pcProximity;
    CVector2 m_Pos;
    
   virtual void Init(TConfigurationNode& t_tree);
   virtual void Reset();
   virtual void Destroy();
   virtual void PreStep();
   //virtual void PostStep();
   
    
private:
   std::string m_strOutput;
   std::ofstream m_cOutput;    
};

#endif
