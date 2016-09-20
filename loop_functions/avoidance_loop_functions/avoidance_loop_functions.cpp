#include "avoidance_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <controllers/footbot_avoidance/footbot_avoidance.h>
#include <argos3/core/utility/logging/argos_log.h>

/****************************************/
/****************************************/

CAvoidanceLoopFunctions::CAvoidanceLoopFunctions()
{
    ;//m_pcValue1=m_pcValue;
}

/****************************************/
/****************************************/

void CAvoidanceLoopFunctions::Init(TConfigurationNode& t_node) {
   try {
      TConfigurationNode& tAvoidance = GetNode(t_node, "avoidance");
      /* Get the output file name from XML */
      GetNodeAttribute(tAvoidance, "output", m_strOutput);
      m_Pos.Set(2,3);
      /* Open the file, erasing its contents */
      m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
      m_cOutput << "step\tX\tY" << std::endl;
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
   }
}

/****************************************/
/****************************************/

void CAvoidanceLoopFunctions::Reset() {
   /* Close the file */
   m_cOutput.close();
   /* Open the file, erasing its contents */
   m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
   m_cOutput << "step\tposition\tangle" << std::endl;
    
}

/****************************************/
/****************************************/

void CAvoidanceLoopFunctions::Destroy() {
   /* Close the file */
   m_cOutput.close();
}

/****************************************/
/****************************************/

void CAvoidanceLoopFunctions::PreStep() {
    
    CVector2 cPos;
    CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
    
    for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
        it != m_cFootbots.end();
        ++it) {
        CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
        CFootBotAvoidance& cController = dynamic_cast<CFootBotAvoidance&>(cFootBot.GetControllableEntity().GetController());
        cPos.Set(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                 cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
        m_cOutput<<GetSpace().GetSimulationClock()<<"\t"<<cPos.GetX()<<"\t"<<cPos.GetY()<<std::endl;
        //LOG<<cPos.GetX()<<std::endl;
    }
    m_cOutput<<"test"<<std::endl;
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CAvoidanceLoopFunctions, "avoidance_loop_functions")
