#include "connection_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <controllers/footbot_connectedmotion/footbot_connectedmotion.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <sstream>


/****************************************/
/****************************************/

CConnectionLoopFunctions::CConnectionLoopFunctions()
{
}

/****************************************/
/****************************************/

void CConnectionLoopFunctions::Init(TConfigurationNode& t_node) {
    try {
        TConfigurationNode& tConnection = GetNode(t_node, "connection");
        GetNodeAttribute(tConnection, "initial_cntr_id", m_nNodeCounter);
        GetNodeAttribute(tConnection, "number_spares", m_nNumberSpares); //add limit to new entities added
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
    }
}

/****************************************/
/****************************************/

void CConnectionLoopFunctions::Reset() {
   //RemoveEntity(*m_pcNode);
}

/****************************************/
/****************************************/

void CConnectionLoopFunctions::Destroy() {

}

/****************************************/
/****************************************/

void CConnectionLoopFunctions::PreStep() {
    
    CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
    
    for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
        it != m_cFootbots.end();
        ++it) {
        CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
        CFootBotConnectedMotion& cController = dynamic_cast<CFootBotConnectedMotion&>(cFootBot.GetControllableEntity().GetController());
        for (TVecPair::iterator itt=m_tNewLevels.begin();itt!=m_tNewLevels.end();++itt) {
            if (cFootBot.GetId()==itt->first) {
                cController.GetTreeData().LevelCntr=itt->second;
                LOG<<itt->first<<std::endl;
                LOG<<itt->second<<std::endl;
            }
        }
    }
    m_tNewLevels.clear();
}

/****************************************/
/****************************************/

void CConnectionLoopFunctions::PostStep() {
    
    CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
    
    for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
        it != m_cFootbots.end();
        ++it) {
        
        /* Get handle to foot-bot entity and controller */
        CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
        CFootBotConnectedMotion& cController = dynamic_cast<CFootBotConnectedMotion&>(cFootBot.GetControllableEntity().GetController());
        CFootBotConnectedMotion::TPairVector& vecTreeInfo = cController.GetParentSonId();
        
        /* Display tree nodes (Parent id and son id)*/
        for (CFootBotConnectedMotion::TPairVector::iterator itt=vecTreeInfo.begin(); itt!= vecTreeInfo.end(); ++itt) {
            LOG<<"Parent:"<<itt->first<<std::endl;
            LOG<<"Son:"<<itt->second<<std::endl;
        }
        vecTreeInfo.clear();

        if (cController.GetTreeData().InfoNewNode!=CVector2(0,0)) {
            
            std::ostringstream oss; // replace by better name
            oss << "cntr" << m_nNodeCounter;
            ++m_nNodeCounter;
            
            Real old_X=cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX();
            Real old_Y=cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY();
            CRadians cZ, cY, cX;
            CQuaternion cTheta=cFootBot.GetEmbodiedEntity().GetOriginAnchor().Orientation;
            cTheta.ToEulerAngles(cZ,cY,cX);
            
            Real fNewX=old_X + cController.GetTreeData().InfoNewNode.Length()*Cos(cZ+cController.GetTreeData().InfoNewNode.Angle())/200;
            Real fNewY=old_Y + cController.GetTreeData().InfoNewNode.Length()*Sin(cZ+cController.GetTreeData().InfoNewNode.Angle())/200;
            CVector3 cPosition=CVector3(fNewX,fNewY,0);

            
            m_pcNode = new CFootBotEntity(oss.str(),              // the id
                                          "fdc",                //controller id
                                          cPosition,       // the position in m
                                          CQuaternion(0,0,0,0),         // the orientation
                                          Real(1),                  // rab range
                                          10,                       //
                                          ToRadians(CDegrees(80.0f)));   // omnicam aperture
            /* Add the footbot to the space */
            AddEntity(*m_pcNode);
            cController.GetTreeData().InfoNewNode=CVector2(0,0);
            cController.GetTreeData().bAlreadyIdle=false;

            
            if (cFootBot.GetId().find("wkr") != std::string::npos) {
                m_tNewLevels.push_back(std::make_pair(oss.str(),(cController.GetTreeData().LevelWkr-1)));
            }
            else {
                m_tNewLevels.push_back(std::make_pair(oss.str(),(cController.GetTreeData().LevelCntr-1)));
            }
            
        }
    }
}

//CQuaternion cOrientation=CRotationMatrix3(cFootBot.STreeData.InfoNewNode.angle()+cZ,0,0);

//        if (cFootBot.GetId()=="cntr2") {
//            if (GetSpace().GetSimulationClock()==2) {
//                m_pcNode = new CFootBotEntity("cntr3",              // the id
//                                                      "fdc",                //controller id
//                                                      CVector3(-1.2,0.4,0),       // the position in m
//                                                      CQuaternion(0,0,0,0),         // the orientation
//                                                      Real(1),                  // rab range
//                                                      10,
//                                                      ToRadians(CDegrees(80.0f)));   // omnicam aperture
//                /* Add the footbot to the space */
//                AddEntity(*m_pcNode);
//            }
//        }
//    }
//
//}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CConnectionLoopFunctions, "connection_loop_functions")
