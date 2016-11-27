#include "connection_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <controllers/footbot_connectedmotion/footbot_connectedmotion.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <sstream>
//#include <argos3/src/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.cpp>


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
        GetNodeAttribute(tConnection, "number_spares", m_nNumberSpares); //add limit to new entities added
        m_nNodeCounter=2;
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
        /* Get handle to foot-bot entity and controller */
        CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
        CFootBotConnectedMotion& cController = dynamic_cast<CFootBotConnectedMotion&>(cFootBot.GetControllableEntity().GetController());
        
        for (TVecNode::iterator it1=m_tLevels.begin();it1!=m_tLevels.end();++it1) {
            if (cFootBot.GetId()==it1->id) {
                /* Set tree data for nodes added in previous PostStep */
                cController.GetTreeData().Level=it1->level;
                cController.GetTreeData().IsLeaf=it1->IsLeaf;
                cController.GetTreeData().IsRoot=it1->IsRoot;
                
                LOG<<it1->id<<std::endl;
                LOG<<it1->level<<std::endl;
            }
        }
    }
    m_tLevels.clear();
}

/****************************************/
/****************************************/

void CConnectionLoopFunctions::PostStep() {
    
    CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
    
    for(CSpace::TMapPerType::iterator it = m_cFootbots.begin(); it != m_cFootbots.end(); ++it) {
        
        /* Get handle to foot-bot entity and controller */
        CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
        CFootBotConnectedMotion& cController = dynamic_cast<CFootBotConnectedMotion&>(cFootBot.GetControllableEntity().GetController());
        
        CFootBotConnectedMotion::TPairVector& vecTree = cController.GetParentSonId();
        
        /* Display tree nodes (Parent id and son id)*/
        for (CFootBotConnectedMotion::TPairVector::iterator itt=vecTree.begin(); itt!= vecTree.end(); ++itt) {
            LOG<<"Parent:"<<itt->first<<std::endl;
            LOG<<"Son:"<<itt->second<<std::endl;
        }
        vecTree.clear();

        if (cController.GetTreeData().StretchedRangeAngle!=CVector2(0,0) && m_nNodeCounter<m_nNumberSpares) {
            
            /* Create robot ID */
            std::ostringstream strId;
            strId << "fb" << m_nNodeCounter;
            ++m_nNodeCounter;
            
            /**/
            Real fOldX=cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX();
            Real fOldY=cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY();
            /**/
            CRadians cTheta, cY, cX;
            CQuaternion cTemp=cFootBot.GetEmbodiedEntity().GetOriginAnchor().Orientation;
            cTemp.ToEulerAngles(cTheta,cY,cX);
            
            /**/
            Real fNewX=fOldX + cController.GetTreeData().StretchedRangeAngle.Length()*Cos(cTheta+cController.GetTreeData().StretchedRangeAngle.Angle())/200;
            Real fNewY=fOldY + cController.GetTreeData().StretchedRangeAngle.Length()*Sin(cTheta+cController.GetTreeData().StretchedRangeAngle.Angle())/200;
            CVector3 cPosition=CVector3(fNewX,fNewY,0);

            m_pcNode = new CFootBotEntity(strId.str(),              // the id
                                          "fdc",                //controller id
                                          cPosition,       // the position in m
                                          CQuaternion(0,0,0,0),         // the orientation
                                          Real(1),                  // rab range
                                          10,                       //
                                          ToRadians(CDegrees(80.0f)));   // omnicam aperture
            /* Add the footbot to the space */
            AddEntity(*m_pcNode);
            
            /* Update tree data for parent footbot*/
            cController.GetTreeData().StretchedRangeAngle=CVector2(0,0);
            cController.GetTreeData().AlreadyIdle=false; // needed ? cfr constructor
            
            m_tLevels.push_back(SNode(strId.str(),(cController.GetTreeData().Level-1),false,false));
        }
        else if (m_nNodeCounter>=m_nNumberSpares) {
            LOG<<"Out of spare robots"<<std::endl;
        }
        else if (cController.GetStartTree()) {
            for (int i=0; i<NUMBER_OF_TASKS; ++i) {
                /* Create robot ID */
                std::ostringstream strId;
                strId << "fb" << m_nNodeCounter;
                ++m_nNodeCounter;
                
                /* */
                Real fOldX=cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX();
                Real fOldY=cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY();
                //rotate CVector2
                
                /* */
                Real fNewX= fOldX + (STRETCH_THRESHOLD/200)*cos(2*PI/NUMBER_OF_TASKS*(i+1));
                Real fNewY= fOldY + (STRETCH_THRESHOLD/200)*sin(2*PI/NUMBER_OF_TASKS*(i+1));
                CVector3 cPosition=CVector3(fNewX,fNewY,0);
                
                m_pcNode = new CFootBotEntity(strId.str(),              // the id
                                              "fdc",                //controller id
                                              cPosition,       // the position in m
                                              CQuaternion(0,0,0,0),     // the orientation
                                              Real(1),                  // rab range
                                              10,                       //
                                              ToRadians(CDegrees(80.0f)));   // omnicam aperture
                /* Add the footbot to the space */
                AddEntity(*m_pcNode);
                m_tLevels.push_back(SNode(strId.str(),(cController.GetTreeData().Level+1),true,false));
            }
            cController.GetStartTree()=false;
        }
    }
}

//            if (GetSpace().GetSimulationClock()==2) {


REGISTER_LOOP_FUNCTIONS(CConnectionLoopFunctions, "connection_loop_functions")
