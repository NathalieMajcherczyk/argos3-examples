#include "connection_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <controllers/footbot_connectedmotion/footbot_connectedmotion.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <sstream>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/angles.h>
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
                
                //LOG<<it1->id<<std::endl;
                //LOG<<it1->level<<std::endl;
            }
        }
    }
    m_tLevels.clear();
}

/****************************************/
/****************************************/

void CConnectionLoopFunctions::PostStep() {
    
    CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
    bool bAddedRobots=false;
    
    for(CSpace::TMapPerType::iterator it = m_cFootbots.begin(); it != m_cFootbots.end(); ++it) {
        
        /* Get handle to foot-bot entity and controller */
        CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
        CFootBotConnectedMotion& cController = dynamic_cast<CFootBotConnectedMotion&>(cFootBot.GetControllableEntity().GetController());
        
        CFootBotConnectedMotion::TPairVector& vecTree = cController.GetParentSonId();
        int tempId;
        
        /* Display tree nodes (Parent id and son id)*/
        for (CFootBotConnectedMotion::TPairVector::iterator itt=vecTree.begin(); itt!= vecTree.end(); ++itt) {
            //LOG<<"Parent:"<<itt->first<<std::endl;
            //LOG<<"Son:"<<itt->second<<std::endl;
            tempId=itt->second;
        }
        vecTree.clear();

        if (cController.GetTreeData().StretchedRangeAngle!=CVector2() && m_nNodeCounter<m_nNumberSpares) {
            
            /* Create robot ID */
            std::ostringstream strId;
            strId << "fb" << m_nNodeCounter;
            
            CRadians cTheta, cY, cX;
            CQuaternion cTemp=cFootBot.GetEmbodiedEntity().GetOriginAnchor().Orientation;
            cTemp.ToEulerAngles(cTheta,cY,cX);
            
            /* Get absolute position of parent */
            CVector3 cOldPosition=CVector3(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY(),cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetZ());
            
            /* Compute absolute position of new robot */
            CVector3 cNewPosition=cOldPosition+(CVector3(cController.GetTreeData().StretchedRangeAngle.Length()/200,0,0).RotateZ(cTheta+cController.GetTreeData().StretchedRangeAngle.Angle()));
            
            DEBUG(",X: %f , Y: %f , Old: %i , New: %i , Level : %i \n",cNewPosition.GetX(),cNewPosition.GetY(),tempId,m_nNodeCounter,cController.GetTreeData().Level-1);

            ++m_nNodeCounter;
            
            m_pcNode = new CFootBotEntity(strId.str(),              // the id
                                          "fdc",                //controller id
                                          cNewPosition,       // the position in m
                                          cTemp,         // the orientation  // or find a way to add Angle
                                          1.0,                  // rab range
                                          10,                       //
                                          ToRadians(CDegrees(80.0f)));   // omnicam aperture
            /* Add the footbot to the space */
            AddEntity(*m_pcNode);
            
            /* Update tree data for parent footbot*/
            cController.GetTreeData().StretchedRangeAngle=CVector2(); // use setter function
            cController.GetTreeData().AlreadyIdle=false; // use setter function
            m_tLevels.push_back(SNode(strId.str(),(cController.GetTreeData().Level-1),false,false));
            bAddedRobots=true;
        }
        else if (m_nNodeCounter>=m_nNumberSpares) {
            LOG<<"Out of spare robots"<<std::endl;
        }
        else if (cController.GetStartTree()) {
            
            /* Get absolute position of root */
            CVector3 cRoot(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY(),cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetZ());
            /* Choose deployment vector from root*/
            CVector3 cDeployment(STRETCH_THRESHOLD/200,0,0);
            CRadians cAngle(2*ARGOS_PI/NUMBER_OF_TASKS);
            CQuaternion cTheta=cFootBot.GetEmbodiedEntity().GetOriginAnchor().Orientation;
            CRadians cY(0);
            CRadians cX(0);
            
            for (int i=0; i<NUMBER_OF_TASKS; ++i) {
                
                /* Create robot ID */
                std::ostringstream strId;
                strId << "fb" << m_nNodeCounter;
                ++m_nNodeCounter;
                
                /* Compute absolute position of new robot */
                CVector3 cPosition = cRoot + cDeployment;
                CQuaternion cTemp2;
                cTemp2.FromEulerAngles(cAngle*i,cY,cX);
                cTheta*=cTemp2;
                
                //DEBUG(",X: %f , Y: %f \n",fNewX,fNewY);
                
                m_pcNode = new CFootBotEntity(strId.str(),              // the id
                                              "fdc",                //controller id
                                              cPosition,       // the position in m
                                              cTheta,           // the orientation (//CQuaternion(),)
                                              1.0,                  // rab range
                                              10,                       //
                                              ToRadians(CDegrees(80.0f)));   // omnicam aperture

                
                /* Rotate deployment vector*/
                cDeployment.RotateZ(cAngle);
                
                /* Add the footbot to the space */
                AddEntity(*m_pcNode);
                m_tLevels.push_back(SNode(strId.str(),(cController.GetTreeData().Level+1),true,false));
            }
            cController.GetStartTree()=false; // Use setter functions
            bAddedRobots=true;
        }
    }
    
    /* Set flag for waiting one time step */
    if (bAddedRobots) {
        for(CSpace::TMapPerType::iterator it = m_cFootbots.begin(); it != m_cFootbots.end(); ++it){
            CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
            CFootBotConnectedMotion& cController = dynamic_cast<CFootBotConnectedMotion&>(cFootBot.GetControllableEntity().GetController());
            cController.SetWaitStep(true);
        }
    }
    
}

//            if (GetSpace().GetSimulationClock()==2) {


REGISTER_LOOP_FUNCTIONS(CConnectionLoopFunctions, "connection_loop_functions")
