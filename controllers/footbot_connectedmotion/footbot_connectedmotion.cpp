/* Include the controller definition */
#include "footbot_connectedmotion.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* Logging */
#include <argos3/core/utility/logging/argos_log.h>


/****************************************/
/****************************************/

void CFootBotConnectedMotion::STreeData::Init() {
    Level=0;
    NextLevel=0;
    AlreadyIdle=false;
    IsLeaf=false; // first node only should be considered as root
    IsRoot=true;
    StretchedRangeAngle=CVector2();
}

/****************************************/
/****************************************/

void CFootBotConnectedMotion::SWheelTurningParams::Init(TConfigurationNode& t_node) {
    try {
        TurningMechanism = NO_TURN;
        CDegrees cAngle;
        GetNodeAttribute(t_node, "hard_turn_angle_threshold", cAngle);
        HardTurnOnAngleThreshold = ToRadians(cAngle);
        GetNodeAttribute(t_node, "soft_turn_angle_threshold", cAngle);
        SoftTurnOnAngleThreshold = ToRadians(cAngle);
        GetNodeAttribute(t_node, "no_turn_angle_threshold", cAngle);
        NoTurnAngleThreshold = ToRadians(cAngle);
        GetNodeAttribute(t_node, "max_speed", MaxSpeed);
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);
    }
}

/****************************************/
/****************************************/

CFootBotConnectedMotion::SCommPacket::SCommPacket() {
    //Angle=new CRadians[MAX_SIZE];
}

/****************************************/
/****************************************/

CFootBotConnectedMotion::SCommPacket::~SCommPacket() {
    //delete [] Angle;
}

/****************************************/
/****************************************/

CFootBotConnectedMotion::CFootBotConnectedMotion() :
    m_pcWheels(NULL),
    m_pcRABA(NULL),
    m_pcRABS(NULL),
    m_pcCamera(NULL),
    m_pcLight(NULL)
    {
        m_bRobotIdle=false;
        m_bWaitStep=false;
        
        m_bStartTree=false;
        m_bRefreshDone=true;
        m_nRefreshLayer=0;
        m_nClock=1;
    }

/****************************************/
/****************************************/

void CFootBotConnectedMotion::Init(TConfigurationNode& t_node) {
    
    m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcRABA   = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing");
    m_pcRABS   = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing");
    m_pcLight  = GetSensor  <CCI_FootBotLightSensor          >("footbot_light");
    m_pcCamera = GetSensor  <CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
    /* Switch the camera on */
    m_pcCamera->Enable();
    
    try {
        /* Wheel turning */
        m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
        /* Tree update */
        m_sTreeData.Init();
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
    }
}

/****************************************/
/****************************************/

void CFootBotConnectedMotion::Reset(){
    m_bRobotIdle=false;
    //STreeData.Reset();
}

/****************************************/
/****************************************/

void CFootBotConnectedMotion::Destroy(){
    ;
}

void CFootBotConnectedMotion::DeleteParentSonId(){
    m_tParentSonId.clear();
}
/****************************************/
/****************************************/

void CFootBotConnectedMotion::ControlStep(){
    
    const std::string& strRobotId=GetId();
    int nCurrentId = FromString<int>(strRobotId.substr(2));
    
    /* Refresh Routine */
    //if (m_sTreeData.Level==m_nRefreshLayer) {
    //    ++m_nRefreshLayer;
    //}
    
    /* Leaf Routine */
    if(m_sTreeData.IsLeaf){
        
        int nCountParent=0;
        int nParentId=0;
        TCommReadings::iterator ptrParent;
        std::vector<Real> vecRanges;
        std::vector<CRadians> vecAngles;
        
        /* Get readings from RAB*/
        bool bCommON=true;
        TCommReadings cCommunication=Receive(strRobotId,bCommON);
        
        /* Act based on readings */
        if (bCommON && !m_bWaitStep) {
            for(TCommReadings::iterator it=cCommunication.begin(); it!=cCommunication.end();++it){
                /* Find new Parent by looking for minimum range within nodes one level below (level root=0) */
                if (it->Level==(m_sTreeData.Level-1)) {
                    if (nCountParent==0) {
                        ptrParent=it;
                        ++nCountParent;
                    }
                    else if(it->Range<ptrParent->Range){
                        ptrParent=it;
                    }
                }
                if (it->Range<=PULL_THRESHOLD) {
                    /* Add repulsion forces */
                    vecAngles.push_back(it->Angle);
                    vecRanges.push_back(-1*(it->Range));
                }
            }
            
            /* Save new Parent-Son node */
            nParentId=ptrParent->Id;
            m_tParentSonId.push_back(std::make_pair(nParentId,nCurrentId));
            
            /* Move if parent within range and not idle */
            if (ptrParent->Range <=STRETCH_THRESHOLD && !ptrParent->Idle) {
                /* Add task attraction force */
                CVector2 temp = VectorToTask(nCurrentId);
                vecAngles.push_back(temp.Angle());
                vecRanges.push_back(temp.Length());
                SetWheelSpeedsFromVector(AdjustmentVector(vecRanges,vecAngles));
                m_bRobotIdle=false;
                Emit(nCurrentId,m_bRobotIdle,m_sTreeData.Level,0);
            }
            /* Stay idle or create new entity otherwise */
            else{
                if(!m_sTreeData.AlreadyIdle){
                    /* Stop robot and set to idle*/
                    m_pcWheels->SetLinearVelocity(0,0);
                    m_bRobotIdle=true;
                    /* Update level (new entity will become parent to one of the parents or to the current robot)*/
                    ++m_sTreeData.Level;
                    /* Create new entity between curent robot and its parent*/
                    if (ptrParent->Range >=STRETCH_THRESHOLD){
                        m_sTreeData.StretchedRangeAngle=CVector2(ptrParent->Range,ptrParent->Angle);
                    }
                    Emit(nCurrentId,m_bRobotIdle,m_sTreeData.Level,0);
                    m_sTreeData.AlreadyIdle=true;
                }
                else {
                    m_pcWheels->SetLinearVelocity(0,0);
                    m_bRobotIdle=true;
                    Emit(nCurrentId,m_bRobotIdle,m_sTreeData.Level,0);
                    m_sTreeData.AlreadyIdle=false;
                }
            }
        }
        /* Stop if no readings */
        else {
            m_pcWheels->SetLinearVelocity(0,0);
            m_bRobotIdle=false;
            Emit(nCurrentId,m_bRobotIdle,m_sTreeData.Level,0);
            m_bWaitStep=false;
            //DEBUG(",Time step: %i, robot: %i \n",m_nClock,nCurrentId);
        }
    }
    
    /* Root Routine */
    else if(m_sTreeData.IsRoot){
        if(m_nClock==1) {
            /* Create n sons around root (n = Number of tasks) */
            m_bStartTree=true;
        }
        else if((m_nClock%20==0) && m_bRefreshDone) {
            m_bRefreshDone=false;
            m_nRefreshLayer=1;
            //LOG<<"Start of refresh"<<std::endl;
        }
        Emit(nCurrentId,false,m_sTreeData.Level,0);
        //++m_nClock;
    }
    
    /* Other Nodes Routine */
    else {
        
        std::vector<Real> vecRanges;
        std::vector<CRadians> vecAngles;
        int nParentId=0;
        int nCountParent=0;
        int nCountSons=0;
        TCommReadings::iterator ptrParent;
        
        
        /* Get readings from RAB*/
        bool bCommON=true;
        TCommReadings cCommunication=Receive(strRobotId,bCommON);
        
        if (bCommON && !m_bWaitStep) {
            
            for (TCommReadings::iterator it=cCommunication.begin(); it!=cCommunication.end();++it) {
                /* Get number of Sons */
                if(it->Level>m_sTreeData.Level){
                    ++nCountSons;
                }
            }
            
            for (TCommReadings::iterator it=cCommunication.begin(); it!=cCommunication.end();++it) {
            /* Compute forces to sons */
                if (it->Range<=PULL_THRESHOLD) {
                    /* Repulsion forces */
                    vecAngles.push_back(it->Angle);
                    vecRanges.push_back(-(it->Range));
                }
                else {
                    /* Attraction forces */
                    if(it->Level>m_sTreeData.Level){
                        /* Add attraction force to Sons */
                        vecAngles.push_back(it->Angle);
                        if (nCountSons==0) {
                            DEBUG("No Sons");
                        }
                        vecRanges.push_back(it->Range/nCountSons);
                    }
                }
                
            /* Compute forces to parents */
                
                /* Find new Parent by looking for minimum range with nodes one level below (level root=0) */
                if (it->Level==(m_sTreeData.Level-1)){
                    if (nCountParent==0) {
                        ptrParent=it;
                        ++nCountParent;
                    }
                    else if(it->Range<ptrParent->Range){
                        ptrParent=it;
                    }
                }
            }
            
            /* Add attraction force to Parent */
            vecAngles.push_back(ptrParent->Angle);
            vecRanges.push_back(ptrParent->Range);
            
            /* Save new Parent-Son node */
            nParentId=ptrParent->Id;
            m_tParentSonId.push_back(std::make_pair(nParentId,nCurrentId));
            
            /* Motion rule : */
            if(ptrParent->Range<=STRETCH_THRESHOLD && !ptrParent->Idle){
                m_bRobotIdle=false;
                Emit(nCurrentId,m_bRobotIdle,m_sTreeData.Level,0);
                SetWheelSpeedsFromVector(AdjustmentVector(vecRanges,vecAngles));
                vecRanges.clear();
                vecAngles.clear();
            }
            else {
                if(!m_sTreeData.AlreadyIdle){
                    m_pcWheels->SetLinearVelocity(0,0);
                    m_bRobotIdle=true;
                    ++m_sTreeData.Level;
                    if (ptrParent->Range>=STRETCH_THRESHOLD){
                        m_sTreeData.StretchedRangeAngle=CVector2(ptrParent->Range,ptrParent->Angle);
                    }
                    Emit(nCurrentId,m_bRobotIdle,m_sTreeData.Level,0);
                    m_sTreeData.AlreadyIdle=true;
                }
                else {
                    m_pcWheels->SetLinearVelocity(0,0);
                    m_bRobotIdle=true;
                    Emit(nCurrentId,m_bRobotIdle,m_sTreeData.Level,0);
                    m_sTreeData.AlreadyIdle=false;
                }
            }
            
        }
        else {
            m_pcWheels->SetLinearVelocity(0,0);
            m_bRobotIdle=false;
            Emit(nCurrentId,m_bRobotIdle,m_sTreeData.Level,0);
            m_bWaitStep=false;
            DEBUG(",Time step: %i, robot: %i \n",m_nClock,nCurrentId);
        }
    }
    
    ++m_nClock;
}

/****************************************/
/****************************************/

CVector2 CFootBotConnectedMotion::VectorToTask(int n_Id){

    CVector2 cTemp;
    /* Task 1 */
    if (n_Id==2){
        cTemp=VectorToBlob(argos::CColor::YELLOW);
    }
    /* Task 2 */
    else if(n_Id==3){
        cTemp=VectorToBlob(argos::CColor::RED);
    }
    /* Task 3 */
    else if (n_Id==4){
        cTemp=VectorToBlob(argos::CColor::BLUE);
    }
    /* Task 4 */
    else if(n_Id==5){
        cTemp=VectorToBlob(argos::CColor::GREEN);
    }
    /* Task 5 */
    else if(n_Id==6){
        cTemp=VectorToBlob(argos::CColor::WHITE);
    }
    /* Task 6 */
    else if(n_Id==7){
        cTemp=VectorToBlob(argos::CColor::PURPLE);
    }
    return cTemp;
}

/****************************************/
/****************************************/

CVector2 CFootBotConnectedMotion::VectorToBlob(CColor c_color) {
    /* Get light readings */
    const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sBlobs = m_pcCamera->GetReadings();
    /* Calculate a vector that points to the <color> light */
    CVector2 cTemp;
    for(size_t i = 0; i < sBlobs.BlobList.size(); ++i) {
        if (sBlobs.BlobList[i]->Color==c_color) {
            cTemp = CVector2(sBlobs.BlobList[i]->Distance, sBlobs.BlobList[i]->Angle);
        }
    }
//    if(cTemp.Length() > 0.0f) {
//        /* Make the vector long as 1/4 of the max speed */
//        cTemp.Normalize();
//        cTemp *= 0.25f * m_sWheelTurningParams.MaxSpeed;
//    }
//    cTemp/=100;
    return cTemp;
}

/****************************************/
/****************************************/

CVector2 CFootBotConnectedMotion::VectorToLight() {
    /* Get light readings */
    const CCI_FootBotLightSensor::TReadings& tReadings = m_pcLight->GetReadings();
    /* Calculate a normalized vector that points to the closest light */
    CVector2 cAccum;
    for(size_t i = 0; i < tReadings.size(); ++i) {
        cAccum += CVector2(tReadings[i].Value, tReadings[i].Angle);
    }
    if(cAccum.Length() > 0.0f) {
        /* Make the vector long as 1/4 of the max speed */
        cAccum.Normalize();
        cAccum *= 0.25f * m_sWheelTurningParams.MaxSpeed;
    }
    return cAccum;
}

/****************************************/
/****************************************/

CVector2 CFootBotConnectedMotion::AdjustmentVector(std::vector<Real> f_range,std::vector<CRadians> c_angle) {
    
    /* Calculate a normalized vector which points to the next target for the robot */
    CVector2 cAccum;
    
    for (size_t i = 0; i<f_range.size(); ++i) {
        cAccum += CVector2(f_range[i],c_angle[i]);
    }
    
    if(cAccum.Length() > 0.0f) {
        /* Make the vector long as 1/4 of the max speed */
        cAccum.Normalize();
        cAccum *= 0.25f * m_sWheelTurningParams.MaxSpeed;
    }
    return cAccum;
}

/****************************************/
/****************************************/

void CFootBotConnectedMotion::SetWheelSpeedsFromVector(const CVector2& c_heading) {
    /* Get the heading angle */
    CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
    /* Get the length of the heading vector */
    Real fHeadingLength = c_heading.Length();
    /* Clamp the speed so that it's not greater than MaxSpeed */
    Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);
    
    /* Turning state switching conditions */
    if(Abs(cHeadingAngle) <= m_sWheelTurningParams.NoTurnAngleThreshold) {
        /* No Turn, heading angle very small */
        m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
    }
    else if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
        /* Hard Turn, heading angle very large */
        m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
    }
    else if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::NO_TURN &&
            Abs(cHeadingAngle) > m_sWheelTurningParams.SoftTurnOnAngleThreshold) {
        /* Soft Turn, heading angle in between the two cases */
        m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
    }
    
    /* Wheel speeds based on current turning state */
    Real fSpeed1, fSpeed2;
    switch(m_sWheelTurningParams.TurningMechanism) {
        case SWheelTurningParams::NO_TURN: {
            /* Just go straight */
            fSpeed1 = fBaseAngularWheelSpeed;
            fSpeed2 = fBaseAngularWheelSpeed;
            break;
        }
            
        case SWheelTurningParams::SOFT_TURN: {
            /* Both wheels go straight, but one is faster than the other */
            Real fSpeedFactor = (m_sWheelTurningParams.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) / m_sWheelTurningParams.HardTurnOnAngleThreshold;
            fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
            fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
            break;
        }
            
        case SWheelTurningParams::HARD_TURN: {
            /* Opposite wheel speeds */
            fSpeed1 = -m_sWheelTurningParams.MaxSpeed;
            fSpeed2 =  m_sWheelTurningParams.MaxSpeed;
            break;
        }
    }
    
    /* Apply the calculated speeds to the appropriate wheels */
    Real fLeftWheelSpeed, fRightWheelSpeed;
    if(cHeadingAngle > CRadians::ZERO) {
        /* Turn Left */
        fLeftWheelSpeed  = fSpeed1;
        fRightWheelSpeed = fSpeed2;
    }
    else {
        /* Turn Right */
        fLeftWheelSpeed  = fSpeed2;
        fRightWheelSpeed = fSpeed1;
    }
    /* Finally, set the wheel speeds */
    m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}

/****************************************/
/****************************************/

void CFootBotConnectedMotion::Emit(UInt8 un_id, bool b_idle, UInt8 un_level,UInt8 un_NextLevel) {
    CByteArray cBuffer;
    cBuffer << un_id;
    cBuffer << static_cast<UInt8>(b_idle);
    cBuffer << un_level;
    cBuffer << un_NextLevel;
    cBuffer << static_cast<UInt16>(0);
    cBuffer << static_cast<UInt16>(0);
    cBuffer << static_cast<UInt16>(0);
    m_pcRABA->SetData(cBuffer);
}

/****************************************/
/****************************************/

CFootBotConnectedMotion::TCommReadings CFootBotConnectedMotion::Receive(const std::string& str_RobotId, bool& CommON) {
    
    int nCurrentId = FromString<int>(str_RobotId.substr(2));
    
    const CCI_RangeAndBearingSensor::TReadings& tReadings=m_pcRABS->GetReadings();
    TCommReadings tBuffer;
    SCommPacket sBuffer;
    CommON=true;
    
    for(size_t i=0; i<tReadings.size();++i){
        CByteArray cTemp;
        cTemp=tReadings[i].Data;
        sBuffer.Id=(cTemp.ToCArray()[0]);//reinterpret_cast<UInt16*>(cTemp.ToCArray()[0]);
        sBuffer.Idle=(cTemp.ToCArray()[1]);//reinterpret_cast<UInt8*>(cTemp.ToCArray()[1]);
        sBuffer.Level=(cTemp.ToCArray()[2]);//reinterpret_cast<UInt16*>(cTemp.ToCArray()[2]);
        sBuffer.NextLevel=(cTemp.ToCArray()[3]);
        sBuffer.Range=tReadings[i].Range;
        sBuffer.Angle=tReadings[i].HorizontalBearing;
        
        if (sBuffer.Id==0 && !sBuffer.Idle && sBuffer.Level==0 && tReadings.size()==1) {
            CommON=false;
        }
        
        /* Workaround for aberrant measurements*/ //still needed ?
        if (sBuffer.Range>0.1) {
        tBuffer.push_back(sBuffer);
        }

        //LOG<<nCurrentId<<'\t'<<sBuffer.Id<<'\t'<<sBuffer.Level<<'\t'<<sBuffer.Range<<'\t'<<sBuffer.Angle<<std::endl;
        //if (sBuffer.Id==0 && !sBuffer.Idle && sBuffer.Level==0) {
                //DEBUG("Buffer: current fb %u Id %u Level %u Range %f Angle %f \n",nCurrentId,sBuffer.Id[i],sBuffer.Level[i],sBuffer.Range[i]);//sBuffer.Angle[i]);
        //        LOG<<nCurrentId<<'\t'<<sBuffer.Id<<'\t'<<sBuffer.Level<<'\t'<<sBuffer.Range<<'\t'<<sBuffer.Angle<<std::endl;
        //}
        
    }
    return tBuffer;
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CFootBotConnectedMotion, "footbot_connectedmotion_controller")
