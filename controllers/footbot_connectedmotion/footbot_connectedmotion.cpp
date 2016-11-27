/* Include the controller definition */
#include "footbot_connectedmotion.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* Logging */
#include <argos3/core/utility/logging/argos_log.h>

//lamport clock

/****************************************/
/****************************************/

void CFootBotConnectedMotion::STreeData::Init() {
    Level=0;
    NextLevel=0;
    AlreadyIdle=false;
    IsLeaf=false; // first node should only be considered as root
    IsRoot=true;
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
    Angle=new CRadians[MAX_SIZE];
}

/****************************************/
/****************************************/

CFootBotConnectedMotion::SCommPacket::~SCommPacket() {
 //   delete [] Angle;
    ;
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
        m_bStartTree=false;
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
    //LOG<<strRobotId<<std::endl;
    int nCurrentId=std::stoi(&strRobotId[2]);
    
    if(m_sTreeData.IsLeaf){
        LOG<<"Leaf:"<<strRobotId<<std::endl;
        int nCountParent=0;
        int nMinIndex=0;
        int nParentId=0;
        std::vector<Real> vecRanges;
        std::vector<CRadians> vecAngles;
        
        /* Get readings from RAB*/
        SCommPacket cCommunication=Receive();
        size_t size=cCommunication.Size;
        
        /* Act based on readings */
        if (cCommunication.CommON) {
            
            
            for(size_t i=0; i<size;++i){
                /* Find new Parent by looking for minimum range within nodes one level below (level root=0) */
                if (cCommunication.Level[i]<=(m_sTreeData.Level-1)) {
                    if (nCountParent==0) {
                        nMinIndex=i;
                        ++nCountParent;
                    }
                    else if(cCommunication.Range[i]<cCommunication.Range[nMinIndex]){
                        nMinIndex=i;
                    }
                }
                if (cCommunication.Range[i]<=PULL_THRESHOLD) {
                    /* Repulsion forces */
                    vecAngles.push_back(cCommunication.Angle[i]);
                    vecRanges.push_back(-0.5*cCommunication.Range[i]);
                }
            }
            
            /* Save new Parent-Son node */
            nParentId=cCommunication.Id[nMinIndex];
            m_tParentSonId.push_back(std::make_pair(nParentId,nCurrentId));
            
            if (cCommunication.Range[nMinIndex]<=STRETCH_THRESHOLD && !cCommunication.Idle[nMinIndex]) {
                /* Perform worker task*/
                CVector2 temp = VectorToTask(nCurrentId);
                vecAngles.push_back(temp.Angle());
                vecRanges.push_back(temp.Length());
                SetWheelSpeedsFromVector(AdjustmentVector(vecRanges,vecAngles));
                vecRanges.clear();
                vecAngles.clear();
                m_bRobotIdle=false;
                Emit(nCurrentId,m_bRobotIdle,m_sTreeData.Level);
            }
            else{
                
                if(!m_sTreeData.AlreadyIdle){
                    /* Stop robot and set to idle*/
                    m_pcWheels->SetLinearVelocity(0,0);
                    m_bRobotIdle=true;
                    /* Update level (new entity will become parent to one of the parents or to the current robot)*/
                    ++m_sTreeData.Level;
                    LOG<<"Level:"<<m_sTreeData.Level<<std::endl;
                    /* Create new entity between curent robot and its parent*/
                    if (cCommunication.Range[nMinIndex]>=STRETCH_THRESHOLD){
                        m_sTreeData.StretchedRangeAngle=CVector2(cCommunication.Range[nMinIndex],cCommunication.Angle[nMinIndex]);
                    }
                    Emit(nCurrentId,m_bRobotIdle,m_sTreeData.Level);
                    m_sTreeData.AlreadyIdle=true;
                }
                else {
                    m_pcWheels->SetLinearVelocity(0,0);
                    m_bRobotIdle=true;
                    Emit(nCurrentId,m_bRobotIdle,m_sTreeData.Level);
                    m_sTreeData.AlreadyIdle=false;
                }
            }
        }
        /* Stay idle if no readings */
        else {
            m_bRobotIdle=false;
            Emit(nCurrentId,m_bRobotIdle,m_sTreeData.Level);
        }
    
    }
    
    else if(m_sTreeData.IsRoot){
        LOG<<"Root:"<<strRobotId<<std::endl;
        if(m_nClock==1) {
            /* Create n sons around root (n = Number of tasks) */
            m_bStartTree=true;
        }
        Emit(nCurrentId,false,m_sTreeData.Level);
        ++m_nClock;
        //GetSimulationClock() ?
    }
    
    else {
        LOG<<"Other:"<<strRobotId<<std::endl;
        std::vector<Real> vecRanges;
        std::vector<CRadians> vecAngles;
        int nMinIndex=0;
        int nParentId=0;
        int nCountParent=0;
        int nCountSons=0;
        
        /* Get readings from RAB*/
        SCommPacket cCommunication=Receive();
        size_t size=cCommunication.Size; //size needed in CommPacket ?
        
        if (cCommunication.CommON) {
            
            /* Get number of Sons */
            for (size_t i=0; i<size;++i) {
                if(cCommunication.Level[i]>m_sTreeData.Level){
                    ++nCountSons;
                }
            }
            for(size_t i=0; i<size;++i){
                if (cCommunication.Range[i]<=PULL_THRESHOLD) {
                    /* Repulsion forces */
                    vecAngles.push_back(cCommunication.Angle[i]);
                    vecRanges.push_back(-cCommunication.Range[i]);
                }
                else {
                    /* Attraction forces */
                    if(cCommunication.Level[i]>m_sTreeData.Level){
                        /* Add attraction force to Sons */
                        vecAngles.push_back(cCommunication.Angle[i]);
                        vecRanges.push_back(cCommunication.Range[i]/nCountSons);
                    }
                }
                /* Find new Parent by looking for minimum range with nodes one level below (level root=0) */
                if (cCommunication.Level[i]==(m_sTreeData.Level-1)){
                    if (nCountParent==0) {
                        nMinIndex=i;
                        ++nCountParent;
                    }
                    else if(cCommunication.Range[i]<cCommunication.Range[nMinIndex]){
                        nMinIndex=i;
                    }
                }
            }
            
            /* Add attraction force to Parent */
            vecAngles.push_back(cCommunication.Angle[nMinIndex]);
            vecRanges.push_back(cCommunication.Range[nMinIndex]);
            
            /* Save new Parent-Son node */
            nParentId=cCommunication.Id[nMinIndex];
            m_tParentSonId.push_back(std::make_pair(nParentId,nCurrentId));
            
            
            /* Motion rule : */
            if(cCommunication.Range[nMinIndex]<=STRETCH_THRESHOLD && !cCommunication.Idle[nMinIndex]){
                m_bRobotIdle=false;
                Emit(nCurrentId,m_bRobotIdle,m_sTreeData.Level);
                SetWheelSpeedsFromVector(AdjustmentVector(vecRanges,vecAngles));
                vecRanges.clear();
                vecAngles.clear();
            }
            else {
                if(!m_sTreeData.AlreadyIdle){
                    m_pcWheels->SetLinearVelocity(0,0);
                    m_bRobotIdle=true;
                    ++m_sTreeData.Level;
                    if (cCommunication.Range[nMinIndex]>=STRETCH_THRESHOLD){
                        m_sTreeData.StretchedRangeAngle=CVector2(cCommunication.Range[nMinIndex],cCommunication.Angle[nMinIndex]);
                    }
                    Emit(nCurrentId,m_bRobotIdle,m_sTreeData.Level);
                    m_sTreeData.AlreadyIdle=true;
                }
                else {
                    m_pcWheels->SetLinearVelocity(0,0);
                    m_bRobotIdle=true;
                    Emit(nCurrentId,m_bRobotIdle,m_sTreeData.Level);
                    m_sTreeData.AlreadyIdle=false;
                }
            }
            
        }
        else {
            m_bRobotIdle=false;
            Emit(nCurrentId,m_bRobotIdle,m_sTreeData.Level);
        }
    }
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
    /* Calculate a normalized vector that points to the <color> light */
    CVector2 cTemp;
    for(size_t i = 0; i < sBlobs.BlobList.size(); ++i) {
        if (sBlobs.BlobList[i]->Color==c_color) {
            cTemp = CVector2(sBlobs.BlobList[i]->Distance, sBlobs.BlobList[i]->Angle);
        }
    }
    if(cTemp.Length() > 0.0f) {
        /* Make the vector long as 1/4 of the max speed */
        cTemp.Normalize();
        cTemp *= 0.25f * m_sWheelTurningParams.MaxSpeed;
    }
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

void CFootBotConnectedMotion::Emit(UInt8 un_id, bool b_idle, UInt8 un_level) {
    CByteArray cBuffer;
    cBuffer << un_id;
    cBuffer << static_cast<UInt8>(b_idle);
    cBuffer << un_level;
    cBuffer << static_cast<UInt8>(0);
    cBuffer << static_cast<UInt16>(0);
    cBuffer << static_cast<UInt16>(0);
    cBuffer << static_cast<UInt16>(0);
    m_pcRABA->SetData(cBuffer);
}

/****************************************/
/****************************************/

CFootBotConnectedMotion::SCommPacket CFootBotConnectedMotion::Receive() {
    
    const CCI_RangeAndBearingSensor::TReadings& tReadings=m_pcRABS->GetReadings();
    SCommPacket sBuffer;
    for(size_t i=0; i<tReadings.size();++i){
        CByteArray cTemp;
        cTemp=tReadings[i].Data;
        sBuffer.Size=tReadings.size();
        sBuffer.Id[i]=(cTemp.ToCArray()[0]);//reinterpret_cast<UInt16*>(cTemp.ToCArray()[0]);
        sBuffer.Idle[i]=(cTemp.ToCArray()[1]);//reinterpret_cast<UInt8*>(cTemp.ToCArray()[1]);
        sBuffer.Level[i]=(cTemp.ToCArray()[2]);//reinterpret_cast<UInt8*>(cTemp.ToCArray()[2]);
        sBuffer.Range[i]=tReadings[i].Range;
        sBuffer.Angle[i]=tReadings[i].HorizontalBearing;
    }
    sBuffer.CommON=true;
    if (sBuffer.Id[0]==0 && !sBuffer.Idle[0] && sBuffer.Level[0]==0) {
        sBuffer.CommON=false;
    }
    return sBuffer;
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CFootBotConnectedMotion, "footbot_connectedmotion_controller")
