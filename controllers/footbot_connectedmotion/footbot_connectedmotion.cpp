/* Include the controller definition */
#include "footbot_connectedmotion.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* Logging */
#include <argos3/core/utility/logging/argos_log.h>


/****************************************/
/****************************************/

void CFootBotConnectedMotion::STreeData::Init() { // figure out how to initialize at different values depending on id, NOTE: clean this up, use 1 variable for Level
    LevelWkr=2;
    LevelCntr=1;
    LevelBkb=0;
    bAlreadyIdle=false;
}

/****************************************/
/****************************************/

//void CFootBotConnectedMotion::SSwarmParams::Init(TConfigurationNode& t_node) {  //needed ?? keep number of cntrs only
//    try {
//        GetNodeAttribute(t_node, "number_wkrs", NumberWkrs);
//        GetNodeAttribute(t_node, "max_cntrs", MaxCntrs);
//        GetNodeAttribute(t_node, "number_bkbs", NumberBkbs);
//        
//        WkrMaxId=NumberWkrs-1;
//        CntrMinId=NumberWkrs;
//        CntrMaxId=NumberWkrs+MaxCntrs-1;
//        BkbMinId=NumberWkrs+MaxCntrs;
//        BkbMaxId=NumberWkrs+MaxCntrs+NumberBkbs-1;
//        
//    }
//    catch(CARGoSException& ex) {
//        THROW_ARGOSEXCEPTION_NESTED("Error initializing controller swarm parameters.", ex);
//    }
//}

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
 //   delete [] angle;
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
        //m_sSwarmParams.Init(GetNode(t_node, "swarm"));
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
    LOG<<strRobotId<<std::endl;
    
    if(strRobotId.find("wkr") != std::string::npos){
        
        int nCurrentId=std::stoi(&strRobotId[3]);
        int nCountParent=0;
        int nMinIndex=0;
        int nParentId=0;
        
        /* Get readings from RAB*/
        SCommPacket cCommunication=Receive();
        size_t size=cCommunication.Size; //remove?

        if (cCommunication.CommON) {
            /* Find new Parent by looking for minimum range with nodes one level below (level root=0) */
            for(size_t i=0; i<size;++i){
                if (cCommunication.Level[i]<=(m_sTreeData.LevelWkr-1)) {
                    if (nCountParent==0) {
                        nMinIndex=i;
                        ++nCountParent;
                    }
                    else if(cCommunication.Range[i]<cCommunication.Range[nMinIndex]){
                        nMinIndex=i;
                    }
                }
            }
            
            /* Save new Parent-Son node */
            nParentId=cCommunication.Id[nMinIndex];
            m_tParentSonId.push_back(std::make_pair(nParentId,nCurrentId));
            
            if (cCommunication.Range[nMinIndex]<=90 && !cCommunication.Idle[nMinIndex]) { //remove hard coded value
                /* Perform worker task and send info through RAB (id, not idle and level in tree)*/
                DoTask(nCurrentId);
                m_bRobotIdle=false;
                Emit(nCurrentId,m_bRobotIdle,m_sTreeData.LevelWkr);
            }
            else{
                
                if(!m_sTreeData.bAlreadyIdle){
                    /* Stop robot and set to idle*/
                    m_pcWheels->SetLinearVelocity(0,0);
                    m_bRobotIdle=true;
                    /* Update level (either new entity to become parent to one of the parents or to the current robot*/
                    ++m_sTreeData.LevelWkr;
                    /* Create new entity between curent robot and its parent*/
                    if (cCommunication.Range[nMinIndex]>=90){
                        CVector2 temp;
                        temp.FromPolarCoordinates(cCommunication.Range[nMinIndex],cCommunication.Angle[nMinIndex]);
                        m_sTreeData.InfoNewNode=temp;
                    }
                    Emit(nCurrentId,m_bRobotIdle,m_sTreeData.LevelWkr);
                    m_sTreeData.bAlreadyIdle=true;
                }
                else {
                    m_pcWheels->SetLinearVelocity(0,0);
                    m_bRobotIdle=true;
                    Emit(nCurrentId,m_bRobotIdle,m_sTreeData.LevelWkr);
                    m_sTreeData.bAlreadyIdle=false;
                }
            }
        }
        else {
            m_bRobotIdle=false;
            Emit(nCurrentId,m_bRobotIdle,m_sTreeData.LevelWkr);
        }
    
    }
    
    else if(strRobotId.find("cntr") != std::string::npos) {
        
        int nCurrentId=std::stoi(&strRobotId[4]);
        std::vector<Real> vecRanges;
        std::vector<CRadians> vecAngles;
        int nMinIndex=0;
        int nParentId=0;
        int nCountParent=0;
        int nCountSons=0;
        
        /* Get readings from RAB*/
        SCommPacket cCommunication=Receive();
        size_t size=cCommunication.Size; //needed ? size needed in CommPacket ?
        
        if (cCommunication.CommON) {
            
            /* Get number of Sons */
            for (size_t i=0; i<size;++i) {
                if(cCommunication.Level[i]>m_sTreeData.LevelCntr){
                    ++nCountSons;
                }
            }
            for(size_t i=0; i<size;++i){
                if (cCommunication.Range[i]<=30) {
                    /* Repulsion forces */
                    vecAngles.push_back(cCommunication.Angle[i]);
                    vecRanges.push_back(-cCommunication.Range[i]);
                }
                else {
                    /* Attraction forces */
                    if(cCommunication.Level[i]>m_sTreeData.LevelCntr){
                        /* Add attraction force to Sons */
                        vecAngles.push_back(cCommunication.Angle[i]);
                        vecRanges.push_back(cCommunication.Range[i]/nCountSons);
                    }
                }
                /* Find new Parent by looking for minimum range with nodes one level below (level root=0) */
                if (cCommunication.Level[i]<=(m_sTreeData.LevelCntr-1)){
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
            if(cCommunication.Range[nMinIndex]<=90 && !cCommunication.Idle[nMinIndex]){
                m_bRobotIdle=false;
                Emit(nCurrentId,m_bRobotIdle,m_sTreeData.LevelCntr);
                SetWheelSpeedsFromVector(AdjustmentVector(vecRanges,vecAngles));
                vecRanges.clear();
                vecAngles.clear();
            }
            else {
                if(!m_sTreeData.bAlreadyIdle){
                    m_pcWheels->SetLinearVelocity(0,0);
                    m_bRobotIdle=true;
                    ++m_sTreeData.LevelCntr;
                    if (cCommunication.Range[nMinIndex]>=90){//enough?
                        CVector2 temp;
                        temp.FromPolarCoordinates(cCommunication.Range[nMinIndex],cCommunication.Angle[nMinIndex]);
                        m_sTreeData.InfoNewNode=temp;
                    }
                    Emit(nCurrentId,m_bRobotIdle,m_sTreeData.LevelCntr);
                    m_sTreeData.bAlreadyIdle=true;
                }
                else {
                    m_pcWheels->SetLinearVelocity(0,0);
                    m_bRobotIdle=true;
                    Emit(nCurrentId,m_bRobotIdle,m_sTreeData.LevelCntr);
                    m_sTreeData.bAlreadyIdle=false;
                }
            }
            
        }
        else {
            m_bRobotIdle=false;
            Emit(nCurrentId,m_bRobotIdle,m_sTreeData.LevelCntr);
        }
    }
    
    else if(strRobotId.find("fb") != std::string::npos){
        Emit(std::stoi(&strRobotId[2]),false,m_sTreeData.LevelBkb); //checks to be added ?
    }
    
    else {
        LOGERR << "We can't be here, there's a bug!" << std::endl;
    }
    
}

/****************************************/
/****************************************/

void CFootBotConnectedMotion::DoTask(int n_Id){

    // Task 0
    if (n_Id==0){
        SetWheelSpeedsFromVector(VectorToBlob(argos::CColor::YELLOW));
    }
    //Task 1
    else if(n_Id==1){
        SetWheelSpeedsFromVector(VectorToBlob(argos::CColor::RED));
    }
    // Task 2
    else if (n_Id==2){
        SetWheelSpeedsFromVector(VectorToBlob(argos::CColor::BLUE));
    }
    // Task 3
    else if(n_Id==3){
        SetWheelSpeedsFromVector(VectorToBlob(argos::CColor::GREEN));
    }
    // Task 4
    else if(n_Id==4){
        SetWheelSpeedsFromVector(VectorToBlob(argos::CColor::WHITE));
    }
    
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

CVector2 CFootBotConnectedMotion::AdjustmentVector(std::vector<Real> f_range,std::vector<CRadians> c_angle) {
    
    /* Calculate a normalized vector that points to the adjusted position for the connector robot */
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
        sBuffer.Id[i]=reinterpret_cast<UInt8>(cTemp.ToCArray()[0]);
        sBuffer.Idle[i]=reinterpret_cast<UInt8>(cTemp.ToCArray()[1]);
        sBuffer.Level[i]=reinterpret_cast<UInt8>(cTemp.ToCArray()[2]);
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
