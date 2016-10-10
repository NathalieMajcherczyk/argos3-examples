/* Include the controller definition */
#include "footbot_connectedmotion.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* Logging */
#include <argos3/core/utility/logging/argos_log.h>

/****************************************/
/****************************************/

void CFootBotConnectedmotion::SWheelTurningParams::Init(TConfigurationNode& t_node) {
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

CFootBotConnectedmotion::CPacket::CPacket() {
    angle=new CRadians[MAX_SIZE];
}

/****************************************/
/****************************************/

CFootBotConnectedmotion::CPacket::~CPacket() {
 //   delete [] angle;
    ;
}

/****************************************/
/****************************************/

CFootBotConnectedmotion::CFootBotConnectedmotion() :
    m_pcWheels(NULL),
    m_pcRABA(NULL),
    m_pcRABS(NULL),
    m_pcLight(NULL)
    {
        m_RobotIdle=true;
        m_BackboneIt=3; //number id of first backbone robot +1
        m_ConnectorIt=0;
    }

/****************************************/
/****************************************/

void CFootBotConnectedmotion::Init(TConfigurationNode& t_node) {
    
    m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcRABA   = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing");
    m_pcRABS   = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing");
    m_pcLight  = GetSensor  <CCI_FootBotLightSensor          >("footbot_light");
    
    try {
        /* Wheel turning */
        m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
    }
}

/****************************************/
/****************************************/

void CFootBotConnectedmotion::Destroy(){
    ;
}

/****************************************/
/****************************************/

void CFootBotConnectedmotion::ControlStep(){
    
   const std::string& t_RobotID=GetId();
   LOG<<t_RobotID<<std::endl;
    
    if(t_RobotID=="wkr"){
    
        CPacket t_comm=Receive();
        int indexCnctr=0;
        size_t size=t_comm.size;
        
        if (size!=0) {
            for(size_t i=0; i<size;++i){
                if(t_comm.id[i]==1){
                    indexCnctr=i;
                    if (t_comm.range[indexCnctr]<=90 && t_comm.idle[indexCnctr]) { //remove hard coded value
                        SetWheelSpeedsFromVector(VectorToLight());
                        m_RobotIdle=false;
                        Emit(0,m_RobotIdle);
                        m_ConnectorIt=0;
                    }
                    else{
                        m_pcWheels->SetLinearVelocity(0,0);
                        m_RobotIdle=true;
                        Emit(0,m_RobotIdle);
                    }
                }
            }
        }
    }
    
    else if(t_RobotID=="cnctr") {
    
        CPacket t_comm=Receive();
        int indexWkr=0;
        int indexBackbone=0;
        size_t size=t_comm.size;
        CRadians t_angleWkr;
        CRadians t_angleBackbone;
        
        if (size!=0) {
            
            /*Find communication info from worker and "chosen" backbone robot*/
            for(size_t i=0; i<size;++i){
                if(t_comm.id[i]==0){
                    indexWkr=i;
                    t_angleWkr=t_comm.angle[i]; //to be removed, bug when t_comm.angle used in function call
                }
                else if (t_comm.id[i]==m_BackboneIt) {
                    indexBackbone=i;
                    t_angleBackbone=t_comm.angle[i]; //to be removed, bug when t_comm.angle used in function call
                }
            }
            
            /*Stay idle or move*/
            if (t_comm.idle[indexWkr]==false) {
                m_RobotIdle=true;
                Emit(1,m_RobotIdle);
                m_pcWheels->SetLinearVelocity(0,0);
            }
            else {
                
                /*Select backbone robot to use for maintaining communication
                 at beginning of motion */
                
                if (m_ConnectorIt==0) {
                        ++m_BackboneIt; // to be replaced by selection rule (use right triangle)
                        ++m_ConnectorIt; // to be replaced by boolean flag
                }
                
                /* Motion rule : keep moving as long as the "connector" does not make a right angle with the
                 selected backbone robot (condition on angle or range)*/
                
                if(t_comm.range[indexBackbone]>60.1){ // hard coded value to be replaced (right triangle)
                    m_RobotIdle=false;
                    Emit(1,m_RobotIdle);
                    SetWheelSpeedsFromVector(AdjustmentVector(t_comm.range[indexWkr],t_comm.range[indexBackbone],t_angleWkr,t_angleBackbone));
                }
                else {
                    m_RobotIdle=true;
                    Emit(1,m_RobotIdle);
                    m_pcWheels->SetLinearVelocity(0,0);
                }
            }
            
        }
        else {
            m_RobotIdle=true;
            Emit(1,m_RobotIdle);
        }
    }
    
    else if(t_RobotID.find("fb") != std::string::npos){
        
        Emit(t_RobotID[2],true);
    }
    
    else {
        LOGERR << "We can't be here, there's a bug!" << std::endl;
    }
    
}

/****************************************/
/****************************************/

CVector2 CFootBotConnectedmotion::VectorToLight() {
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

CVector2 CFootBotConnectedmotion::AdjustmentVector(Real range1, Real range2, CRadians angle1, CRadians angle2) {
    
    /* Calculate a normalized vector that points to the adjusted position for the connector robot */
    CVector2 cAccum;
    
    cAccum += CVector2(range1,angle1);
    cAccum += CVector2(range2,angle2);
    
    if(cAccum.Length() > 0.0f) {
        /* Make the vector long as 1/4 of the max speed */
        cAccum.Normalize();
        cAccum *= 0.25f * m_sWheelTurningParams.MaxSpeed;
    }
    
    return cAccum;
}


/****************************************/
/****************************************/

void CFootBotConnectedmotion::SetWheelSpeedsFromVector(const CVector2& c_heading) {
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

void CFootBotConnectedmotion::Emit(UInt8 t_id, bool Idle) {
    CByteArray buf;
    buf << t_id;
    buf << static_cast<UInt8>(Idle);
    buf << static_cast<UInt32>(0);
    buf << static_cast<UInt16>(0);
    buf << static_cast<UInt16>(0);
    m_pcRABA->SetData(buf);
}

/****************************************/
/****************************************/

CFootBotConnectedmotion::CPacket CFootBotConnectedmotion::Receive() {
    
    const CCI_RangeAndBearingSensor::TReadings& tReadings=m_pcRABS->GetReadings();
    CPacket buf;
    for(size_t i=0; i<tReadings.size();++i){
        CByteArray temp;
        temp=tReadings[i].Data;
        buf.size=tReadings.size();
        buf.id[i]=reinterpret_cast<UInt8>(temp.ToCArray()[0]);
        buf.idle[i]= reinterpret_cast<UInt8>(temp.ToCArray()[1]);
        buf.range[i]=tReadings[i].Range;
        buf.angle[i]=tReadings[i].HorizontalBearing;
        //LOG<<buf.angle[i]<<std::endl;
        //LOG<<buf.range[i]<<std::endl;
    }
    //LOG<<buf.size<<std::endl;
    return buf;
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CFootBotConnectedmotion, "footbot_connectedmotion_controller")
