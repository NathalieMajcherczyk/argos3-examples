/* Include the controller definition */
#include "footbot_avoidance.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>

//static CRadians SPACING = CRadians(ARGOS_PI / 12.0f);
//static CRadians START_ANGLE = SPACING * 0.5f;
//for(size_t i = 0; i < 24; ++i) {
//      m_tReadings[i].Angle = START_ANGLE + i * SPACING;
//      m_tReadings[i].Angle.SignedNormalize();
//    }

/****************************************/
/****************************************/

CFootBotAvoidance::CFootBotAvoidance() :
m_pcWheels(NULL),
m_pcProximity(NULL),
m_fWheelVelocity(2.5f){
}

/****************************************/
/****************************************/

void CFootBotAvoidance::Init(TConfigurationNode& t_node) {
    /*
     * Get sensor/actuator handles
     *
     * The passed string (ex. "differential_steering") corresponds to the
     * XML tag of the device whose handle we want to have. For a list of
     * allowed values, type at the command prompt:
     *
     * $ argos3 -q actuators
     *
     * to have a list of all the possible actuators, or
     *
     * $ argos3 -q sensors
     *
     * to have a list of all the possible sensors.
     *
     * NOTE: ARGoS creates and initializes actuators and sensors
     * internally, on the basis of the lists provided the configuration
     * file at the <controllers><footbot_avoidance><actuators> and
     * <controllers><footbot_avoidance><sensors> sections. If you forgot to
     * list a device in the XML and then you request it here, an error
     * occurs.
     */
    
    m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
    /*
     * Parse the configuration file
     *
     * The user defines this part. Here, the algorithm accepts three
     * parameters and it's nice to put them in the config file so we don't
     * have to recompile if we want to try other settings.
     */

    GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
}

/****************************************/
/****************************************/

void CFootBotAvoidance::ControlStep() {
    
    const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
    m_Value.clear();
    for (size_t i = 0; i < tProxReads.size(); ++i) {
        Real temp=tProxReads[i].Value;
        m_Value.push_back(temp);
    }
    
    if(m_Value.size()!=0){
        int max=distance(m_Value.begin(),max_element(m_Value.begin(), m_Value.end()));
        Real fMaxReadVal=m_Value.at(max);
        if(fMaxReadVal > 0.0f) {
            if (max>=0 && max<=5) {
                /*Obstacle on the left, turn right*/
                m_pcWheels->SetLinearVelocity(m_fWheelVelocity,0.0f);
            }
            else if (max>=17 && max<=23) {
                /*Obstacle on the right, turn left*/
                m_pcWheels->SetLinearVelocity(0.0f,m_fWheelVelocity);
            }
            else {
                m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
            }
            
        }
        else {
            m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
        }
    }
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotAvoidance, "footbot_avoidance_controller")
