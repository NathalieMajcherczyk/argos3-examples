/*
 * AUTHOR:
 *
 * Controller for the foot-bot.
 *
 * This controller
 *
 * This controller is meant to be used with the XML files:
 *    experiments/connectedmotion.argos
 */

#ifndef FOOTBOT_CONNECTEDMOTION_H
#define FOOTBOT_CONNECTEDMOTION_H

/*
 * Include some necessary headers.
 */

/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the RAB actuator class */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the RAB sensor class */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
/* Definition of the light sensor class */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>



using namespace argos;

class CFootBotConnectedmotion : public CCI_Controller {
    

public:
    
    /*
     * The following variables are used as parameters for
     * turning during navigation.
     */
    
    struct SWheelTurningParams {
        /*
         * The turning mechanism.
         * The robot can be in three different turning states.
         */
        enum ETurningMechanism
        {
            NO_TURN = 0, // go straight
            SOFT_TURN,   // both wheels are turning forwards, but at different speeds
            HARD_TURN    // wheels are turning with opposite speeds
        } TurningMechanism;
        /*
         * Angular thresholds to change turning state.
         */
        CRadians HardTurnOnAngleThreshold;
        CRadians SoftTurnOnAngleThreshold;
        CRadians NoTurnAngleThreshold;
        /* Maximum wheel speed */
        Real MaxSpeed;
        
        void Init(TConfigurationNode& t_tree);
    };

public:
    
    const static int MAX_SIZE = 10;
    
    /*Variables used for communication between robots*/
    
    struct CPacket {
        
        UInt8 id[MAX_SIZE];
        bool idle[MAX_SIZE];
        Real range[MAX_SIZE];
        CRadians* angle;
        size_t size;
        
        CPacket();
        ~CPacket();
    };
    
public:
    
    /* Class constructor. */
    CFootBotConnectedmotion();
    
    /* Class destructor. */
    virtual ~CFootBotConnectedmotion() {
    }
    
    /*
     * This function initializes the controller.
     */
    virtual void Init(TConfigurationNode& t_node);
    
    /*
     * This function is called once every time step.
     */
    virtual void ControlStep();
    
    /*
     * This function resets the controller to its state right after the
     * Init().
     * It is called when you press the reset button in the GUI.
     */
    virtual void Reset() {}
    
    /*
     * Called to cleanup what done by Init() when the experiment finishes.
     */
    virtual void Destroy();
    
protected:
    
    /*
     * Calculates the vector to the closest light.
     */
    virtual CVector2 VectorToLight();
    
    virtual CVector2 AdjustmentVector(Real range1, Real range2, CRadians angle1, CRadians angle2);
    
    /*
     * Gets a direction vector as input and transforms it into wheel actuation.
     */
    void SetWheelSpeedsFromVector(const CVector2& c_heading);
    
private:
    
    /* Pointer to the differential steering actuator */
    CCI_DifferentialSteeringActuator* m_pcWheels;
    /* Pointer to the range and bearing actuator */
    CCI_RangeAndBearingActuator*  m_pcRABA;
    /* Pointer to the range and bearing sensor */
    CCI_RangeAndBearingSensor* m_pcRABS;
    /* Pointer to the foot-bot light sensor */
    CCI_FootBotLightSensor* m_pcLight;
    /* The turning parameters. */
    SWheelTurningParams m_sWheelTurningParams;

    
    bool m_RobotIdle;
    void Emit(UInt8 t_id,bool Idle);
    int m_BackboneIt;
    int m_ConnectorIt;
    CPacket Receive(); // not sure if needed

    
};

#endif
