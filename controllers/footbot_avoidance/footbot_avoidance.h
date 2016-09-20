/*
 * AUTHOR: Carlo Pinciroli <cpinciro@ulb.ac.be>
 *
 * An example avoidance controller for the foot-bot.
 *
 * This controller makes the robots behave as gas particles. The robots
 * go straight until they get close enough to another robot, in which
 * case they turn, loosely simulating an elastic collision. The net effect
 * is that over time the robots diffuse in the environment.
 *
 * The controller uses the proximity sensor to detect obstacles and the
 * wheels to move the robot around.
 *
 * This controller is meant to be used with the XML files:
 *    experiments/AVOIDANCE_1.argos
 *    experiments/AVOIDANCE_10.argos
 */

#ifndef FOOTBOT_AVOIDANCE_H
#define FOOTBOT_AVOIDANCE_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
//#include <argos3/core/utility/math/angles.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>


/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CFootBotAvoidance : public CCI_Controller {
    
public:
    
    /* Class constructor. */
    CFootBotAvoidance();
    
    /* Class destructor. */
    virtual ~CFootBotAvoidance() {
//    LOG << "destroyed";
    }
    
    /*
     * This function initializes the controller.
     * The 't_node' variable points to the <parameters> section in the XML
     * file in the <controllers><footbot_avoidance_controller> section.
     */
    virtual void Init(TConfigurationNode& t_node);
    
    /*
     * This function is called once every time step.
     * The length of the time step is set in the XML file.
     */
    virtual void ControlStep();
    
    /*
     * This function resets the controller to its state right after the
     * Init().
     * It is called when you press the reset button in the GUI.
     * In this example controller there is no need for resetting anything,
     * so the function could have been omitted. It's here just for
     * completeness.
     */
    virtual void Reset() {}
    
    /*
     * Called to cleanup what done by Init() when the experiment finishes.
     * In this example controller there is no need for clean anything up,
     * so the function could have been omitted. It's here just for
     * completeness.
     */
    virtual void Destroy() {}
    
    //CVector2 m_ProxReads(Real f_length, const CRadians & f_angle);
    //CCI_FootBotProximitySensor::TReadings& GetReads() const;
    
    typedef std::vector<Real> RVector;
    
    
    RVector m_Value;
    
private:
    
    //CCI_FootBotProximitySensor::TReadings& tProxReads;
    
    //CRadians tAngle[24];
    //const CRadians* ptAngle;
    //static CRadians SPACING;
    //static CRadians START_ANGLE;
    
    /* Pointer to the foot-bot proximity sensor */
    CCI_FootBotProximitySensor* m_pcProximity;
    
    /* Pointer to the differential steering actuator */
    CCI_DifferentialSteeringActuator* m_pcWheels;
    
    /*
     * The following variables are used as parameters for the
     * algorithm. You can set their value in the <parameters> section
     * of the XML configuration file, under the
     * <controllers><footbot_avoidance_controller> section.
     */
    
    /* Wheel speed. */
    Real m_fWheelVelocity;
    
};

#endif
