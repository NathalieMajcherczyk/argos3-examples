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
/* Definition of the camera sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
/* Color definition*/
#include <argos3/core/utility/datatypes/color.h>

using namespace argos;

class CFootBotConnectedMotion : public CCI_Controller {
    

public:
    
    struct STreeData {
        UInt8 LevelWkr;
        UInt8 LevelCntr;
        UInt8 LevelBkb;
        CVector2 InfoNewNode;
        bool bAlreadyIdle;
        void Init();
    };
    

    
//    struct SSwarmParams {
//       
//        int NumberWkrs;
//        int MaxCntrs;
//        int NumberBkbs;
//        
//        int WkrMaxId;
//        int CntrMinId;
//        int CntrMaxId;
//        int BkbMinId;
//        int BkbMaxId;
//        
//        void Init(TConfigurationNode& t_tree);
//    };

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
    
    const static int MAX_SIZE = 50; //remove and replace by pointers
    
    /*Variables used for communication between robots*/
    
    struct SCommPacket {
        
        UInt8 Id[MAX_SIZE];
        bool Idle[MAX_SIZE];
        UInt8 Level[MAX_SIZE];
        bool CommON;
        Real Range[MAX_SIZE];
        CRadians* Angle;
        size_t Size;
        
        SCommPacket();
        ~SCommPacket();
    };
    
public:
    
    /* Class constructor. */
    CFootBotConnectedMotion();
    
    /* Class destructor. */
    virtual ~CFootBotConnectedMotion() {
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
    virtual void Reset();
    
    /*
     * Called to cleanup what done by Init() when the experiment finishes.
     */
    virtual void Destroy();
    
    typedef std::vector< std::pair <int, int> > TPairVector;
    
    /*
     * Returns the son-parent pairs
     */
    TPairVector inline & GetParentSonId (){
        return m_tParentSonId;
    }
    /*
     * Returns
     */
    STreeData inline & GetTreeData(){
        return m_sTreeData;
    }
    
    /*
     * Deletes son-parent pairs
     */
    virtual void DeleteParentSonId (); //needed ?
    
    
protected:
    
    void DoTask(int n_Id);
    
    /*
     * Calculates the vector to the closest light.
     */
    virtual CVector2 VectorToBlob(CColor c_color);
    
    virtual CVector2 AdjustmentVector(std::vector<Real> f_range,std::vector<CRadians> c_angle);
    
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
    
//    /* The swarm parameters. */
//    SSwarmParams m_sSwarmParams;

    /* Pointer to the omnidirectional camera sensor */
    CCI_ColoredBlobOmnidirectionalCameraSensor* m_pcCamera;
    
    bool m_bRobotIdle;
    TPairVector m_tParentSonId;
    STreeData m_sTreeData;
    
    void Emit(UInt8 un_id,bool b_idle, UInt8 un_level);
    SCommPacket Receive();
    
};

#endif
