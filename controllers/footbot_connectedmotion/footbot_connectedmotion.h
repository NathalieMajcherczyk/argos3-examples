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

// Okay to use const here ?
const Real STRETCH_THRESHOLD = 90;
const Real PULL_THRESHOLD = 40;
const Real NUMBER_OF_TASKS = 6; // set a as parameters

class CFootBotConnectedMotion : public CCI_Controller {
    
public:
    
    struct STreeData {
        UInt8 Level;
        UInt8 NextLevel;
        bool IsLeaf;
        bool IsRoot;
        bool AlreadyIdle;
        CVector2 StretchedRangeAngle; // Range and angle between nodes stretched more than the allowed range
        
        /* Default Constructor */
        STreeData(): Level(0), NextLevel(0), AlreadyIdle(false), IsLeaf(false), IsRoot(false) {}
        
        /* Constructor */
        STreeData(UInt8 Level, UInt8 NextLevel, bool AlreadyIdle, bool IsLeaf, bool IsRoot): Level(Level), NextLevel(NextLevel), AlreadyIdle(AlreadyIdle), IsLeaf(IsLeaf), IsRoot(IsRoot) {}
        
        void Init();
    };
    
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
    
//    struct MatchId {
//        MatchId(const UInt8& id): _id(id) {}
//        bool operator()(const SCommPacket& obj) const
//        {
//            return obj. == _id;
//        }
//        private :
//            const UInt8& _id;
//    };
    
    /*Variables used for communication between robots*/
    
    struct SCommPacket {
        
        UInt8 Id;
        bool Idle;
        UInt8 Level;
        UInt8 NextLevel;
        Real Range;
        CRadians Angle;
        
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
    typedef std::vector<SCommPacket> TCommReadings;
    
    /* Getter functions */
    
    /* Returns son-parent pairs */
    TPairVector inline & GetParentSonId (){
        return m_tParentSonId;
    }
    /* Returns tree data */
    STreeData inline & GetTreeData(){
        return m_sTreeData;
    }
    
    /* Returns tree start flag */
    bool inline & GetStartTree(){
        return m_bStartTree;
    }
    
    /* Set wait step flag */
    
    void inline SetWaitStep(bool b_wait){
        m_bWaitStep=b_wait;
    }
    
    /* Deletes son-parent pairs */
    virtual void DeleteParentSonId (); //needed ?

protected:
    
    /* Calculates the vector to the task */
    virtual CVector2 VectorToTask(int n_Id);
    
    /* Calculates the vector to the closest light */
    virtual CVector2 VectorToLight();
    
    /* Calculates the vector to blob of specified color */
    virtual CVector2 VectorToBlob(CColor c_color);
    
    /* Calculates a normalized resulting vector */
    virtual CVector2 AdjustmentVector(std::vector<Real> f_range,std::vector<CRadians> c_angle); // pass by const ref !!
    
    /* Gets a direction vector as input and transforms it into wheel actuation. */
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

    /* Pointer to the omnidirectional camera sensor */
    CCI_ColoredBlobOmnidirectionalCameraSensor* m_pcCamera;
    
    /* Robot status variable */
    bool m_bRobotIdle;
    
    /* Flag to skip a step after having added*/ //make it a global variable ?
    bool m_bWaitStep;
    
    /* Variables for tree reconfiguration*/
    bool m_bRefreshDone;
    bool m_bStartTree;
    int m_nRefreshLayer;
    int m_nClock;
    
    TPairVector m_tParentSonId;
    STreeData m_sTreeData;
    
    void Emit(UInt8 un_id,bool b_idle, UInt8 un_level, UInt8 un_NextLevel);
    TCommReadings Receive(const std::string& str_RobotId, bool& b_CommON);
    
};

#endif
