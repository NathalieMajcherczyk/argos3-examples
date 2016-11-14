#ifndef CONNECTION_LOOP_FUNCTIONS_H
#define CONNECTION_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <controllers/footbot_connectedmotion/footbot_connectedmotion.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
//#include <argos3/plugins/robots/generic/control_interface/ci_proximity_sensor.h>

using namespace argos;

class CConnectionLoopFunctions : public CLoopFunctions {

public:
    
    CConnectionLoopFunctions();
    virtual ~CConnectionLoopFunctions() {}
    
    CVector2 m_Pos;
    
    virtual void Init(TConfigurationNode& t_tree);
    virtual void Reset();
    virtual void Destroy();
    virtual void PreStep();
    virtual void PostStep();
    
private:
    CFootBotEntity* m_pcNode;
    int m_nNodeCounter;
    int m_nNumberSpares;
    typedef std::vector< std::pair <std::string, int> > TVecPair;
    TVecPair m_tNewLevels;
    
};

#endif
