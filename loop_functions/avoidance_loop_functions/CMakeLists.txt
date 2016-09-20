add_library(avoidance_loop_functions MODULE 
	avoidance_loop_functions.h
	avoidance_loop_functions.cpp)

target_link_libraries(avoidance_loop_functions
  footbot_avoidance
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot)

