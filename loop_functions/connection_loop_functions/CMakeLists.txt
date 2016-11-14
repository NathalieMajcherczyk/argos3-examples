add_library(connection_loop_functions MODULE 
	connection_loop_functions.h
	connection_loop_functions.cpp)

target_link_libraries(connection_loop_functions
  footbot_connectedmotion
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot)

