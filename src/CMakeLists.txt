
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library librtt_ati_sensor-<target>.so
# and installs in the directory lib/orocos/rtt_ati_sensor/
#
orocos_component(rtt_ati_sensor rtt_ati_sensor-component.hpp rtt_ati_sensor-component.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/rtt_ati_sensor/ directory
#
orocos_install_headers(rtt_ati_sensor-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library librtt_ati_sensor-plugin-<target>.so
# and installs in the directory lib/orocos/rtt_ati_sensor/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
orocos_plugin(rtt_ati_sensor-plugin rtt_ati_sensor-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library librtt_ati_sensor-service-<target>.so
# and installs in the directory lib/orocos/rtt_ati_sensor/plugins/
#
orocos_service(rtt_ati_sensor-service rtt_ati_sensor-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a normal library (optional):
#
# Creates a library librtt_ati_sensor-support-<target>.so and installs it in
# lib/
#
orocos_library(rtt_ati_sensor-support support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.


