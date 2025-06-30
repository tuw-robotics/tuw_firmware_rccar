cmake_minimum_required(VERSION 3.16)

option(USE_TRACING "Enable tracing defaults" OFF)

if(USE_TRACING)
  set(SDKCONFIG_DEFAULTS "sdkconfig.defaults;sdkconfig.mros;sdkconfig.tracing")
else()
  set(SDKCONFIG_DEFAULTS "sdkconfig.defaults;sdkconfig.mros")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(firmware)
