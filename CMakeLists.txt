
cmake_minimum_required(VERSION 3.5)


set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/examples/common_components/protocol_examples_common)
#这一段是udp使用的没有这一段#include "protocol_examples_common.h"就无法声明


include($ENV{IDF_PATH}/tools/cmake/project.cmake)
get_filename_component(ProjectId ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" ProjectId ${ProjectId})
project(${ProjectId})
