cmake_minimum_required (VERSION 3.8)

add_library (rosaservernavgraph SHARED
	rosaservernavgraph.cpp
)

set_property (TARGET rosaservernavgraph PROPERTY CXX_STANDARD 17)
set_property (TARGET rosaservernavgraph PROPERTY C_VISIBILITY_PRESET hidden)
set_property (TARGET rosaservernavgraph PROPERTY CXX_VISIBILITY_PRESET hidden)
set_property (TARGET rosaservernavgraph PROPERTY VISIBILITY_INLINES_HIDDEN ON)

target_link_libraries (rosaservernavgraph ${CMAKE_SOURCE_DIR}/../RosaServerSAM/moonjit/src/libluajit.so)
include_directories (${CMAKE_SOURCE_DIR}/../RosaServerSAM/moonjit/src)
include_directories (${CMAKE_SOURCE_DIR}/../RosaServerSAM/sol2/include)