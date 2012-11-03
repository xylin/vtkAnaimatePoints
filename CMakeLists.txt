cmake_minimum_required(VERSION 2.6)
PROJECT(DataAnimation)
 
FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})
 
ADD_EXECUTABLE(DataAnimation DataAnimation.cxx)
TARGET_LINK_LIBRARIES(DataAnimation vtkHybrid)