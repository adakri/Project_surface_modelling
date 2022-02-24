cmake_minimum_required(VERSION 3.0.2)

project(Surf_Modelling)

# options de compilation
set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")


# chemin vers les header files
include_directories(include)


# Ajouter les fichiers sources
set(SOURCES src/main.cpp src/Polygon.cpp src/Mesh.cpp src/Edge.cpp src/Triangle.cpp src/Vec2.cpp 
                src/delaunator_ext.cpp
                src/Viewer.cpp src/Deformation.cpp 
                 )

add_executable(exec ${SOURCES})

# Detect and add SFML et al.
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules" ${CMAKE_MODULE_PATH})

set(EIGEN3_INCLUDE_DIR external_libs/eigen-3.4.0 )

#Eigen is header only, should only be included


target_sources(exec PRIVATE ${SOURCES})

target_include_directories(exec PRIVATE
  external_libs/SFML-2.5.1/include 
  external_libs/TGUI-0.9.2/include
  ${EIGEN3_INCLUDE_DIR}
  include/)

target_link_directories(exec PRIVATE external_libs/SFML-2.5.1/lib external_libs/TGUI-0.9.2/lib )
  
target_link_libraries(exec PRIVATE
  sfml-graphics
  sfml-window
  sfml-system
  tgui
  )