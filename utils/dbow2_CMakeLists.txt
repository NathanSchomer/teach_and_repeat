cmake_minimum_required(VERSION 3.5)
project(dbow2)

# ROS 2 build system
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Source and header files
set(HDRS
  include/DBoW2/BowVector.h           include/DBoW2/FBrief.h
  include/DBoW2/QueryResults.h        include/DBoW2/TemplatedDatabase.h   include/DBoW2/FORB.h
  include/DBoW2/DBoW2.h               include/DBoW2/FClass.h              include/DBoW2/FeatureVector.h
  include/DBoW2/ScoringObject.h       include/DBoW2/TemplatedVocabulary.h
)

set(SRCS 
  src/BowVector.cpp     src/FBrief.cpp        src/FORB.cpp
  src/FeatureVector.cpp src/QueryResults.cpp  src/ScoringObject.cpp
)

# DBoW2 library target
add_library(${PROJECT_NAME} SHARED ${SRCS} ${HDRS})
target_include_directories(${PROJECT_NAME} PUBLIC
  include/DBoW2
  include
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 14)

# Export
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(OpenCV)

# Install
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/DBoW2
  DESTINATION include
)

ament_package()
