# Note that headers are optional, and do not affect add_library, but they will not
# show up in IDEs unless they are listed in add_library.

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-march=native -mtune=native -O3 ")
set(CMAKE_C_FLAGS "-march=native -mtune=native -O3 ")


set(INCLUDE_DIRS 
	${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/include/state_spaces
        ${PROJECT_SOURCE_DIR}/include/state_spaces/real_vector_space
        ${PROJECT_SOURCE_DIR}/include/planners
        ${PROJECT_SOURCE_DIR}/include/planners/rrt
        ${PROJECT_SOURCE_DIR}/include/planners/rbt
        ${PROJECT_SOURCE_DIR}/include/planners/drbt
        ${PROJECT_SOURCE_DIR}/include/planners/rbt_star
        ${PROJECT_SOURCE_DIR}/include/planners/trajectory
        ${PROJECT_SOURCE_DIR}/include/robots
        ${PROJECT_SOURCE_DIR}/include/environments
        ${PROJECT_SOURCE_DIR}/include/environments/objects
        ${PROJECT_SOURCE_DIR}/include/scenario
        ${PROJECT_SOURCE_DIR}/include/configurations
        ${PROJECT_SOURCE_DIR}/include/benchmark
        ${PROJECT_SOURCE_DIR}/../STRRT_Planner/include
        ${PROJECT_SOURCE_DIR}/../STRRT_Planner/include/CollisionManager
        ${PROJECT_SOURCE_DIR}/../STRRT_Planner/include/config_read_writer
)

# Optionally glob, but only for CMake 3.12 or later:
file(GLOB_RECURSE HEADER_LIST CONFIGURE_DEPENDS "${PROJECT_SOURCE_DIR}/include/*.h")

file(GLOB_RECURSE SRC_LIST CONFIGURE_DEPENDS "*.cpp")


# message(${SRC_LIST})
# Make an automatic library - will be static or dynamic based on user setting
add_library(rpmpl_library SHARED ${SRC_LIST} ${HEADER_LIST})

# We need this directory, and users of our library will need it too
target_include_directories(rpmpl_library PUBLIC ${INCLUDE_DIRS})

# All users of this library will need at least C++17
target_compile_features(rpmpl_library PUBLIC cxx_std_17)

add_executable(main main.cpp)
target_compile_features(main PRIVATE cxx_std_17)
target_link_libraries(main PUBLIC config_read_writer CollisionManager rpmpl_library ${PROJECT_LIBRARIES})
target_include_directories(main PUBLIC ${PROJECT_SOURCE_DIR}/apps)
# IDEs should put the headers in a nice place
source_group(
  TREE "${PROJECT_SOURCE_DIR}/include"
  PREFIX "Header Files"
  FILES ${HEADER_LIST})