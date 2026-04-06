# Get the MUJOCO path from the environment
# if(DEFINED ENV{MUJOCO})
#     set(MUJOCOPATH $ENV{MUJOCO})
# else()
#     message(FATAL_ERROR "MUJOCO environment variable is not set!")
# endif()

# Locate the MuJoco library file in ${MUJOCOPATH}/lib
find_library(MUJOCOLIBRARY mujoco HINTS "${MUJOCOPATH}/lib")
if(NOT MUJOCOLIBRARY)
  message(FATAL_ERROR "Could not find the MuJoco library in ${MUJOCOPATH}/lib")
endif()

# Source files for experiments
file(GLOB EXPERIMENTS_SOURCES
    *.cc
    *.h
    ../environments/*.cc
    ../environments/*.h

    ../environments/gazebo/GazeboEnv.h
    ,,/environments/gazebo/Turlebot4_Nav.h


    ../environments/mujoco/MuJocoEnv.h

    runners/*.cc
    runners/*.h
    evaluators/EvalData.cc
    evaluators/EvalData.h
)

# Create the TPGExperimentMPI executable
add_executable(TPGExperimentMPI ${EXPERIMENTS_SOURCES})

# Include directories
target_include_directories(TPGExperimentMPI
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/runners
    ${CMAKE_CURRENT_SOURCE_DIR}/evaluators
    ${CMAKE_CURRENT_SOURCE_DIR}/../engine
    ${CMAKE_CURRENT_SOURCE_DIR}/../environments
    ${CMAKE_CURRENT_SOURCE_DIR}/../environments/classic_control
    ${CMAKE_CURRENT_SOURCE_DIR}/../environments/mujoco
    ${CMAKE_CURRENT_SOURCE_DIR}/../environments/sequence

    ${CMAKE_CURRENT_SOURCE_DIR}/../environments/gazebo
    

    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${CURL_INCLUDE_DIRS}
    ${MUJOCOPATH}/include
)

# Link libraries
target_link_libraries(TPGExperimentMPI
    PRIVATE
    engine
    MPI::MPI_CXX
    Boost::thread
    Boost::system
    Boost::mpi
    Boost::serialization
    Boost::iostreams
    CURL::libcurl
    ${CMAKE_THREAD_LIBS_INIT}
    glfw
    GLEW::GLEW
    OpenGL::GL
    ${MUJOCOLIBRARY}
    TBB::tbb
    glut
    GLU
    OSMesa

    ignition-common4::core
    ignition-msgs8
    ignition-transport11
    ignition-gazebo6::core
    sdformat13::sdformat13

    ${OpenCV_LIBS}

)

# Set output directory for the executable to match the expected SCons path
set_target_properties(TPGExperimentMPI PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/release/experiments") 
