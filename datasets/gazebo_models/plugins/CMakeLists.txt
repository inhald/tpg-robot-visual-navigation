cmake_minimum_required(VERSION 3.10)
project(randomized_spawner_plugin)

set(CMAKE_CXX_STANDARD 17)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(ignition-math6 REQUIRED)
find_package(ignition-physics5 REQUIRED)
find_package(ignition-rendering6 REQUIRED)
find_package(sdformat13 REQUIRED)


add_library(RandomizedSpawner SHARED RandomizedSpawner.cc)

target_include_directories(RandomizedSpawner
  PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}"  
)

target_link_libraries(RandomizedSpawner
  ignition-gazebo6::ignition-gazebo6
  ignition-plugin1::ignition-plugin1
  ignition-math6::ignition-math6
  ignition-physics5::ignition-physics5
  ignition-rendering6::ignition-rendering6
  sdformat13::sdformat13
  )



#---Path Length Plugin ----
add_library(PathLengthPlugin SHARED PathLengthPlugin.cc)
target_include_directories(PathLengthPlugin PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(PathLengthPlugin
  ignition-gazebo6::ignition-gazebo6
  ignition-plugin1::ignition-plugin1
  ignition-math6::ignition-math6
  ignition-physics5::ignition-physics5
  ignition-rendering6::ignition-rendering6
  sdformat13::sdformat13
)

target_compile_features(PathLengthPlugin PUBLIC cxx_std_17)

# Install into Ignition’s default plugin path
install(TARGETS RandomizedSpawner PathLengthPlugin
        LIBRARY DESTINATION lib/ignition/gazebo6/plugins)

