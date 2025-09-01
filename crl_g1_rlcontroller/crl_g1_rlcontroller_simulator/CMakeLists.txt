cmake_minimum_required(VERSION 3.11)

# set name of the project
project(crl_g1_rlcontroller_simulator)

# set cxx standard and compiler flag
set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------------------------------------------------------
# ros dependencies
# -----------------------------------------------------------------------------

find_package(crl_humanoid_simulator 0.8.0 EXACT REQUIRED)
find_package(crl_g1_rlcontroller_commons 0.1.0 REQUIRED)

# -----------------------------------------------------------------------------
# resource files
# -----------------------------------------------------------------------------

# launch
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# config
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)

# -----------------------------------------------------------------------------
# code
# -----------------------------------------------------------------------------

# executable
add_executable(sim src/main.cpp)

ament_target_dependencies(sim crl_humanoid_simulator crl_g1_rlcontroller_commons)

install(
        TARGETS sim
        DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------------------------------------------------------
ament_export_dependencies(crl_humanoid_simulator crl_g1_rlcontroller_commons)
ament_package()
