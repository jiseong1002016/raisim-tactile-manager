cmake_minimum_required(VERSION 3.16)
project(raisim_tactile_manager VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(RAISIM_TACTILE_BUILD_EXAMPLES "Build examples" ON)
option(RAISIM_TACTILE_BUILD_TESTS "Build tests" ON)

find_package(Eigen3 REQUIRED)
find_package(raisim CONFIG REQUIRED)

add_library(raisim_tactile_manager
  src/ContactCollector.cpp
  src/HybridPenetrationTactile.cpp
  src/MeshSurface.cpp
  src/SensorGrid.cpp
)

target_include_directories(raisim_tactile_manager
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(raisim_tactile_manager
  PUBLIC
    Eigen3::Eigen
    raisim::raisim
)

target_compile_options(raisim_tactile_manager PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic>
)

add_library(raisim_tactile::manager ALIAS raisim_tactile_manager)

if(RAISIM_TACTILE_BUILD_EXAMPLES)
  add_executable(raisim_tactile_bolt_wrench_minimal
    examples/bolt_wrench_minimal/main.cpp
  )
  target_link_libraries(raisim_tactile_bolt_wrench_minimal
    PRIVATE raisim_tactile_manager
  )
endif()

if(RAISIM_TACTILE_BUILD_TESTS)
  enable_testing()
  add_executable(raisim_tactile_core_test tests/test_core.cpp)
  target_link_libraries(raisim_tactile_core_test
    PRIVATE raisim_tactile_manager
  )
  add_test(NAME raisim_tactile_core_test COMMAND raisim_tactile_core_test)
endif()

install(TARGETS raisim_tactile_manager
  EXPORT raisim_tactile_managerTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
