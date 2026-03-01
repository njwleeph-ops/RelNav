cmake_minimum_required(VERSION 3.16)
project(relnavsimbackend CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(GTest REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(relnav_core
    src/cw_dynamics.cpp
    src/gnc_algorithms.cpp
    src/monte_carlo.cpp
)

target_include_directories(relnav_core PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/include/cpp-httplib
    ${CMAKE_SOURCE_DIR}/include/json/include
)

target_link_libraries(relnav_core PUBLIC
    Eigen3::Eigen
    Threads::Threads
)


add_executable(server
    src/main.cpp
    src/config.cpp
)

target_link_libraries(server PRIVATE
    relnav_core
    yaml-cpp::yaml-cpp
)

function(add_test_executable name source) 
    add_executable(${name} ${source} ${ARGN})
    target_link_libraries(${name} PRIVATE
        relnav_core
        GTest::gtest
        GTest::gtest_main
    )
endfunction()

add_test_executable(guidance_testing
    src/tests/unit_tests/guidance_tests.cpp
)

add_test_executable(monte_carlo_testing
    src/tests/unit_tests/monte_carlo_tests.cpp
)

add_test_executable(nav_testing
    src/tests/unit_tests/navigation_tests.cpp
)

add_test_executable(controller_validation_test
    src/tests/validation/controller_validation.cpp
    src/tests/unit_tests/controller_validation_tests.cpp
)

add_executable(controller_validation
    src/tests/validation/controller_validation.cpp
    src/tests/validation/controller_validation_main.cpp
)

target_link_libraries(controller_validation PRIVATE
    relnav_core
)

target_include_directories(controller_validation PRIVATE
    src/tests/validation
)
