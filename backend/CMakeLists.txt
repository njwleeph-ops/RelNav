cmake_minimum_required(VERSION 3.16)
project(relnavsimbackend CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(GTest REQUIRED)


include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/include/cpp-httplib
    ${CMAKE_SOURCE_DIR}/include/json/include
    ${GTEST_INCLUDE_DIRS}
)

add_executable(server
    src/main.cpp
    src/cw_dynamics.cpp
    src/monte_carlo.cpp
    src/gnc_algorithms.cpp
)

add_executable(testing 
    src/tests/test_glideslope_updates.cpp
    src/gnc_algorithms.cpp
    src/cw_dynamics.cpp
)

target_include_directories(server PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(server PRIVATE
    Threads::Threads
    Eigen3::Eigen
)

target_include_directories(testing PRIVATE 
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(testing PRIVATE 
    GTest::gtest
    GTest::gtest_main
    Eigen3::Eigen
)
