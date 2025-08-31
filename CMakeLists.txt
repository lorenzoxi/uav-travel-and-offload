cmake_minimum_required(VERSION 3.27)
project(2025-ccnc-drone)

set(CMAKE_CXX_STANDARD 17)

set(CPLEX_ROOT_DIR "/Applications/CPLEX_Studio2211")
set(CPLEX_INCLUDE_DIR "${CPLEX_ROOT_DIR}/cplex/include")
set(CONCERT_INCLUDE_DIR "${CPLEX_ROOT_DIR}/concert/include")
set(CPLEX_LIB_DIR "${CPLEX_ROOT_DIR}/cplex/lib/arm64_osx/static_pic")
set(CONCERT_LIB_DIR "${CPLEX_ROOT_DIR}/concert/lib/arm64_osx/static_pic")
set(Boost_INCLUDE_DIR "/usr/local/Cellar/boost/1.82.0_1/include")
set(Boost_LIBRARY_DIR "/usr/local/Cellar/boost/1.82.0_1/lib")
set(GUROBI_HOME "/Library/gurobi1102/macos_universal2")
set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)

include_directories(${GUROBI_HOME}/include)
include_directories(${CPLEX_INCLUDE_DIR})
include_directories(${CONCERT_INCLUDE_DIR})
link_directories(${CPLEX_LIB_DIR})
link_directories(${CONCERT_LIB_DIR})
link_directories(${GUROBI_HOME}/lib)

find_package(CGAL REQUIRED)


add_executable(2025-ccnc-drone
        include/json.hpp
        main.cpp
)
target_link_libraries(2025-ccnc-drone ilocplex concert cplex m pthread gurobi_c++ gurobi110 CGAL::CGAL)
