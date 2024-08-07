cmake_minimum_required(VERSION 3.16)
project(LQR)

find_package(Eigen3 REQUIRED COMPONENTS)
include_directories(${EIGEN3_INCLUDE_DIR})

set(CMAKE_CXX_STANDARD 11)


############################################################################
# matplotlib-cpp
include(GNUInstallDirs)
set(PACKAGE_NAME matplotlib_cpp)
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/${PACKAGE_NAME}/cmake)

# Library target
add_library(matplotlib_cpp INTERFACE)
target_include_directories(matplotlib_cpp
        INTERFACE
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
        )
target_compile_features(matplotlib_cpp INTERFACE
        cxx_std_20
        )
# TODO: Use `Development.Embed` component when requiring cmake >= 3.18
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
target_link_libraries(matplotlib_cpp INTERFACE
        Python3::Python
        Python3::Module
        )
find_package(Python3 COMPONENTS NumPy)
if (Python3_NumPy_FOUND)
    target_link_libraries(matplotlib_cpp INTERFACE
            Python3::NumPy
            )
else ()
    target_compile_definitions(matplotlib_cpp INTERFACE WITHOUT_NUMPY)
endif ()
install(
        TARGETS matplotlib_cpp
        EXPORT install_targets
)
############################################################################

include_directories(include)

add_executable(lqr_controller src/LQR.cpp
                              src/KinematicModel.cpp
                              src/main.cpp
                              src/Reference_path.cpp)

                              
target_link_libraries(lqr_controller PUBLIC m PRIVATE --coverage PUBLIC matplotlib_cpp)

add_executable(lqr_pid_controller src/LQR.cpp
                              src/KinematicModel.cpp
                              src/main_pid.cpp
                              src/Reference_path.cpp
                              src/pid_control.cpp)
target_link_libraries(lqr_pid_controller PUBLIC m PRIVATE --coverage PUBLIC matplotlib_cpp)
