cmake_minimum_required(VERSION 3.31)

project(MissionPlanner VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

if(MSVC)
    add_compile_options(/wd4819)
    add_compile_options(/utf-8)
endif()


set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/c_library_v2 
                    ${CMAKE_CURRENT_SOURCE_DIR}/c_library_v2/all 
                    ${CMAKE_CURRENT_SOURCE_DIR}/c_library_v2/all/common)


include(Asio)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Qml Quick QuickWidgets Location Positioning Network)

set(SOURCES src/main.cpp src/mavlink_udp.cpp)
add_executable(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME}
                        Qt5::Core
                        Qt5::Widgets
                        Qt5::Qml
                        Qt5::Quick
                        Qt5::QuickWidgets
                        Qt5::Location
                        Qt5::Positioning
                        Qt5::Network
                        asio
)
target_include_directories(${PROJECT_NAME} PRIVATE src)


