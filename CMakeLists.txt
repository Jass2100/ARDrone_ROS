cmake_minimum_required(VERSION 2.8.3)

set(PROJECT drone)
project(${PROJECT})

set(SOURCES
        drone.cpp
        main.cpp
        mainwindow.cpp
        ControllerNode.cpp
)

set(MOC_HEADERS
        drone.h
        mainwindow.h
        ControllerNode.h
)

set(UIS
        mainwindow.ui
)

find_package(catkin REQUIRED COMPONENTS roscpp)

find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})
include_directories(${CMAKE_BINARY_DIR})

QT4_WRAP_UI(UI_HEADERS ${UIS})
QT4_WRAP_CPP(MOC_SRCS ${MOC_HEADERS})

add_executable(${PROJECT} ${SOURCES} ${MOC_SRCS} ${UI_HEADERS})

target_link_libraries(${PROJECT} ${QT_LIBRARIES} ${catkin_LIBRARIES})
