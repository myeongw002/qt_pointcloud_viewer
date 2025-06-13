cmake_minimum_required(VERSION 3.10)
project(qt_pointcloud_viewer)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)
set(OpenGL_GL_PREFERENCE GLVND)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui OpenGL)
find_package(OpenGL REQUIRED)

# UI 파일 처리
set(UI_FILES ui/mainwindow.ui)
qt5_wrap_ui(UI_HEADERS ${UI_FILES})

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/mainwindow.cpp
    src/pointcloud_widget.cpp
    src/viewer_panel.cpp
    src/robot_select_dialog.cpp
    src/viewer_container.cpp
    src/viewer_window.cpp
    src/data_broker.cpp
    src/shape_helper.cpp
    src/control_tree_widget.cpp
    ${UI_HEADERS}  # 생성된 UI 헤더 추가
)

# MOC 파일 자동 생성
set(MOC_HEADERS include/${PROJECT_NAME}/mainwindow.hpp 
    include/${PROJECT_NAME}/pointcloud_widget.hpp 
    include/${PROJECT_NAME}/viewer_panel.hpp
    include/${PROJECT_NAME}/robot_select_dialog.hpp
    include/${PROJECT_NAME}/viewer_container.hpp
    include/${PROJECT_NAME}/viewer_window.hpp
    include/${PROJECT_NAME}/data_broker.hpp
    include/${PROJECT_NAME}/control_tree_widget.hpp)
qt5_wrap_cpp(MOC_SRCS ${MOC_HEADERS})

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} 
    Qt5::Widgets 
    Qt5::Gui 
    Qt5::Core  # Core 추가
    Qt5::OpenGL 
    OpenGL::GL
    ${PCL_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ${OpenGL_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME} 
    rclcpp 
    sensor_msgs 
    nav_msgs
    pcl_conversions 
    pcl_ros
)

target_sources(${PROJECT_NAME} PRIVATE 
    ${MOC_SRCS} 
    ${UI_HEADERS}  # UI 헤더 추가
)

install(TARGETS ${PROJECT_NAME} 
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()