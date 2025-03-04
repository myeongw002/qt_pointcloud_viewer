cmake_minimum_required(VERSION 3.8)
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
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui OpenGL)
find_package(OpenGL REQUIRED)

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/mainwindow.cpp
    src/pointcloud_widget.cpp
    ui/mainwindow.ui
)

# MOC 파일 자동 생성
set(MOC_HEADERS include/${PROJECT_NAME}/mainwindow.h include/${PROJECT_NAME}/pointcloud_widget.h)
qt5_wrap_cpp(MOC_SRCS ${MOC_HEADERS})

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
    $<INSTALL_INTERFACE:include>
)


target_link_libraries(${PROJECT_NAME} 
    Qt5::Widgets 
    Qt5::OpenGL 
    OpenGL::GL
    ${PCL_LIBRARIES}  # PCL 라이브러리 명시
    ${rclcpp_LIBRARIES}
    ${OpenGL_LIBRARIES}
    ${OPENGL_glx_LIBRARY}
)

ament_target_dependencies(${PROJECT_NAME} 
    rclcpp 
    sensor_msgs 
    pcl_conversions 
    pcl_ros
    Qt5Core  # 추가
    Qt5Widgets
)

target_sources(${PROJECT_NAME} PRIVATE ${MOC_SRCS})

install(TARGETS ${PROJECT_NAME} 
        DESTINATION lib/${PROJECT_NAME}
)


ament_package()
