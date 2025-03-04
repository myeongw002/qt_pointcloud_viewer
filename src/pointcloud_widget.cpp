#include "pointcloud_widget.h"
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>

PointCloudWidget::PointCloudWidget(QWidget *parent, rclcpp::Node::SharedPtr ros_node)
    : QOpenGLWidget(parent), cloud(new pcl::PointCloud<pcl::PointXYZ>), node(ros_node) {
    
    // ✅ Subscribe to the PointCloud2 topic with QoS
    rclcpp::QoS qos_settings(rclcpp::KeepLast(10));
    qos_settings.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_settings.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/tugv/os64_pts", qos_settings,
        std::bind(&PointCloudWidget::pointCloudCallback, this, std::placeholders::_1));
    
    std::cout << "Subscribed to /tugv/os64_pts" << std::endl;
}

void PointCloudWidget::initializeGL() {
    glEnable(GL_DEPTH_TEST);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float aspect = width() / static_cast<float>(height());
    glFrustum(-aspect, aspect, -1.0, 1.0, 1.0, 100.0);  // ✅ Set perspective

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void PointCloudWidget::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    glTranslatef(0.0f, 0.0f, -10.0f);  // ✅ Move the view backward

    glBegin(GL_POINTS);
    glColor3f(0.0f, 1.0f, 0.0f);

    for (const auto &point : cloud->points) {
        glVertex3f(point.x, point.y, point.z);
    }

    glEnd();
}

void PointCloudWidget::resizeGL(int w, int h) {
    if (h == 0) h = 1;  // Prevent division by zero
    float aspect = static_cast<float>(w) / static_cast<float>(h);
    
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glFrustum(-aspect, aspect, -1.0, 1.0, 1.0, 100.0);  // ✅ Update projection
    
    glMatrixMode(GL_MODELVIEW);
}

void PointCloudWidget::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::fromROSMsg(*msg, *cloud);
    
    if (cloud->points.empty()) {
        std::cout << "Received an empty point cloud!" << std::endl;
    } else {
        std::cout << "Received point cloud with " << cloud->points.size() << " points." << std::endl;
    }

    update();  // Refresh OpenGL rendering
}
