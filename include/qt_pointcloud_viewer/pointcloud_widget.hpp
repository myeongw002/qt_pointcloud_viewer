#ifndef POINTCLOUD_WIDGET_H
#define POINTCLOUD_WIDGET_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <mutex>

namespace Widget {
    class PointCloudWidget : public QOpenGLWidget, protected QOpenGLFunctions {
        Q_OBJECT
    
    public:
        explicit PointCloudWidget(QWidget *parent = nullptr);
        ~PointCloudWidget();
        void setNode(rclcpp::Node::SharedPtr ros_node = nullptr);
        void setTopicName(int index);
        std::string getTopicName();
        void setShowAxes(bool show);
        void setShowGrid(bool show);
        void setRotationSensitivity(float sensitivity);
        void setFocusPoint(const glm::vec3& focus);

    protected:
        void initializeGL() override;
        void paintGL() override;
        void resizeGL(int w, int h) override;
        void mousePressEvent(QMouseEvent *event) override;
        void mouseReleaseEvent(QMouseEvent *event) override;
        void mouseMoveEvent(QMouseEvent *event) override;
        void wheelEvent(QWheelEvent *event) override;
        void hideIndicator();

    private:
        // ROS2 and PCL
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;
        std::mutex cloudMutex_;
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
        void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        // Camera and Orbit View
        glm::vec3 focusPoint_ = glm::vec3(0.0f, 0.0f, 0.0f);
        float distance_ = 10.0f;
        float yaw_ = 0.0f;
        float pitch_ = 0.0f;
        float rotationSensitivity_ = 0.5f;
        glm::vec3 cameraPos_;
        QPoint lastMousePos_;
        void updateCameraPosition();

        // Rendering
        glm::mat4 viewMatrix_;
        glm::mat4 projectionMatrix_;
        void drawPoints();
        void drawAxes();
        void drawGrid();
        void drawCameraIndicator();

        // Indicator and UI
        bool showIndicator_ = false;
        QTimer hideTimer_;
        const int timerInterval_ = 100;
        std::string topicName_ = "";
        bool showAxes_ = false;
        bool showGrid_ = false;
    };
}

#endif // POINTCLOUD_WIDGET_H