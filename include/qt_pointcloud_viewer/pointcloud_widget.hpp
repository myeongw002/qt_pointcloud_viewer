#ifndef POINTCLOUD_WIDGET_H
#define POINTCLOUD_WIDGET_H

#include <QOpenGLWidget>
#include <QMouseEvent>  // ✅ Include for mouse input
#include <QWheelEvent>  // ✅ Include for wheel input
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PointCloudWidget : public QOpenGLWidget {
    Q_OBJECT

public:
    explicit PointCloudWidget(QWidget *parent = nullptr);
    void setNode(rclcpp::Node::SharedPtr ros_node = nullptr);
    void setStartFlag(bool flag); 
    void setTopicName(int index);
    std::string getTopicName() ;

protected:
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int w, int h) override;
    
    // ✅ Mouse interaction functions
    void mousePressEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;
    void hideIndicator();
    
    
private:
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // ✅ Variables for rotation, zoom, and panning
    float rotationX_ = 0.0f;
    float rotationY_ = 0.0f;
    float zoom_ = -10.0f;
    float panX_ = 0.0f;  // ✅ New: Panning left/right
    float panY_ = 0.0f;  // ✅ New: Panning up/down
    QPoint lastMousePos_;
    QTimer hideTimer_;  // ✅ Timer to hide the indicator
    bool showIndicator_;  // ✅ Flag to control indicator visibility
    const int timerInterval_ = 100;  // ✅ Interval for hiding the indicator
    bool startFlag_ = true; // ✅ Flag to control the start of the point cloud display
    std::string topicName_ = ""; // ✅ Topic name for point cloud data
    // rclcpp::Subscription subscription;
};

#endif // POINTCLOUD_WIDGET_H
