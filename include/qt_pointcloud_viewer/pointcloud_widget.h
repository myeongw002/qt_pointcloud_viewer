#ifndef POINTCLOUD_WIDGET_H
#define POINTCLOUD_WIDGET_H

#include <QOpenGLWidget>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PointCloudWidget : public QOpenGLWidget {
    Q_OBJECT

public:
    explicit PointCloudWidget(QWidget *parent = nullptr, rclcpp::Node::SharedPtr ros_node = nullptr);

protected:
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int w, int h) override;

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    rclcpp::Node::SharedPtr node;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription;

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
};

#endif // POINTCLOUD_WIDGET_H
