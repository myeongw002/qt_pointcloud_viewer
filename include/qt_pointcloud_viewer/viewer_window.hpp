/* viewer_window.hpp */
#pragma once
#include <QOpenGLWindow>
#include <QOpenGLFunctions>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>



namespace Widget {

class ViewerWindow final : public QOpenGLWindow, protected QOpenGLFunctions
{
    Q_OBJECT
public:
    explicit ViewerWindow(const QString &robot,
                          rclcpp::Node::SharedPtr node,
                          QOpenGLContext *share = nullptr);

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    QString toTopic(const QString &robot) const;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    std::mutex mtx_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_{new pcl::PointCloud<pcl::PointXYZI>};

    glm::mat4 proj_;
};

} // namespace Widget