/* data_broker.hpp ------------------------------------------------------ */
#pragma once
#include <QObject>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <QMap>

using CloudConstPtr = pcl::PointCloud<pcl::PointXYZI>::ConstPtr;

class DataBroker final : public QObject, public rclcpp::Node
{
    Q_OBJECT
public:
    explicit DataBroker(const QStringList& robots, QObject *parent = nullptr);

signals:
    void cloudArrived(const QString& robot, CloudConstPtr cloud);
    void pathArrived(const QString& robot, const std::vector<geometry_msgs::msg::PoseStamped>& path);

private:
    void createPcdSub(const QString& robot, const std::string& topic);
    void createPathSub(const QString& robot, const std::string& topic);

    std::unordered_map<std::string,
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> pcdSubs_;
    std::unordered_map<std::string,
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr> pathSubs_;
};
