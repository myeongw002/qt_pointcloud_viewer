/* data_broker.hpp ------------------------------------------------------ */
#pragma once
#include <QObject>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <QMap>
#include "common_types.hpp"

// Types 네임스페이스 사용
using CloudConstPtr = Types::CloudConstPtr;
using PathConstPtr = Types::PathConstPtr;
class DataBroker final : public QObject, public rclcpp::Node
{
    Q_OBJECT
public:
    explicit DataBroker(const QStringList& robots, QObject *parent = nullptr);

signals:
    void cloudArrived(const QString& robot, CloudConstPtr cloud);
    void pathArrived(const QString& robot, PathConstPtr path);  // PathConstPtr 타입으로 변경
    void gridMapUpdateRequested(const QString& robot, CloudConstPtr cloud);

private:
    void createPcdSub(const QString& robot, const std::string& topic);
    void createPathSub(const QString& robot, const std::string& topic);

    std::unordered_map<std::string,
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> pcdSubs_;
    std::unordered_map<std::string,
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr> pathSubs_;
};
