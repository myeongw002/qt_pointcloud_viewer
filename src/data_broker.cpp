/* data_broker.cpp ------------------------------------------------------ */
#include "data_broker.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <QDebug>

static QString robotToPcdTopic(const QString& r) {
    if (r == "TUGV") return "/tugv/viz_global_cloud";
    if (r == "MUGV") return "/mugv/viz_global_cloud";
    if (r == "SUGV1") return "/sugv1/viz_global_cloud";
    if (r == "SUGV2") return "/sugv2/viz_global_cloud";
    if (r == "SUAV") return "/suav/viz_global_cloud";
    return "/";
}

static QString robotToPathTopic(const QString& r) {
    if (r == "TUGV") return "/tugv/viz_path";
    if (r == "MUGV") return "/mugv/viz_path";
    if (r == "SUGV1") return "/sugv1/viz_path";
    if (r == "SUGV2") return "/sugv2/viz_path";
    if (r == "SUAV") return "/suav/viz_path";
    return "/";
}

DataBroker::DataBroker(const QStringList& robots, QObject *parent)
    : QObject(parent), Node("qt_pointcloud_viewer", rclcpp::NodeOptions().use_intra_process_comms(true))
{
    for (const QString& r : robots) {
        createPcdSub(r, robotToPcdTopic(r).toStdString());
        createPathSub(r, robotToPathTopic(r).toStdString());
    }
    qDebug() << "DataBroker initialized with robots:" << robots;
}

void DataBroker::createPcdSub(const QString& robot, const std::string& topic) {
    auto qos = rclcpp::SensorDataQoS();
    pcdSubs_[topic] = create_subscription<sensor_msgs::msg::PointCloud2>(
        topic, qos,
        [this, robot](const sensor_msgs::msg::PointCloud2::SharedPtr msg){
            auto tmp = boost::make_shared<Types::Cloud>();  // Types::Cloud 사용
            pcl::fromROSMsg(*msg, *tmp);
            CloudConstPtr cloud = tmp;
            
            emit cloudArrived(robot, cloud);
            emit gridMapUpdateRequested(robot, cloud);
        });
}

void DataBroker::createPathSub(const QString& robot, const std::string& topic) {
    auto qos = rclcpp::SensorDataQoS();
    pathSubs_[topic] = create_subscription<nav_msgs::msg::Path>(
        topic, qos,
        [this, robot](const nav_msgs::msg::Path::SharedPtr msg){
            // PathConstPtr 타입으로 변환
            auto pathPtr = std::make_shared<const Types::Path>(msg->poses);
            emit pathArrived(robot, pathPtr);  // PathConstPtr 시그널 발생
        });
}