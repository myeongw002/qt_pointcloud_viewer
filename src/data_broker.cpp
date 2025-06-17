/* data_broker.cpp ------------------------------------------------------ */
#include "data_broker.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <QDebug>

static QString robotToPcdTopic(const QString& r)
{
    const QMap<QString,QString> map{
        {"TUGV","/tugv/viz_global_cloud"},
        {"MUGV","/mugv/viz_global_cloud"},
        {"SUGV1","/sugv1/viz_global_cloud"},
        {"SUGV2","/sugv2/viz_global_cloud"},
        {"SUAV"  ,"/suav/viz_global_cloud"} };
    return map.value(r,"/");
}

static QString robotToPathTopic(const QString& r)
{
    const QMap<QString,QString> map{
        {"TUGV","/tugv/viz_path"},
        {"MUGV","/mugv/viz_path"},
        {"SUGV1","/sugv1/viz_path"},
        {"SUGV2","/sugv2/viz_path"},
        {"SUAV"  ,"/suav/viz_path"} };
    return map.value(r,"/");
}

DataBroker::DataBroker(const QStringList& robots, QObject *parent)
    : QObject(parent), Node("qt_pointcloud_viewer", rclcpp::NodeOptions().use_intra_process_comms(true))
{
    for (const QString& r : robots) {  // Added variable declaration
        createPcdSub(r, robotToPcdTopic(r).toStdString());
        createPathSub(r, robotToPathTopic(r).toStdString());
    }
    qDebug() << "DataBroker initialized with robots:" << robots;
}

void DataBroker::createPcdSub(const QString& robot, const std::string& topic)
{
    auto qos = rclcpp::SensorDataQoS();           // best-effort, depth 5
    pcdSubs_[topic] = create_subscription<sensor_msgs::msg::PointCloud2>(
        topic, qos,
        [this, robot](const sensor_msgs::msg::PointCloud2::SharedPtr msg){
            auto tmp = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
            pcl::fromROSMsg(*msg, *tmp);               // Convert once
            CloudConstPtr cloud = tmp;
            emit cloudArrived(robot, cloud);  // Qt → GUI thread
        });
    // qDebug() << "Subscribed to" << robot << "at" << QString::fromStdString(topic);
}

void DataBroker::createPathSub(const QString& robot, const std::string& topic)
{
    auto qos = rclcpp::SensorDataQoS();
    pathSubs_[topic] = create_subscription<nav_msgs::msg::Path>(
        topic, qos,
        [this, robot](const nav_msgs::msg::Path::SharedPtr msg){
            // Convert Path message to vector
            std::vector<geometry_msgs::msg::PoseStamped> pathVector = msg->poses;
            emit pathArrived(robot, pathVector);  // Send converted vector
        });
}