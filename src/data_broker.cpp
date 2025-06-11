/* data_broker.cpp ------------------------------------------------------ */
#include "data_broker.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <QDebug>

static QString robotToTopic(const QString& r)
{
    const QMap<QString,QString> map{
        {"TUGV","/tugv/viz_global_cloud"},
        {"MUGV","/mugv/viz_global_cloud"},
        {"SUGV1","/sugv1/viz_global_cloud"},
        {"SUGV2","/sugv2/viz_global_cloud"},
        {"SUAV"  ,"/suav/viz_global_cloud"} };
    return map.value(r,"/");
}

DataBroker::DataBroker(const QStringList& robots, QObject *parent)
    : QObject(parent), Node("qt_pointcloud_broker", rclcpp::NodeOptions().use_intra_process_comms(true))
{
    for (const QString& r : robots)
        createSub(r, robotToTopic(r).toStdString());
}

void DataBroker::createSub(const QString& robot, const std::string& topic)
{
    auto qos = rclcpp::SensorDataQoS();           // best-effort, depth 5
    subs_[topic] = create_subscription<sensor_msgs::msg::PointCloud2>(
        topic, qos,
        [this, robot](const sensor_msgs::msg::PointCloud2::SharedPtr msg){
            auto tmp = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
            pcl::fromROSMsg(*msg, *tmp);               // 변환 1회
            CloudConstPtr cloud = tmp;
            emit cloudArrived(robot, cloud);  // Qt → GUI 스레드
        });
    qDebug() << "Subscribed to" << robot << "at" << QString::fromStdString(topic);
}
