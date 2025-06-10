#pragma once
#include <QMainWindow>
#include "pointcloud_widget.hpp"
#include <rclcpp/rclcpp.hpp>

namespace Widget {

class ViewerWindow final : public QMainWindow
{
    Q_OBJECT
public:
    ViewerWindow(const QString &robot,
                 rclcpp::Node::SharedPtr node,
                 QWidget *parent = nullptr);
    int robotToTopic(const QString &robot);
    
private:
    Widget::PointCloudWidget *pcw_;
};

} // namespace Widget