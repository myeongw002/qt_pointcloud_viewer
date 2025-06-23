#pragma once
#include <QMainWindow>
#include <memory>
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
    ~ViewerWindow();
    
    int robotToTopic(const QString &robot);
    
    // 안전한 접근자
    Widget::PointCloudWidget* getPointCloudWidget() const;
    
private:
    std::unique_ptr<Widget::PointCloudWidget> pcw_;  // Smart pointer로 변경
};

} // namespace Widget