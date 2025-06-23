#include "viewer_window.hpp"
#include "viewer_settings_manager.hpp"
#include <QVBoxLayout>
#include <pcl_conversions/pcl_conversions.h>

namespace Widget {

ViewerWindow::ViewerWindow(const QString &robot,
                           rclcpp::Node::SharedPtr node,
                           QWidget *parent)
    : QMainWindow(parent)
{
    setAttribute(Qt::WA_DeleteOnClose);
    setWindowTitle(QString("%1 – PointCloud").arg(robot));
    resize(800, 600);

    auto *central = new QWidget;
    auto *lay = new QVBoxLayout(central);
    lay->setContentsMargins(0,0,0,0);

    // Smart pointer로 생성
    pcw_ = std::make_unique<Widget::PointCloudWidget>(central);
    pcw_->setRobot(robot);
    
    // Apply synchronized settings
    Widget::ViewerSettingsManager::instance()->synchronizeSettings(pcw_.get(), robot);
    
    // raw pointer를 레이아웃에 추가 (Qt의 부모-자식 관계)
    lay->addWidget(pcw_.get());
    setCentralWidget(central);
    
    // Setup cleanup when window closes
    connect(this, &QObject::destroyed, [this]() {
        Widget::ViewerSettingsManager::instance()->unregisterWidget(pcw_.get());
    });
}

ViewerWindow::~ViewerWindow() {
    // Smart pointer가 자동으로 정리함
    // 하지만 명시적으로 정리하고 싶다면:
    if (pcw_) {
        Widget::ViewerSettingsManager::instance()->unregisterWidget(pcw_.get());
        pcw_.reset();
    }
}

Widget::PointCloudWidget* ViewerWindow::getPointCloudWidget() const {
    return pcw_.get();
}

int ViewerWindow::robotToTopic(const QString &r) {
    const QMap<QString,int> map{
        {"TUGV", 1},
        {"MUGV", 2},
        {"SUGV1", 3},
        {"SUGV2", 4},
        {"SUAV", 5} };
    return map.value(r, 0);
}

} // namespace Widget
