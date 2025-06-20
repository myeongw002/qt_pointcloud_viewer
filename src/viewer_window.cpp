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

    pcw_ = new Widget::PointCloudWidget;
    pcw_->setRobot(robot);
    
    // Apply synchronized settings
    Widget::ViewerSettingsManager::instance()->synchronizeSettings(pcw_, robot);
    
    lay->addWidget(pcw_);
    setCentralWidget(central);
    
    // Setup cleanup when window closes
    connect(this, &QObject::destroyed, [this]() {
        Widget::ViewerSettingsManager::instance()->unregisterWidget(pcw_);
    });
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
