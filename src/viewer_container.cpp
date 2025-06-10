/* viewer_container.cpp */
#include "viewer_container.hpp"
#include "viewer_window.hpp"
#include <QVBoxLayout>

namespace Widget {

ViewerContainer::ViewerContainer(const QString &robot,
                                 rclcpp::Node::SharedPtr node,
                                 QOpenGLContext *share,
                                 QWidget *parent)
    : QWidget(parent)
{
    auto *win = new Widget::ViewerWindow(robot, node, parent);
    // container_ = QWidget::createWindowContainer(win, this);
    // container_->setAttribute(Qt::WA_DeleteOnClose);

    auto *lay = new QVBoxLayout(this);
    lay->setContentsMargins(0,0,0,0);
    lay->addWidget(container_);
    resize(800,600);
    setAttribute(Qt::WA_DeleteOnClose);
    show();
}

} // namespace Widget