/* viewer_container.hpp */
#pragma once
#include <QWidget>
#include "viewer_window.hpp"


namespace Widget {

class ViewerContainer final : public QWidget
{
    Q_OBJECT
public:
    ViewerContainer(const QString &robot,
                    rclcpp::Node::SharedPtr node,
                    QOpenGLContext *share,
                    QWidget *parent = nullptr);
private:
    QWidget *container_{nullptr};
};

} // namespace Widget