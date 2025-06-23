/* viewer_container.hpp */
#pragma once
#include <QWidget>
#include <memory>
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
    ~ViewerContainer();
    
    // 안전한 접근자
    ViewerWindow* getViewerWindow() const;
    
private:
    std::unique_ptr<ViewerWindow> viewerWindow_;  // Smart pointer로 변경
};

} // namespace Widget