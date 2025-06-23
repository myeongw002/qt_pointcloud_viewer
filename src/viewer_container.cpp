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
    // Smart pointer로 ViewerWindow 생성
    viewerWindow_ = std::make_unique<ViewerWindow>(robot, node, this);
    
    // 레이아웃 설정
    auto *lay = new QVBoxLayout(this);
    lay->setContentsMargins(0,0,0,0);
    lay->addWidget(viewerWindow_.get());  // raw pointer를 레이아웃에 추가
    
    resize(800, 600);
    setAttribute(Qt::WA_DeleteOnClose);
    show();
}

ViewerContainer::~ViewerContainer() {
    // Smart pointer가 자동으로 정리함
}

ViewerWindow* ViewerContainer::getViewerWindow() const {
    return viewerWindow_.get();
}

} // namespace Widget