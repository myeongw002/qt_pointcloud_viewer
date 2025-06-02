#include "float_widget.hpp"
#include <QDebug>
namespace Widget {

FloatWidget::FloatWidget(QWidget *parent) :
    QWidget(parent)
{
    // 기본 크기 설정
    // resize(400, 300);
}

FloatWidget::~FloatWidget()
{
    // 더 이상 ui 객체가 없으므로 delete 불필요
}

void FloatWidget::setFloatingState(bool floating)
{
    if (isFloating_ == floating) return;

    isFloating_ = floating;
    if (floating) {
        setParent(nullptr);
        setWindowFlags(Qt::Window | Qt::WindowTitleHint);
        show();
    } else {
        setWindowFlags(Qt::Widget);
        show();
    }
}

} // namespace Widget