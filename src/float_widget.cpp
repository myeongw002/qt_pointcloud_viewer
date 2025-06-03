#include "float_widget.hpp"
#include <QDebug>

namespace Widget {

FloatWidget::FloatWidget(QWidget *parent) :
    QWidget(parent)
{
}

FloatWidget::~FloatWidget()
{
}

void FloatWidget::setFloatingState(bool floating)
{
    if (isFloating_ == floating) return;

    isFloating_ = floating;
    if (floating) {
        setParent(nullptr);
        setWindowFlags(Qt::Window | Qt::WindowTitleHint);
        // move(100, 100); // 초기 위치 설정
        resize(400, 300); // 크기 설정
        show();
    } else {
        setWindowFlags(Qt::Widget);
        show();
    }
}

void FloatWidget::setGridPosition(int row, int col)
{
    gridRow_ = row;
    gridCol_ = col;
}

void FloatWidget::closeEvent(QCloseEvent *event)
{
    if (isFloating_) {
        emit requestDock(objectName(), gridRow_, gridCol_);
        deleteLater();
        event->accept();
    } else {
        event->accept();
    }
}

} // namespace Widget