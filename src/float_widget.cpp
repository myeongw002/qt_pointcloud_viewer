#include "float_widget.hpp"

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
    // if (isFloating_) {
    //     // 창 닫기 대신 도킹 요청
    //     emit requestDock();
    //     event->ignore(); // 닫기 이벤트 무시
    // } else {
    //     event->accept(); // 플로팅 상태가 아니면 닫기 허용
    // }
}

} // namespace Widget