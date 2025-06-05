#include "float_widget.hpp"
#include <QDebug>
#include <QTimer>
#include <QWindow>


namespace Widget {

FloatWidget::FloatWidget(QWidget *parent) :
    QWidget(parent)
{
    setAttribute(Qt::WA_DeleteOnClose, false);
}

FloatWidget::~FloatWidget()
{
    qDebug() << "FloatWidget destroyed";
}

void FloatWidget::setFloatingState(bool floating)
{
    if (isFloating_ == floating) return;

    isFloating_ = floating;
    if (floating) {
        setParent(nullptr);
        setWindowFlags(Qt::Window | Qt::WindowTitleHint | Qt::WindowStaysOnTopHint);
        // setParent(nullptr);
        // move(100, 100); // 초기 위치 설정
        resize(400, 300); // 크기 설정
        if (QWindow *win = windowHandle()) {
            win->setFlags(Qt::Window | Qt::WindowTitleHint | Qt::WindowStaysOnTopHint);
        }
        show();
    } else {
        setWindowFlags(Qt::Widget);
        setParent(parentWidget());
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
        emit aboutToBeDeleted();
        // 비동기적으로 deleteLater 호출
        QTimer::singleShot(0, this, [this]() {
            deleteLater();
        });
        event->accept();
    } else {
        event->accept();
    }
}

} // namespace Widget