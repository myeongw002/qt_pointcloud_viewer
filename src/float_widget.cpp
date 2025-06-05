#include "float_widget.hpp"
#include <QDebug>
#include <QTimer>
#include <QWindow>
#include <QApplication>

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
        // setWindowFlags(Qt::Window | Qt::WindowTitleHint | Qt::WindowStaysOnTopHint);
        // move(100, 100); // 초기 위치 설정
        resize(400, 300); // 크기 설정
        setWindowFlags(Qt::Window | Qt::WindowTitleHint);
        setStyleSheet("border: 1px solid black; background-color: white;"); // 플로팅 시각적 효과
        show();
    } else {
        setWindowFlags(Qt::Widget);
        // setParent(parentWidget());
        setStyleSheet(""); // 스타일 초기화
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
        hide();
        qDebug() << "Window hidden, checking visibility:" << (windowHandle() ? windowHandle()->isVisible() : false);

            // 이벤트 루프 정리
        QApplication::processEvents();
        qDebug() << "Events processed, windowHandle:" << windowHandle();

        // 삭제 지연
        QTimer::singleShot(0, this, [this]() {
            qDebug() << "Deleting FloatWidget after event processing" << this;
            deleteLater();
        });
        event->accept();
    } else {
        event->accept();
    }
}

} // namespace Widget