#include "float_widget.hpp"
#include <QDebug>
#include <QCloseEvent>
#include <QWindow>
#include <QVBoxLayout>
#include "pointcloud_widget.hpp"
#include <QMainWindow>

namespace Widget {

FloatWidget::FloatWidget(QWidget *parent) :
    QWidget(parent)
{
    setAttribute(Qt::WA_DeleteOnClose, false);
}

FloatWidget::~FloatWidget()
{
    qDebug() << "FloatWidget destroyed";
    if (floatingWindow_) {
        floatingWindow_->deleteLater();
    }
    if (windowContainer_) {
        windowContainer_->deleteLater();
    }
    if (pointCloudWidget_) {
        pointCloudWidget_->deleteLater();
    }
}

void FloatWidget::setGridPosition(int row, int col)
{
    gridRow_ = row;
    gridCol_ = col;
}

void FloatWidget::setFloatingState(bool floating)
{
    if (isFloating_ == floating) return;

    isFloating_ = floating;
    if (floating) {
        // 기존 QWidget 숨기기
        hide();
        qDebug() << "Hiding FloatWidget:" << this;

        // 새로운 QWindow 생성
        if (!floatingWindow_) {
            floatingWindow_ = new QWindow();
            floatingWindow_->setFlags(Qt::Window | Qt::WindowTitleHint);
            floatingWindow_->setGeometry(100, 100, 400, 300);
            floatingWindow_->setTitle(objectName());

            // QWindow을 QWidget으로 감싸기
            windowContainer_ = QWidget::createWindowContainer(floatingWindow_, nullptr);
            windowContainer_->setMinimumSize(400, 300);

            // // PointCloudWidget 생성 및 추가
            pointCloudWidget_ = new PointCloudWidget(windowContainer_);
            QVBoxLayout *layout = new QVBoxLayout(windowContainer_);
            layout->addWidget(pointCloudWidget_);
            windowContainer_->setLayout(layout);

            floatingWindow_->show();
            windowContainer_->show();
            qDebug() << "Created QWindow:" << floatingWindow_;
        }

        // QWindow 닫기 이벤트 처리
        QObject::connect(floatingWindow_, &QWindow::visibilityChanged, this, [this](QWindow::Visibility visibility) {
            if (visibility == QWindow::Hidden) {
                // QWindow가 닫힘
                qDebug() << "QWindow closed, restoring FloatWidget:" << this;
                if (floatingWindow_) {
                    floatingWindow_->deleteLater();
                    floatingWindow_ = nullptr;
                }
                if (windowContainer_) {
                    windowContainer_->deleteLater();
                    windowContainer_ = nullptr;
                }
                if (pointCloudWidget_) {
                    pointCloudWidget_->deleteLater();
                    pointCloudWidget_ = nullptr;
                }
                setFloatingState(false); // 플로팅 상태 해제
                show(); // 기존 QWidget 표시
                emit requestDock(objectName(), gridRow_, gridCol_);
            }
        });
    } else {
        // 플로팅 해제 시
        if (floatingWindow_) {
            floatingWindow_->hide();
            floatingWindow_->deleteLater();
            floatingWindow_ = nullptr;
        }
        if (windowContainer_) {
            windowContainer_->deleteLater();
            windowContainer_ = nullptr;
        }
        if (pointCloudWidget_) {
            pointCloudWidget_->deleteLater();
            pointCloudWidget_ = nullptr;
        }
        setParent(parentWidget()); // 부모 재설정
        show(); // 기존 QWidget 표시
        qDebug() << "FloatWidget restored:" << this;
    }
}

void FloatWidget::closeEvent(QCloseEvent *event)
{
    if (isFloating_) {
        // 플로팅 상태에서는 QWindow가 닫히므로 여기서 처리하지 않음
        event->accept();
    } else {
        event->accept();
    }
}

} // namespace Widget