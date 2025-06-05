#include "float_widget.hpp"
#include <QDebug>
#include <QCloseEvent>
#include <QWindow>
#include <QVBoxLayout>
#include "pointcloud_widget.hpp"
#include <QMainWindow>
#include <QSizePolicy>

namespace Widget {

FloatWidget::FloatWidget(QWidget *parent) :
    QWidget(parent),
    originalParent_(nullptr),
    isRestoring_(false)
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
}

void FloatWidget::setGridPosition(int row, int col)
{
    gridRow_ = row;
    gridCol_ = col;
}

void FloatWidget::setFloatingState(bool floating)
{
    qDebug() << "setFloatingState called with floating =" << floating << ", isFloating_ =" << isFloating_ << ", isRestoring_ =" << isRestoring_;
    if (isFloating_ == floating || isRestoring_) return;

    isFloating_ = floating;
    if (floating) {
        hide();
        qDebug() << "Hiding FloatWidget:" << this;

        if (!pointCloudWidget_) {
            qDebug() << "No PointCloudWidget available to float!";
            return;
        }

        QSizePolicy policy = pointCloudWidget_->sizePolicy();
        qDebug() << "PointCloudWidget SizePolicy before floating - Horizontal:" << policy.horizontalPolicy()
                 << ", Vertical:" << policy.verticalPolicy();

        originalParent_ = pointCloudWidget_->parentWidget();
        qDebug() << "Saving original parent:" << originalParent_;

        if (!floatingWindow_) {
            floatingWindow_ = new QWindow();
            floatingWindow_->setFlags(Qt::Window | Qt::WindowTitleHint);
            floatingWindow_->setGeometry(100, 100, 400, 300);
            floatingWindow_->setTitle(objectName());

            windowContainer_ = QWidget::createWindowContainer(floatingWindow_, nullptr);
            windowContainer_->setMinimumSize(400, 300);

            pointCloudWidget_->setParent(windowContainer_);
            QVBoxLayout *layout = new QVBoxLayout(windowContainer_);
            layout->addWidget(pointCloudWidget_);
            windowContainer_->setLayout(layout);

            pointCloudWidget_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
            policy = pointCloudWidget_->sizePolicy();
            qDebug() << "PointCloudWidget SizePolicy after floating - Horizontal:" << policy.horizontalPolicy()
                     << ", Vertical:" << policy.verticalPolicy();

            floatingWindow_->show();
            windowContainer_->show();
            pointCloudWidget_->show();
            qDebug() << "Created QWindow:" << floatingWindow_;
        }

        QObject::connect(floatingWindow_, &QWindow::visibilityChanged, this, [this](QWindow::Visibility visibility) {
            if (visibility == QWindow::Hidden) {
                qDebug() << "QWindow closed, restoring FloatWidget:" << this;
                if (floatingWindow_) {
                    floatingWindow_->deleteLater();
                    floatingWindow_ = nullptr;
                }
                if (windowContainer_) {
                    windowContainer_->deleteLater();
                    windowContainer_ = nullptr;
                }
                if (pointCloudWidget_ && this->originalParent_) {
                    isRestoring_ = true;
                    pointCloudWidget_->setParent(this->originalParent_);
                    qDebug() << "Restored PointCloudWidget to parent:" << this->originalParent_;
                    // 크기 제약 초기화
                    pointCloudWidget_->setMinimumSize(0, 0);
                    pointCloudWidget_->setMaximumSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX);
                    pointCloudWidget_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
                    QSizePolicy policy = pointCloudWidget_->sizePolicy();
                    qDebug() << "PointCloudWidget SizePolicy on restore - Horizontal:" << policy.horizontalPolicy()
                             << ", Vertical:" << policy.verticalPolicy();
                    qDebug() << "PointCloudWidget geometry" << pointCloudWidget_->geometry();
                    qDebug() << "PointCloudWidget size" << pointCloudWidget_->size();
                    QVBoxLayout *layout = qobject_cast<QVBoxLayout*>(this->originalParent_->layout());
                    if (layout) {
                        layout->removeWidget(pointCloudWidget_);
                        delete layout;
                    }
                    // QGridLayout으로 대체 (MainWindow에서 이미 처리됨)
                    isRestoring_ = false;
                }
                isFloating_ = false;
                show();
                emit requestDock(objectName(), gridRow_, gridCol_);
            }
        });
    } else {
        qDebug() << "Restoring FloatWidget, pointCloudWidget_ =" << pointCloudWidget_ << ", originalParent_ =" << originalParent_;
        if (floatingWindow_) {
            floatingWindow_->hide();
            floatingWindow_->deleteLater();
            floatingWindow_ = nullptr;
        }
        if (windowContainer_) {
            windowContainer_->deleteLater();
            windowContainer_ = nullptr;
        }
        if (pointCloudWidget_ && originalParent_) {
            pointCloudWidget_->setParent(originalParent_);
            qDebug() << "Restored PointCloudWidget to parent:" << originalParent_;
            pointCloudWidget_->setMinimumSize(0, 0);
            pointCloudWidget_->setMaximumSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX);
            pointCloudWidget_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
            QSizePolicy policy = pointCloudWidget_->sizePolicy();
            qDebug() << "PointCloudWidget SizePolicy on direct restore - Horizontal:" << policy.horizontalPolicy()
                     << ", Vertical:" << policy.verticalPolicy();
            QVBoxLayout *layout = qobject_cast<QVBoxLayout*>(originalParent_->layout());
            if (layout) {
                layout->removeWidget(pointCloudWidget_);
                delete layout;
            }
            // QGridLayout으로 대체 (MainWindow에서 처리)
            pointCloudWidget_->show();
        }
        setParent(parentWidget());
        show();
        qDebug() << "FloatWidget restored:" << this;
    }
}

void FloatWidget::allocViewer(PointCloudWidget *viewer)
{
    if (pointCloudWidget_) {
        qDebug() << "PointCloudWidget already allocated, skipping.";
        return;
    }
    pointCloudWidget_ = viewer;
    if (pointCloudWidget_) {
        qDebug() << "PointCloudWidget allocated in FloatWidget:" << this;
        pointCloudWidget_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        QSizePolicy policy = pointCloudWidget_->sizePolicy();
        qDebug() << "PointCloudWidget SizePolicy after alloc - Horizontal:" << policy.horizontalPolicy()
                 << ", Vertical:" << policy.verticalPolicy();
        qDebug() << "Float widget size:" << size();
        qDebug() << "Viewer size:" << pointCloudWidget_->size();
        qDebug() << "Float widget geometry:" << geometry();
        qDebug() << "Viewer geometry:" << pointCloudWidget_->geometry();
    } else {
        qDebug() << "Failed to allocate PointCloudWidget in FloatWidget:" << this;
    }
}

PointCloudWidget* FloatWidget::getPointCloudWidget() const
{
    return pointCloudWidget_;
}

void FloatWidget::closeEvent(QCloseEvent *event)
{
    if (isFloating_) {
        event->accept();
    } else {
        event->accept();
    }
}

} // namespace Widget