#ifndef FLOAT_WIDGET_HPP
#define FLOAT_WIDGET_HPP

#include <QWidget>
#include <QWindow>
#include "pointcloud_widget.hpp"


namespace Widget {

class FloatWidget : public QWidget {
    Q_OBJECT
public:
    explicit FloatWidget(QWidget *parent = nullptr);
    ~FloatWidget() override;

    void setGridPosition(int row, int col);
    void setFloatingState(bool floating);

signals:
    void requestDock(const QString &name, int row, int col);
    void aboutToBeDeleted();

protected:
    void closeEvent(QCloseEvent *event) override;

private:
    bool isFloating_ = false;
    int gridRow_ = 0;
    int gridCol_ = 0;
    QWindow *floatingWindow_ = nullptr; // 플로팅 창으로 사용할 QWindow
    QWidget *windowContainer_ = nullptr; // QWindow를 감싸는 QWidget
    PointCloudWidget *pointCloudWidget_ = nullptr; // PointCloudWidget 인스턴스
};

} // namespace Widget

#endif // FLOAT_WIDGET_HPP