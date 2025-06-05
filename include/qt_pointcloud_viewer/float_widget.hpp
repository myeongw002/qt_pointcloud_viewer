#ifndef FLOAT_WIDGET_HPP
#define FLOAT_WIDGET_HPP

#include <QWidget>
#include <QWindow>


namespace Widget {

class PointCloudWidget;

class FloatWidget : public QWidget {
    Q_OBJECT
public:
    explicit FloatWidget(QWidget *parent = nullptr);
    ~FloatWidget() override;

    void setGridPosition(int row, int col);
    void setFloatingState(bool floating);
    void allocViewer(PointCloudWidget *viewer);
    PointCloudWidget* getPointCloudWidget() const ;
signals:
    void requestDock(const QString &name, int row, int col);
    void aboutToBeDeleted();

protected:
    void closeEvent(QCloseEvent *event) override;

private:
    bool isFloating_ = false;
    int gridRow_ = 0;
    int gridCol_ = 0;
    QWindow *floatingWindow_ = nullptr;
    QWidget *windowContainer_ = nullptr;
    PointCloudWidget *pointCloudWidget_ = nullptr;
    QWidget *originalParent_ = nullptr;
    bool isRestoring_ = false; // 중복 복원 방지
};

} // namespace Widget

#endif // FLOAT_WIDGET_HPP