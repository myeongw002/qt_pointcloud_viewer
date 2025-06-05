#ifndef FLOATWIDGET_H
#define FLOATWIDGET_H

#include <QWidget>
#include <QPushButton>
#include <QVBoxLayout>
#include <QCloseEvent>

namespace Widget {
class FloatWidget : public QWidget
{
    Q_OBJECT

public:
    explicit FloatWidget(QWidget *parent = nullptr);
    ~FloatWidget();
    void setFloatingState(bool floating);
    bool isFloating() const { return isFloating_; }
    void setGridPosition(int row, int col) ;

signals:
    void requestDock(const QString &widgetName, int row, int col); // 인자 추가
    void aboutToBeDeleted(); // 삭제 전 알림 추가
    
protected:
    void closeEvent(QCloseEvent *event) override; // 창 닫기 이벤트 처리

private:
    QPushButton *exampleButton_;
    QVBoxLayout *layout_;
    bool isFloating_ = false;
    int gridRow_ = 0; // 그리드 위치 행
    int gridCol_ = 0; // 그리드 위치 열
};
} // namespace Widget

#endif // FLOATWIDGET_H