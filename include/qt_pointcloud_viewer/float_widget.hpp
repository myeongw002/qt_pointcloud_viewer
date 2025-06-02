#ifndef FLOATWIDGET_H
#define FLOATWIDGET_H

#include <QWidget>
#include <QPushButton>
#include <QVBoxLayout>

namespace Widget {
class FloatWidget : public QWidget
{
    Q_OBJECT

public:
    explicit FloatWidget(QWidget *parent = nullptr);
    ~FloatWidget();
    void setFloatingState(bool floating);
    bool isFloating() const { return isFloating_; }

private:
    bool isFloating_ = false;
};
} // namespace Widget

#endif // FLOATWIDGET_H