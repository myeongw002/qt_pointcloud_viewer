/* robot_select_dialog.hpp */
#pragma once
#include <QDialog>
#include <QListWidget>
#include <QPushButton>

namespace Widget {

class RobotSelectDialog final : public QDialog
{
    Q_OBJECT
public:
    explicit RobotSelectDialog(QWidget *parent = nullptr);
    ~RobotSelectDialog() override; // 가상 소멸자 선언
    QString robotName() const;               // 선택된 로봇 반환

private:
    QListWidget *list_;
};

}