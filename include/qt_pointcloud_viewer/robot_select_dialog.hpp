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
    ~RobotSelectDialog() override;  // Virtual destructor declaration
    QString robotName() const;      // Return selected robot

private:
    QListWidget *list_;
};

} // namespace Widget