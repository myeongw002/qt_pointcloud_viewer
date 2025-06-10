/* robot_select_dialog.cpp */
#include "robot_select_dialog.hpp"
#include <QVBoxLayout>

namespace Widget {

RobotSelectDialog::RobotSelectDialog(QWidget *parent)
    : QDialog(parent), list_(new QListWidget)
{
    setWindowTitle("Select Robot");
    list_->addItems({"TUGV","MUGV","SUGV1","SUGV2","SUAV"});
    list_->setCurrentRow(0);

    auto *btn = new QPushButton("Open Viewer");
    connect(btn, &QPushButton::clicked, this, &QDialog::accept);

    auto *lay = new QVBoxLayout(this);
    lay->addWidget(list_);
    lay->addWidget(btn);
}

RobotSelectDialog::~RobotSelectDialog() {}


QString RobotSelectDialog::robotName() const
{
    return list_->currentItem()->text();
}


} // namespace Widget

