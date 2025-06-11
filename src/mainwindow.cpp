#include "./ui_mainwindow.h"
#include "mainwindow.hpp"
#include "pointcloud_widget.hpp"
#include "viewer_panel.hpp"
#include "robot_select_dialog.hpp"
#include "viewer_container.hpp"

#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QVector3D>
#include <QComboBox>    
 #include <QTextEdit>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui_(new Ui::MainWindow) {

    try {
        std::cout << "Setting up UI..." << std::endl;
        ui_->setupUi(this);
    } catch (const std::exception &e) {
        std::cerr << "❌ Exception in setupUi(): " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "❌ Unknown Exception in setupUi()" << std::endl;
    }

    ui_->dockWidget->setVisible(false);
    connect(ui_->actionShowPanel, &QAction::triggered, this, [this]() {
        ui_->dockWidget->setVisible(!ui_->dockWidget->isVisible());
    });
    connect(ui_->actionNewViewer, &QAction::triggered, this, &MainWindow::openNewViewer);
    // if (!menuBar()) {
    //     setMenuBar(ui_->menubar);
    // }
    // ui_->menubar->setNativeMenuBar(false);
    node_ = rclcpp::Node::make_shared("qt_pointcloud_viewer");
    broker_ = std::make_shared<DataBroker>(
                QStringList{"TUGV","MUGV","SUGV1","SUGV2","SUAV"},
                this);          // this = QObject parent


    // auto viewer0_ = findChild<Widget::PointCloudWidget*>("openGLWidget_0");
    // viewer0_->setRobot("");
    // connect(broker_.get(), &DataBroker::cloudArrived,
    //     viewer0_,        &Widget::PointCloudWidget::onCloudShared,
    //     Qt::QueuedConnection);

    try {
        for (int i = 0; i < panelCount_; ++i) {
            Widget::ViewerPanel* panel = new Widget::ViewerPanel(this);
            
            auto* viewer = qobject_cast<Widget::PointCloudWidget*>(findChild<QWidget*>(QString("openGLWidget_%1").arg(i)));
            if (viewer) {
                panel->setPointCloudWidget(viewer, node_);
                // viewer->setTopicName(i); // Set topic name based on panel index
                static const QStringList robots = {"COMBINED","TUGV","MUGV","SUGV1","SUGV2","SUAV"};
                viewer->setRobot(robots[i]);
                connect(broker_.get(), &DataBroker::cloudArrived,
                        viewer, &Widget::PointCloudWidget::onCloudShared,
                        Qt::QueuedConnection);
            }
            // QLabel* label = findChild<QLabel*>(QString("label_%1").arg(i+1));
            // if (label) {
            //     panel->setStatusLabel(label);
            // }
            // QComboBox* combo = findChild<QComboBox*>(QString("robotComboBox_%1").arg(i+1));
            // if (combo) {
            //     panel->setRobotComboBox(combo);
            // }
            // QCheckBox* axisCheckBox = findChild<QCheckBox*>(QString("axisCheckBox_%1").arg(i+1));
            // if (axisCheckBox) {
            //     panel->setAxisCheckBox(axisCheckBox);
            // }
            // QCheckBox* gridCheckBox = findChild<QCheckBox*>(QString("gridCheckBox_%1").arg(i+1));
            // if (gridCheckBox) {
            //     panel->setGridCheckBox(gridCheckBox);
            // }
            
            panel ->setPanelIdx_(i);
            panels_.push_back(panel);
        }
    } catch (const std::exception &e) {
        std::cerr << "❌ Exception in creating ViewerPanel: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "❌ Unknown Exception in creating ViewerPanel" << std::endl;
    }    
    

    

    ros_thread_ = std::thread([this](){ rclcpp::spin(broker_); });
}

MainWindow::~MainWindow() {
    if (ros_thread_.joinable()) {
        rclcpp::shutdown();
        ros_thread_.join();
    }
    delete ui_;
}

void MainWindow::openNewViewer()
{
    Widget::RobotSelectDialog dlg(this);
    if (dlg.exec() != QDialog::Accepted) return;

    // Store the robot name before the lambda
    QString robotName = dlg.robotName();
    
    QMetaObject::invokeMethod(this, [this, robotName]{
        auto* win = new Widget::ViewerWindow(robotName, node_, nullptr);
        auto* pcw = win->findChild<Widget::PointCloudWidget*>();
        pcw->setRobot(robotName);
        connect(broker_.get(), &DataBroker::cloudArrived,
                pcw,    &Widget::PointCloudWidget::onCloudShared,
                Qt::QueuedConnection);
        win->show();
    }, Qt::QueuedConnection);
}
    