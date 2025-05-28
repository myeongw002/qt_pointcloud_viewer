#include "mainwindow.hpp"
#include "./ui_mainwindow.h"
#include "pointcloud_widget.hpp"
#include "viewer_panel.hpp"
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

    ui_->toolBar->setStyleSheet(
            "QToolButton {padding: 5px; }"
    );

    ui_->dockWidget->setVisible(false);
    connect(ui_->actionShowPanel, &QAction::triggered, this, [this]() {
        ui_->dockWidget->setVisible(!ui_->dockWidget->isVisible());
    });
    // if (!menuBar()) {
    //     setMenuBar(ui_->menubar);
    // }
    // ui_->menubar->setNativeMenuBar(false);
    node_ = rclcpp::Node::make_shared("qt_pointcloud_viewer");
    
    try {
        for (int i = 0; i < panelCount_; ++i) {
            Widget::ViewerPanel* panel = new Widget::ViewerPanel(this);
            
            auto* viewer = qobject_cast<Widget::PointCloudWidget*>(findChild<QWidget*>(QString("openGLWidget_%1").arg(i)));
            if (viewer) {
                panel->setPointCloudWidget(viewer, node_);
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
    

    

    ros_thread_ = std::thread([this]() { rclcpp::spin(node_); });
}

MainWindow::~MainWindow() {
    if (ros_thread_.joinable()) {
        rclcpp::shutdown();
        ros_thread_.join();
    }
    delete ui_;
}

