#include "mainwindow.hpp"
#include "./ui_mainwindow.h"
#include "pointcloud_widget.hpp"
#include "viewer_panel.hpp"
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QVector3D>
#include <QComboBox>    

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


    node_ = rclcpp::Node::make_shared("qt_pointcloud_viewer");
    
    try {
        for (int i = 0; i < 3; ++i) {
            ViewerPanel* panel = new ViewerPanel(this);
        
            QLabel* label = findChild<QLabel*>(QString("label_%1").arg(i+1));
            if (label) {
                panel->setStatusLabel(label);
            }
        
            QComboBox* combo = findChild<QComboBox*>(QString("comboBox_%1").arg(i+1));
            if (combo) {
                panel->setComboBox(combo);
            }
        
            auto* viewer = qobject_cast<PointCloudWidget*>(findChild<QWidget*>(QString("openGLWidget_%1").arg(i+1)));
            if (viewer) {
                panel->setPointCloudWidget(viewer, node_);
            }
            
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

/*
void MainWindow::startStreaming() {
    if (currentIndex_ == 0) {
        std::cout << "❌ No topic selected. Please select a topic." << std::endl;
        return ;
    } 

    viewer1_->setStartFlag(true);
}
*/

// void MainWindow::updateStatus(const QString &status) {
//     ui_->label->setText(status);
//     ui_->label->adjustSize(); 
// }

