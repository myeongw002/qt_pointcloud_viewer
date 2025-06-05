#include "mainwindow.hpp"
#include "./ui_mainwindow.h"
#include "pointcloud_widget.hpp"
#include "viewer_panel.hpp"
#include "float_widget.hpp"
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QVector3D>
#include <QComboBox>    
#include <QTextEdit>
#include <QDebug>
#include <QGridLayout>


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

    // ui_->toolBar->setStyleSheet(
    //         "QToolButton {padding: 10px; }"
    // );



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
            
            auto* floatWidget = qobject_cast<Widget::FloatWidget*>(findChild<QWidget*>(QString("widget_%1").arg(i)));
            auto* viewer = qobject_cast<Widget::PointCloudWidget*>(findChild<QWidget*>(QString("openGLWidget_%1").arg(i)));
            
            if (floatWidget && viewer) {
                // FloatWidget에 PointCloudWidget 전달
                panel->setFloatWidget(floatWidget);
                panel->setPointCloudWidget(viewer, node_);
                floatWidget->allocViewer(viewer);
                viewer->setTopicName(i);
                qDebug() << "Float widget size:" << floatWidget->size();
                qDebug() << "Viewer size:" << viewer->size();
                qDebug() << "Float widget geometry:" << floatWidget->geometry();
                qDebug() << "Viewer geometry:" << viewer->geometry();
            } else {
                qDebug() << "Failed to find floatWidget or viewer for index:" << i;
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

        
        auto floatWidget_2 = panels_.at(2)->getFloatWidget();
        floatWidget_ = floatWidget_2; // Store the reference to the second float widget
        if (floatWidget_2) {
            int row = 0, col = 0;
            if (ui_->gridLayout && getWidgetGridPosition(floatWidget_2, row, col)) {
                std::cout << "Found widget_2 at grid cell (" << row << ", " << col << ")" << std::endl;
            } else {
                std::cerr << "❌ widget_2 not found in grid layout!" << std::endl;
            }
            // ui_->gridLayout->removeWidget(floatWidget_2);
            floatWidget_2->setGridPosition(row, col);   
            // floatWidget_2->setParent(this);
            floatWidget_2->setFloatingState(true);
            connect(floatWidget_2, &Widget::FloatWidget::requestDock, this, &MainWindow::handleDockRequest);
            // connect(floatWidget_2, &Widget::FloatWidget::aboutToBeDeleted, this, [this]() {
            //     qDebug() << "FloatWidget_2 about to be deleted, clearing reference";
                // floatWidget_2 = nullptr;
            // });
        } else {
            std::cerr << "❌ widget_2 not found in ViewerPanel!" << std::endl;
        }

        // floatWidget_ = qobject_cast<Widget::FloatWidget*>(findChild<QWidget*>(QString("widget_1")));
        // if (floatWidget_) {
        //     int row = 0, col = 0;
        //     if (ui_->gridLayout && getWidgetGridPosition(floatWidget_, row, col)) {
        //         std::cout << "Found widget_1 at grid cell (" << row << ", " << col << ")" << std::endl;
        //     } else {
        //         std::cerr << "❌ widget_1 not found in grid layout!" << std::endl;
        //     }
        //     // ui_->gridLayout->removeWidget(floatWidget_);
        //     floatWidget_->setGridPosition(row, col);

        //     // floatWidget_->setParent(this);
        //     floatWidget_->setFloatingState(true);
        //     connect(floatWidget_, &Widget::FloatWidget::requestDock, this, &MainWindow::handleDockRequest);
        //     connect(floatWidget_, &Widget::FloatWidget::aboutToBeDeleted, this, [this]() {
        //         qDebug() << "FloatWidget about to be deleted, clearing reference";
        //         floatWidget_ = nullptr;
        //     });
        // }
    
        
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

bool MainWindow::getWidgetGridPosition(QWidget* widget, int& row, int& col)
{
    if (!ui_->gridLayout || !widget) {
        return false;
    }
    int index = ui_->gridLayout->indexOf(widget);
    if (index != -1) {
        int rowSpan, colSpan;
        ui_->gridLayout->getItemPosition(index, &row, &col, &rowSpan, &colSpan);
        return true;
    }
    return false;
}

void MainWindow::handleDockRequest(const QString &widgetName, int row, int col)
{
    qDebug() << "handleDockRequest for" << widgetName << "at (" << row << "," << col << ")";
    if (widgetName == "widget_2" && floatWidget_) {
        floatWidget_->setVisible(true);
        if (ui_->gridLayout) {
            ui_->gridLayout->addWidget(floatWidget_, row, col);
            
            // FloatWidget 내부에 QGridLayout 확인 및 추가
            QGridLayout *gridLayout = qobject_cast<QGridLayout*>(floatWidget_->layout());
            if (!gridLayout) {
                qDebug() << "No QGridLayout found in FloatWidget_2, creating new one";
                QLayout *oldLayout = floatWidget_->layout();
                if (oldLayout) {
                    qDebug() << "Removing old layout";
                    oldLayout->removeWidget(floatWidget_->findChild<Widget::PointCloudWidget*>());
                    delete oldLayout;
                }
                gridLayout = new QGridLayout(floatWidget_);
                gridLayout->setContentsMargins(0, 0, 0, 0);
                gridLayout->setRowStretch(0, 1);
                gridLayout->setColumnStretch(0, 1);
                floatWidget_->setLayout(gridLayout);
                if (floatWidget_->getPointCloudWidget()) {
                    gridLayout->addWidget(floatWidget_->getPointCloudWidget(), 0, 0);
                    qDebug() << "Added pointCloudWidget_ to QGridLayout";
                }
            } else {
                qDebug() << "QGridLayout already exists in FloatWidget_2";
            }

            QSizePolicy sizePolicy = floatWidget_->sizePolicy();
            qDebug() << "FloatWidget_2 size policy before:" << sizePolicy.horizontalPolicy() << sizePolicy.verticalPolicy();
            floatWidget_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
            sizePolicy = floatWidget_->sizePolicy();
            qDebug() << "FloatWidget_2 size policy after:" << sizePolicy.horizontalPolicy() << sizePolicy.verticalPolicy();

            qDebug() << "Added to gridLayout, floatWidget_2 size =" << floatWidget_->size();
            qDebug() << "FloatWidget_2 geometry:" << floatWidget_->geometry();
        }
    }
}
// void MainWindow::closeEvent(QCloseEvent *event)
// {
//     qDebug() << "MainWindow closeEvent called";
//     // event->ignore(); // 종료 방지 테스트
// }