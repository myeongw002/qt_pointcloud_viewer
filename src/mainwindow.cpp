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
            
            auto* viewer = qobject_cast<Widget::PointCloudWidget*>(findChild<QWidget*>(QString("openGLWidget_%1").arg(i)));
            if (viewer) {
                panel->setPointCloudWidget(viewer, node_);
                viewer->setTopicName(i); // Set topic name based on panel index
            }
            // auto* floatWidget = qobject_cast<Widget::FloatWidget*>(findChild<QWidget*>(QString("widget_1")));
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

        auto* floatWidget = qobject_cast<Widget::FloatWidget*>(findChild<QWidget*>(QString("widget_1")));
        if (floatWidget ) {
            floatWidget_ = floatWidget;
            int row = 0, col = 0;
            if (ui_->gridLayout && getWidgetGridPosition(floatWidget, row, col)) {
                std::cout << "Found  at grid cell (" << row << ", " << col << ")" << std::endl;
            } 
            else {
                std::cerr << "❌  not found in grid layout!" << std::endl;
            }
                ui_->gridLayout->removeWidget(floatWidget);
                floatWidget->setGridPosition(row, col);
            
            floatWidget_->setParent(this);
            floatWidget_->setFloatingState(true);
            // floatWidget->move(100, 100); // 초기 위치 설정
            // floatWidget->resize(400, 300); // 크기 설정
            //qDebug() << "FloatWidget parent after setup:" << floatWidget_->parent();
            connect(floatWidget_, &Widget::FloatWidget::requestDock, this, &MainWindow::handleDockRequest);
            connect(floatWidget_, &Widget::FloatWidget::aboutToBeDeleted, this, [this]() {
                qDebug() << "FloatWidget about to be deleted, clearing reference";
                floatWidget_ = nullptr;
            });
            if (floatWidget_) {
                qDebug() << "FloatWidget parent after setup:" << (floatWidget_->parent() ? "Valid parent" : "No parent");
            } else {
                qDebug() << "FloatWidget is null after setup";
            }
        }
        else {
            std::cerr << "❌ Custom float widget not found!" << std::endl;
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
    qDebug() << "handleDockRequest called for" << widgetName << "at (" << row << "," << col << ")";
    qDebug() << "floatWidget_:" << floatWidget_ << ", isFloating:" << (floatWidget_ ? floatWidget_->isFloating() : false);
    qDebug() << "Top level widgets before:" << QApplication::topLevelWidgets();

    if (floatWidget_ && floatWidget_->objectName() == widgetName && floatWidget_->isFloating()) {
        floatWidget_->setFloatingState(false);
        floatWidget_->setParent(this);
        if (ui_->gridLayout) {
            ui_->gridLayout->addWidget(floatWidget_, row, col);
        }
        qDebug() << "FloatWidget parent after setParent:" << (floatWidget_->parent() ? "Valid parent" : "No parent");
        floatWidget_->show();
        floatWidget_->blockSignals(true); // 신호 차단
        floatWidget_->setEnabled(false); // 상호작용 비활성화
        std::cout << "Float widget " << widgetName.toStdString() << " docked back to grid cell (" << row << ", " << col << ")" << std::endl;
        // setAttribute(Qt::WA_DeleteOnClose, false);
        // floatWidget_->hide();
    } else {
        std::cerr << "❌ Invalid float widget state or layout for " << widgetName.toStdString() << std::endl;
    }

    qDebug() << "Top level widgets after:" << QApplication::topLevelWidgets();
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    qDebug() << "MainWindow closeEvent called";
    event->ignore(); // 종료 방지 테스트
}