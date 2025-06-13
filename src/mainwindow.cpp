#include "./ui_mainwindow.h"
#include "mainwindow.hpp"
#include "pointcloud_widget.hpp"
#include "viewer_panel.hpp"
#include "robot_select_dialog.hpp"
#include "viewer_container.hpp"
#include "control_tree_widget.hpp"
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
        
        // ✅ PointCloudWidget들을 해시맵에 저장
        setupPointCloudWidgets();
        
        // ✅ 제어 패널 설정
        setupControlPanel();
        
    } catch (const std::exception &e) {
        std::cerr << "❌ Exception in setupUi(): " << e.what() << std::endl;
    }

    ui_->dockWidget->setVisible(false);

    connect(ui_->actionShowPanel, &QAction::triggered, this, [this]() {
        ui_->dockWidget->setVisible(!ui_->dockWidget->isVisible());
    });
    connect(ui_->actionNewViewer, &QAction::triggered, this, &MainWindow::openNewViewer);

    broker_ = std::make_shared<DataBroker>(
                QStringList{"TUGV","MUGV","SUGV1","SUGV2","SUAV"},
                this);

    // 기존 panels_ 설정 코드...
    setupViewerPanels();
    
    ros_thread_ = std::thread([this](){ rclcpp::spin(broker_); });
}

MainWindow::~MainWindow() {
    if (ros_thread_.joinable()) {
        rclcpp::shutdown();
        ros_thread_.join();
    }
    delete ui_;
}

// ✅ PointCloudWidget들 설정
void MainWindow::setupPointCloudWidgets() {
    // Qt Designer에서 생성된 PointCloudWidget들을 찾아서 저장
    // ✅ 탭 순서: COMBINED, TUGV, MUGV, SUGV1, SUGV2, SUAV (좌상단부터 오른쪽으로)
    QStringList robotNames = {"COMBINED", "TUGV", "MUGV", "SUGV1", "SUGV2", "SUAV"};
    
    for (int i = 0; i < robotNames.size() && i < panelCount_; ++i) {
        const QString& robotName = robotNames[i];
        
        // openGLWidget_0, openGLWidget_1, ... 형태로 찾기
        auto* viewer = qobject_cast<Widget::PointCloudWidget*>(
            findChild<QWidget*>(QString("openGLWidget_%1").arg(i))
        );
        
        if (viewer) {
            pointCloudWidgets_[robotName] = viewer;
            viewer->setRobot(robotName);
            std::cout << "✅ Found PointCloudWidget for " << robotName.toStdString() 
                      << " at index " << i << std::endl;
        } else {
            std::cerr << "❌ Could not find PointCloudWidget for " << robotName.toStdString() << std::endl;
        }
    }
}

// ✅ 제어 패널 설정
void MainWindow::setupControlPanel() {
    // Qt Designer에서 생성된 위젯들 가져오기
    controlDockWidget_ = ui_->dockWidget;
    controlTabWidget_ = ui_->tabWidget;
    
    if (!controlDockWidget_ || !controlTabWidget_) {
        std::cerr << "❌ Control widgets not found in UI file!" << std::endl;
        return;
    }
    
    // ✅ 탭 순서: COMBINED, TUGV, MUGV, SUGV1, SUGV2, SUAV (좌상단부터 오른쪽으로)
    QStringList robotNames = {"COMBINED", "TUGV", "MUGV", "SUGV1", "SUGV2", "SUAV"};
    
    // 기존 탭들 제거
    controlTabWidget_->clear();
    
    for (int i = 0; i < robotNames.size(); ++i) {
        const QString& robotName = robotNames[i];
        
        // TreeWidget 생성
        Widget::ControlTreeWidget* treeWidget = new Widget::ControlTreeWidget(this);
        treeWidget->setRobotName(robotName);
        
        // ✅ MainWindow 참조 설정
        treeWidget->setMainWindow(this);
        
        // 해당 PointCloudWidget과 연결
        Widget::PointCloudWidget* targetWidget = getWidgetByName(robotName);
        if (targetWidget) {
            treeWidget->setTargetWidget(targetWidget);
            std::cout << "✅ Connected TreeWidget to " << robotName.toStdString() 
                      << " (tab index: " << i << ")" << std::endl;
        } else {
            std::cerr << "❌ Could not find PointCloudWidget for " << robotName.toStdString() << std::endl;
        }
        
        // 새 탭 생성
        QWidget* tabPage = new QWidget();
        QVBoxLayout* layout = new QVBoxLayout(tabPage);
        layout->setContentsMargins(5, 5, 5, 5);
        layout->addWidget(treeWidget);
        
        // ✅ 탭에 추가 (순서 보장)
        controlTabWidget_->addTab(tabPage, robotName);
        
        // TreeWidget 저장
        controlTrees_[robotName] = treeWidget;
    }
    
    // 탭 변경 시 이벤트 연결
    connect(controlTabWidget_, &QTabWidget::currentChanged, 
            this, &MainWindow::onControlTabChanged);
    
    std::cout << "✅ Control panel setup complete with " << robotNames.size() << " tabs" << std::endl;
}

// ✅ 제어 TreeWidget들 생성
void MainWindow::createControlTrees() {
    QStringList robotNames = {"COMBINED", "TUGV", "MUGV", "SUGV1", "SUGV2", "SUAV"};
    
    // 기존 탭들 제거
    controlTabWidget_->clear();
    
    for (const QString& robotName : robotNames) {
        // TreeWidget 생성
        Widget::ControlTreeWidget* treeWidget = new Widget::ControlTreeWidget(this);
        treeWidget->setRobotName(robotName);
        
        // 해당 PointCloudWidget과 연결
        Widget::PointCloudWidget* targetWidget = getWidgetByName(robotName);
        if (targetWidget) {
            treeWidget->setTargetWidget(targetWidget);
            std::cout << "✅ Connected TreeWidget to " << robotName.toStdString() << std::endl;
        }
        
        // 새 탭 생성
        QWidget* tabPage = new QWidget();
        QVBoxLayout* layout = new QVBoxLayout(tabPage);
        layout->setContentsMargins(5, 5, 5, 5);
        layout->addWidget(treeWidget);
        
        // 탭에 추가
        controlTabWidget_->addTab(tabPage, robotName);
        
        // TreeWidget 저장
        controlTrees_[robotName] = treeWidget;
    }
}

// ✅ 로봇 이름으로 PointCloudWidget 찾기
Widget::PointCloudWidget* MainWindow::getWidgetByName(const QString& robotName) {
    // ✅ 탭 순서에 맞게 매핑: COMBINED(0), TUGV(1), MUGV(2), SUGV1(3), SUGV2(4), SUAV(5)
    int widgetIndex = -1;
    
    if (robotName == "COMBINED") {
        widgetIndex = 0;  // 좌상단
    } else if (robotName == "TUGV") {
        widgetIndex = 1;  // 우상단
    } else if (robotName == "MUGV") {
        widgetIndex = 2;  // 중앙좌
    } else if (robotName == "SUGV1") {
        widgetIndex = 3;  // 중앙우
    } else if (robotName == "SUGV2") {
        widgetIndex = 4;  // 하좌
    } else if (robotName == "SUAV") {
        widgetIndex = 5;  // 하우
    }
    
    if (widgetIndex >= 0) {
        auto* widget = qobject_cast<Widget::PointCloudWidget*>(
            findChild<QWidget*>(QString("openGLWidget_%1").arg(widgetIndex))
        );
        
        if (widget) {
            std::cout << "✅ Found widget for " << robotName.toStdString() 
                      << " at openGLWidget_" << widgetIndex << std::endl;
        } else {
            std::cerr << "❌ Widget openGLWidget_" << widgetIndex 
                      << " not found for " << robotName.toStdString() << std::endl;
        }
        
        return widget;
    }
    
    return nullptr;
}

// ✅ 기존 viewer panels 설정 (기존 코드를 함수로 분리)
void MainWindow::setupViewerPanels() {
    try {
        for (int i = 0; i < panelCount_; ++i) {
            Widget::ViewerPanel* panel = new Widget::ViewerPanel(this);
            
            auto* viewer = qobject_cast<Widget::PointCloudWidget*>(
                findChild<QWidget*>(QString("openGLWidget_%1").arg(i))
            );
            
            if (viewer) {
                panel->setPointCloudWidget(viewer);
                
                // DataBroker와 연결
                connect(broker_.get(), &DataBroker::cloudArrived,
                        viewer, &Widget::PointCloudWidget::onCloudShared,
                        Qt::QueuedConnection);
                connect(broker_.get(), &DataBroker::pathArrived,
                        viewer, &Widget::PointCloudWidget::onPathShared,
                        Qt::QueuedConnection);
            }
            
            panel->setPanelIdx_(i);
            panels_.push_back(panel);
        }
    } catch (const std::exception &e) {
        std::cerr << "❌ Exception in creating ViewerPanel: " << e.what() << std::endl;
    }
}

// ✅ 제어 탭 변경 이벤트
void MainWindow::onControlTabChanged(int index) {
    QString tabName = controlTabWidget_->tabText(index);
    std::cout << "📑 Control tab changed to: " << tabName.toStdString() << std::endl;
    
    // 현재 탭의 TreeWidget 활성화
    if (controlTrees_.contains(tabName)) {
        Widget::ControlTreeWidget* currentTree = controlTrees_[tabName];
        // 필요한 경우 추가 설정
    }
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
        connect(broker_.get(), &DataBroker::pathArrived,
                pcw,    &Widget::PointCloudWidget::onPathShared,
                Qt::QueuedConnection);
        win->show();
    }, Qt::QueuedConnection);
}
