#include "./ui_mainwindow.h"
#include "mainwindow.hpp"
#include "pointcloud_widget.hpp"
#include "robot_select_dialog.hpp"
#include "viewer_container.hpp"
#include "control_tree_widget.hpp"
#include "debug_console_widget.hpp"

// ✅ Qt 위젯 헤더들 추가
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QVector3D>
#include <QComboBox>    
#include <QTextEdit>
#include <QDebug>
#include <QMenuBar>        // ✅ QMenuBar 헤더 추가
#include <QMenu>           // ✅ QMenu 헤더 추가
#include <QAction>         // ✅ QAction 헤더 추가 (이미 있을 수 있음)
#include <QDockWidget>     // ✅ QDockWidget 헤더 추가
#include <QKeySequence>    // ✅ QKeySequence 헤더 추가
#include <QTabWidget>      // ✅ QTabWidget 헤더 추가

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui_(new Ui::MainWindow)  // ✅ ui_ 초기화 수정
    , debugConsole_(nullptr)
    , debugConsoleDock_(nullptr)
    , debugConsoleAction_(nullptr)
{
    try {
        std::cout << "Setting up UI..." << std::endl;
        ui_->setupUi(this);
        
        // ✅ PointCloudWidget들을 해시맵에 저장
        setupPointCloudWidgets();
        
        // ✅ 제어 패널 설정
        setupControlPanel();
        
        // ✅ 디버그 콘솔 설정 추가
        setupDebugConsole();
        
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
    // 디버그 콘솔 정리
    if (debugConsole_) {
        delete debugConsole_;
    }
    
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
        widgetIndex = 0; 
    } else if (robotName == "TUGV") {
        widgetIndex = 1; 
    } else if (robotName == "MUGV") {
        widgetIndex = 2;  
    } else if (robotName == "SUGV1") {
        widgetIndex = 3; 
    } else if (robotName == "SUGV2") {
        widgetIndex = 4; 
    } else if (robotName == "SUAV") {
        widgetIndex = 5; 
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
            auto* viewer = qobject_cast<Widget::PointCloudWidget*>(
                findChild<QWidget*>(QString("openGLWidget_%1").arg(i))
            );
            
            if (viewer) {                
                // DataBroker와 연결
                connect(broker_.get(), &DataBroker::cloudArrived,
                        viewer, &Widget::PointCloudWidget::onCloudShared,
                        Qt::QueuedConnection);
                connect(broker_.get(), &DataBroker::pathArrived,
                        viewer, &Widget::PointCloudWidget::onPathShared,
                        Qt::QueuedConnection);
            }
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

// 디버그 콘솔 설정
void MainWindow::setupDebugConsole() {
    // 디버그 콘솔 위젯 생성
    debugConsole_ = new Widget::DebugConsoleWidget(this);
    
    // 도킹 위젯으로 감싸기
    debugConsoleDock_ = new QDockWidget("Debug Console", this);
    debugConsoleDock_->setWidget(debugConsole_);
    debugConsoleDock_->setAllowedAreas(Qt::BottomDockWidgetArea | Qt::RightDockWidgetArea);
    
    // 메인 윈도우에 도킹 위젯 추가
    addDockWidget(Qt::BottomDockWidgetArea, debugConsoleDock_);
    
    // 초기에는 숨김
    debugConsoleDock_->hide();
    
    // ✅ 메뉴바 처리 개선
    QMenuBar* mainMenuBar = menuBar();  // 메뉴바 포인터 얻기
    
    if (mainMenuBar) {
        // View 메뉴 찾기 또는 생성
        QMenu* viewMenu = nullptr;
        
        // 기존 메뉴들 중에서 View 메뉴 찾기
        QList<QAction*> menuActions = mainMenuBar->actions();
        for (QAction* action : menuActions) {
            if (action->text() == "View" || action->text() == "&View") {
                viewMenu = action->menu();
                break;
            }
        }
        
        // View 메뉴가 없으면 새로 생성
        if (!viewMenu) {
            viewMenu = mainMenuBar->addMenu("&View");
            qDebug() << "✅ Created new View menu";
        } else {
            qDebug() << "✅ Found existing View menu";
        }
        
        // 디버그 콘솔 토글 액션 추가
        debugConsoleAction_ = viewMenu->addAction("&Debug Console");
        debugConsoleAction_->setCheckable(true);
        debugConsoleAction_->setShortcut(QKeySequence("F12"));
        debugConsoleAction_->setStatusTip("Show/hide debug console (F12)");
        
        connect(debugConsoleAction_, &QAction::triggered, 
                this, &MainWindow::toggleDebugConsole);
                
        qDebug() << "✅ Debug console action added to View menu";
    } else {
        qDebug() << "❌ Could not access menu bar";
    }
    
    // 도킹 위젯 표시/숨김과 액션 상태 동기화
    connect(debugConsoleDock_, &QDockWidget::visibilityChanged, 
            [this](bool visible) {
        if (debugConsoleAction_) {
            debugConsoleAction_->setChecked(visible);
        }
        qDebug() << "🖥️ Debug console visibility:" << (visible ? "SHOWN" : "HIDDEN");
    });
    
    qDebug() << "✅ Debug console setup completed";
}

void MainWindow::toggleDebugConsole() {
    if (debugConsoleDock_) {
        if (debugConsoleDock_->isVisible()) {
            debugConsoleDock_->hide();
            qDebug() << "🖥️ Debug console hidden";
        } else {
            debugConsoleDock_->show();
            debugConsoleDock_->raise();  // 앞으로 가져오기
            qDebug() << "🖥️ Debug console shown";
        }
    } else {
        qDebug() << "❌ Debug console dock widget not found";
    }
}

// ✅ 추가 디버그 콘솔 슬롯들 구현
void MainWindow::showDebugConsole() {
    if (debugConsoleDock_) {
        debugConsoleDock_->show();
        debugConsoleDock_->raise();
        qDebug() << "🖥️ Debug console shown (explicit)";
    }
}

void MainWindow::hideDebugConsole() {
    if (debugConsoleDock_) {
        debugConsoleDock_->hide();
        qDebug() << "🖥️ Debug console hidden (explicit)";
    }
}

// ✅ 유틸리티 함수들 구현
void MainWindow::updateStatusBar(const QString& message) {
    if (statusBar()) {
        statusBar()->showMessage(message, 3000);  // 3초간 표시
        qDebug() << "📊 Status:" << message;
    }
}

void MainWindow::logToConsole(const QString& message, Widget::DebugConsoleWidget::LogLevel level) {
    if (debugConsole_) {
        debugConsole_->appendLog(message, level);
    } else {
        // 폴백: qDebug로 출력
        switch (level) {
            case Widget::DebugConsoleWidget::DEBUG:
                qDebug() << message;
                break;
            case Widget::DebugConsoleWidget::INFO:
                qInfo() << message;
                break;
            case Widget::DebugConsoleWidget::WARNING:
                qWarning() << message;
                break;
            case Widget::DebugConsoleWidget::CRITICAL:
                qCritical() << message;
                break;
        }
    }
}

Widget::PointCloudWidget* MainWindow::findPointCloudWidget(const QString& objectName) {
    return qobject_cast<Widget::PointCloudWidget*>(findChild<QWidget*>(objectName));
}

// ✅ 설정 관리 함수들 구현
void MainWindow::saveSettings() {
    // QSettings로 창 상태 저장
    // 나중에 구현
    qDebug() << "💾 Settings saved";
}

void MainWindow::loadSettings() {
    // QSettings로 창 상태 복원
    // 나중에 구현
    qDebug() << "📂 Settings loaded";
}

void MainWindow::resetToDefaultLayout() {
    // 기본 레이아웃으로 리셋
    if (debugConsoleDock_) {
        debugConsoleDock_->hide();
    }
    if (controlDockWidget_) {
        controlDockWidget_->show();
    }
    qDebug() << "🔄 Layout reset to default";
}

// ✅ 이벤트 오버라이드들
void MainWindow::closeEvent(QCloseEvent* event) {
    saveSettings();
    QMainWindow::closeEvent(event);
}

void MainWindow::resizeEvent(QResizeEvent* event) {
    QMainWindow::resizeEvent(event);
    updateStatusBar(QString("Window resized to %1x%2")
                   .arg(event->size().width())
                   .arg(event->size().height()));
}

void MainWindow::showEvent(QShowEvent* event) {
    QMainWindow::showEvent(event);
    loadSettings();
}

void MainWindow::keyPressEvent(QKeyEvent* event) {
    // F12 키로 디버그 콘솔 토글
    if (event->key() == Qt::Key_F12) {
        toggleDebugConsole();
        event->accept();
        return;
    }
    
    QMainWindow::keyPressEvent(event);
}
