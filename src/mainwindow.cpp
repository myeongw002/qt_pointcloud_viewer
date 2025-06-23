#include "./ui_mainwindow.h"
#include "mainwindow.hpp"
#include "pointcloud_widget.hpp"
#include "robot_select_dialog.hpp"
#include "viewer_container.hpp"
#include "control_tree_widget.hpp"
#include "debug_console_widget.hpp"
#include "viewer_settings_manager.hpp"

// Qt widget headers
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QVector3D>
#include <QComboBox>    
#include <QTextEdit>
#include <QDebug>
#include <QMenuBar>
#include <QMenu>
#include <QAction>
#include <QDockWidget>
#include <QKeySequence>
#include <QTabWidget>
#include <QKeyEvent>

// ============================================================================
// Constructor & Destructor
// ============================================================================
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui_(new Ui::MainWindow)
    , debugConsole_(nullptr)
    , debugConsoleDock_(nullptr)
    , debugConsoleAction_(nullptr)
    , controlPanelAction_(nullptr)
{
    try {
        std::cout << "Setting up UI..." << std::endl;
        ui_->setupUi(this);
        
        // Initialize all components
        setupPointCloudWidgets();
        setupControlPanel();
        setupDebugConsole();
        
        // Initial visibility (변경: 기본값을 숨김으로 설정)
        ui_->dockWidget->setVisible(false);  // ← true에서 false로 변경
        
        // Connect actions
        connect(ui_->actionShowPanel, &QAction::triggered, this, [this]() {
                ui_->dockWidget->setVisible(!ui_->dockWidget->isVisible());
            });
        connect(ui_->actionNewViewer, &QAction::triggered, this, &MainWindow::openNewViewer);
        
        // Setup broker and threads
        broker_ = std::make_shared<DataBroker>(
                    QStringList{"TUGV","MUGV","SUGV1","SUGV2","SUAV"},
                    this);
        
        // ROS node 생성
        node_ = std::make_shared<rclcpp::Node>("qt_pointcloud_viewer_main");
        
        // Interest Object 서비스 서버 초기화 (클래스 이름 통일)
        interestObjectServer_ = std::make_shared<Service::InterestObjectServer>(node_, this);
        
        // 서비스 서버 시그널 연결 (클래스 이름 통일)
        connect(interestObjectServer_.get(), &Service::InterestObjectServer::serviceRequestReceived,
                this, [this](const QString& robotId, int objectCount) {
            updateStatusBar(QString("Received %1 objects from %2").arg(objectCount).arg(robotId));
            qDebug() << "Service request processed:" << robotId << "objects:" << objectCount;
        });
        
        connect(interestObjectServer_.get(), &Service::InterestObjectServer::objectsRegistered,
                this, [this](int count) {
            qDebug() << "Successfully registered" << count << "interest objects";
        });
        
        setupViewerPanels();
        ros_thread_ = std::thread([this](){
            try {
                // MultiThreadedExecutor 생성
                rclcpp::executors::MultiThreadedExecutor executor;
                
                // 두 노드 모두 추가
                executor.add_node(broker_);      // DataBroker 노드
                executor.add_node(node_);        // 서비스 서버 노드

                qDebug() << "ROS executor initialized with nodes";
                RCLCPP_INFO(node_->get_logger(), "MultiThreadedExecutor spinning with service server");
                
                // 실행 시작
                executor.spin();
                
            } catch (const std::exception& e) {
                std::cerr << "❌ ROS executor error: " << e.what() << std::endl;
                RCLCPP_ERROR(node_->get_logger(), "Executor error: %s", e.what());
            }
        });
        
    } catch (const std::exception &e) {
        std::cerr << "Exception in setupUi(): " << e.what() << std::endl;
    }
}

MainWindow::~MainWindow() {
    if (debugConsole_) {
        delete debugConsole_;
    }
    
    if (ros_thread_.joinable()) {
        rclcpp::shutdown();
        ros_thread_.join();
    }
    delete ui_;
}

// ============================================================================
// Setup Functions
// ============================================================================
void MainWindow::setupPointCloudWidgets() {
    QStringList robotNames = {"COMBINED", "TUGV", "MUGV", "SUGV1", "SUGV2", "SUAV"};
    
    for (int i = 0; i < robotNames.size() && i < panelCount_; ++i) {
        const QString& robotName = robotNames[i];
        
        auto* viewer = qobject_cast<Widget::PointCloudWidget*>(
            findChild<QWidget*>(QString("openGLWidget_%1").arg(i))
        );
        
        if (viewer) {
            pointCloudWidgets_[robotName] = viewer;
            viewer->setRobot(robotName);
            Widget::ViewerSettingsManager::instance()->registerWidget(robotName, viewer);
            
        } else {
            std::cerr << "Could not find PointCloudWidget for " << robotName.toStdString() << std::endl;
        }
    }
    
    qDebug() << "All widgets initialized with proper default settings";
}

void MainWindow::setupControlPanel() {
    // Get widgets from UI
    controlDockWidget_ = ui_->dockWidget;
    controlTabWidget_ = ui_->tabWidget;
    
    if (!controlDockWidget_ || !controlTabWidget_) {
        std::cerr << "Control widgets not found in UI file!" << std::endl;
        return;
    }
    
    // Create control trees
    createControlTrees();
    
    // Connect tab change event
    connect(controlTabWidget_, &QTabWidget::currentChanged, 
            this, &MainWindow::onControlTabChanged);
    
    std::cout << "Control panel setup complete" << std::endl;
}

void MainWindow::createControlTrees() {
    QStringList robotNames = {"COMBINED", "TUGV", "MUGV", "SUGV1", "SUGV2", "SUAV"};
    
    // Clear existing tabs
    controlTabWidget_->clear();
    
    for (int i = 0; i < robotNames.size(); ++i) {
        const QString& robotName = robotNames[i];
        
        // Create TreeWidget
        Widget::ControlTreeWidget* treeWidget = new Widget::ControlTreeWidget(this);
        treeWidget->setRobotName(robotName);
        treeWidget->setMainWindow(this);
        
        // Connect to corresponding PointCloudWidget
        Widget::PointCloudWidget* targetWidget = getWidgetByName(robotName);
        if (targetWidget) {
            treeWidget->setTargetWidget(targetWidget);
            std::cout << "Connected TreeWidget to " << robotName.toStdString() 
                      << " (tab index: " << i << ")" << std::endl;
        } else {
            std::cerr << "Could not find PointCloudWidget for " << robotName.toStdString() << std::endl;
        }
        
        // Create tab page
        QWidget* tabPage = new QWidget();
        QVBoxLayout* layout = new QVBoxLayout(tabPage);
        layout->setContentsMargins(5, 5, 5, 5);
        layout->addWidget(treeWidget);
        
        // Add tab
        controlTabWidget_->addTab(tabPage, robotName);
        controlTrees_[robotName] = treeWidget;
    }
}

void MainWindow::setupDebugConsole() {
    // Create debug console widget
    debugConsole_ = new Widget::DebugConsoleWidget(this);
    
    // Wrap with dock widget
    debugConsoleDock_ = new QDockWidget("Debug Console", this);
    debugConsoleDock_->setWidget(debugConsole_);
    debugConsoleDock_->setAllowedAreas(Qt::BottomDockWidgetArea | Qt::RightDockWidgetArea);
    
    // Add to main window (initially hidden)
    addDockWidget(Qt::BottomDockWidgetArea, debugConsoleDock_);
    debugConsoleDock_->hide();
    
    // Setup view menu
    setupViewMenu();
    
    // Connect visibility signal
    connect(debugConsoleDock_, &QDockWidget::visibilityChanged, 
            [this](bool visible) {
        if (debugConsoleAction_) {
            debugConsoleAction_->setChecked(visible);
        }
        qDebug() << "Debug console visibility:" << (visible ? "SHOWN" : "HIDDEN");
    });
    
    qDebug() << "Debug console setup completed";
}

void MainWindow::setupViewMenu() {
    QMenuBar* mainMenuBar = menuBar();
    if (!mainMenuBar) {
        qDebug() << "Could not access menu bar";
        return;
    }
    
    // Find or create View menu
    QMenu* viewMenu = nullptr;
    QList<QAction*> menuActions = mainMenuBar->actions();
    for (QAction* action : menuActions) {
        if (action->text() == "View" || action->text() == "&View") {
            viewMenu = action->menu();
            break;
        }
    }
    
    if (!viewMenu) {
        viewMenu = mainMenuBar->addMenu("&View");
        qDebug() << "Created new View menu";
    } else {
        qDebug() << "Found existing View menu";
    }
    
    // Add Control Panel Toggle (변경: 기본값을 unchecked로 설정)
    controlPanelAction_ = viewMenu->addAction("&Control Panel");
    controlPanelAction_->setCheckable(true);
    controlPanelAction_->setChecked(false);  // ← true에서 false로 변경
    controlPanelAction_->setShortcut(QKeySequence("Ctrl+P"));
    controlPanelAction_->setStatusTip("Show/hide control panel (Ctrl+P)");
    
    connect(controlPanelAction_, &QAction::triggered, 
            this, &MainWindow::toggleControlPanel);
    
    // Connect control dock visibility
    if (controlDockWidget_) {
        connect(controlDockWidget_, &QDockWidget::visibilityChanged, 
                [this](bool visible) {
            if (controlPanelAction_) {
                controlPanelAction_->setChecked(visible);
            }
            qDebug() << "Control panel visibility:" << (visible ? "SHOWN" : "HIDDEN");
        });
    }
    
    // Add Debug Console Toggle (기존과 동일)
    debugConsoleAction_ = viewMenu->addAction("&Debug Console");
    debugConsoleAction_->setCheckable(true);
    debugConsoleAction_->setChecked(false);
    debugConsoleAction_->setShortcut(QKeySequence("F12"));
    debugConsoleAction_->setStatusTip("Show/hide debug console (F12)");
    
    connect(debugConsoleAction_, &QAction::triggered, 
            this, &MainWindow::toggleDebugConsole);
    
    // Add separator and reset layout
    viewMenu->addSeparator();
    
    QAction* resetLayoutAction = viewMenu->addAction("&Reset Layout");
    resetLayoutAction->setShortcut(QKeySequence("Ctrl+R"));
    resetLayoutAction->setStatusTip("Reset window layout to default (Ctrl+R)");
    
    connect(resetLayoutAction, &QAction::triggered, 
            this, &MainWindow::resetToDefaultLayout);
    
    qDebug() << "View menu setup completed with Control Panel and Debug Console toggles";
}

void MainWindow::setupViewerPanels() {
    try {
        for (int i = 0; i < panelCount_; ++i) {            
            auto* viewer = qobject_cast<Widget::PointCloudWidget*>(
                findChild<QWidget*>(QString("openGLWidget_%1").arg(i))
            );
            
            if (viewer) {                
                // Connect to DataBroker
                connect(broker_.get(), &DataBroker::cloudArrived,
                        viewer, &Widget::PointCloudWidget::onCloudShared,
                        Qt::QueuedConnection);
                connect(broker_.get(), &DataBroker::pathArrived,
                        viewer, &Widget::PointCloudWidget::onPathShared,
                        Qt::QueuedConnection);
            }
        }
    } catch (const std::exception &e) {
        std::cerr << "Exception in creating ViewerPanel: " << e.what() << std::endl;
    }
}

// ============================================================================
// Helper Functions
// ============================================================================
Widget::PointCloudWidget* MainWindow::getWidgetByName(const QString& robotName) {
    int widgetIndex = -1;
    
    if (robotName == "COMBINED") widgetIndex = 0;
    else if (robotName == "TUGV") widgetIndex = 1;
    else if (robotName == "MUGV") widgetIndex = 2;
    else if (robotName == "SUGV1") widgetIndex = 3;
    else if (robotName == "SUGV2") widgetIndex = 4;
    else if (robotName == "SUAV") widgetIndex = 5;
    
    if (widgetIndex >= 0) {
        auto* widget = qobject_cast<Widget::PointCloudWidget*>(
            findChild<QWidget*>(QString("openGLWidget_%1").arg(widgetIndex))
        );
        
        if (widget) {
            std::cout << "Found widget for " << robotName.toStdString() 
                      << " at openGLWidget_" << widgetIndex << std::endl;
        } else {
            std::cerr << "Widget openGLWidget_" << widgetIndex 
                      << " not found for " << robotName.toStdString() << std::endl;
        }
        
        return widget;
    }
    
    return nullptr;
}

void MainWindow::updateStatusBar(const QString& message) {
    if (statusBar()) {
        statusBar()->showMessage(message, 3000);  // Show for 3 seconds
    }
}

// ============================================================================
// Event Handlers
// ============================================================================
void MainWindow::onControlTabChanged(int index) {
    if (!controlTabWidget_ || index < 0 || index >= controlTabWidget_->count()) {
        qDebug() << "Invalid tab index:" << index;
        return;
    }
    
    QString tabName = controlTabWidget_->tabText(index);
    qDebug() << "Control tab changed to:" << tabName << "at index:" << index;
    
    // Optional: Handle specific robot tab changes
    if (controlTrees_.contains(tabName)) {
        Widget::ControlTreeWidget* currentTree = controlTrees_[tabName];
        if (currentTree) {
            // Ensure the tree is properly synchronized with its target widget
            // currentTree->syncWithWidget(); // Uncomment if needed
            qDebug() << "Switched to control tree for:" << tabName;
        }
    } else {
        qDebug() << "No control tree found for tab:" << tabName;
    }
    
    // Optional: Update status bar
    updateStatusBar(QString("Switched to %1 controls").arg(tabName));
}

void MainWindow::closeEvent(QCloseEvent* event) {
    qDebug() << "MainWindow closing...";
    
    // Save window geometry and state
    // QSettings settings;
    // settings.setValue("geometry", saveGeometry());
    // settings.setValue("windowState", saveState());
    
    // Cleanup ROS thread
    if (ros_thread_.joinable()) {
        rclcpp::shutdown();
        ros_thread_.join();
        qDebug() << "ROS thread terminated";
    }
    
    // Accept the close event
    event->accept();
    qDebug() << "MainWindow closed successfully";
}

void MainWindow::resizeEvent(QResizeEvent* event) {
    // Call parent implementation
    QMainWindow::resizeEvent(event);
    
    // Optional: Log resize for debugging
    // qDebug() << "MainWindow resized to:" << event->size();
    
    // Optional: Save window size
    // QSettings settings;
    // settings.setValue("size", event->size());
}

void MainWindow::showEvent(QShowEvent* event) {
    // Call parent implementation
    QMainWindow::showEvent(event);
    
    // Optional: Restore window geometry on first show
    static bool firstShow = true;
    if (firstShow) {
        qDebug() << "MainWindow shown for the first time";
        
        // Optional: Restore saved geometry
        // QSettings settings;
        // restoreGeometry(settings.value("geometry").toByteArray());
        // restoreState(settings.value("windowState").toByteArray());
        
        firstShow = false;
    }
}

void MainWindow::keyPressEvent(QKeyEvent* event) {
    // Handle keyboard shortcuts
    if (event->key() == Qt::Key_F12) {
        toggleDebugConsole();
        event->accept();
        return;
    }
    
    if (event->modifiers() & Qt::ControlModifier) {
        switch (event->key()) {
        case Qt::Key_P:
            toggleControlPanel();
            event->accept();
            return;
        case Qt::Key_R:
            resetToDefaultLayout();
            event->accept();
            return;
        }
    }
    
    // Call parent implementation for unhandled keys
    QMainWindow::keyPressEvent(event);
}

// ============================================================================
// Toggle Functions
// ============================================================================
void MainWindow::toggleControlPanel() {
    if (controlDockWidget_) {
        if (controlDockWidget_->isVisible()) {
            controlDockWidget_->hide();
            qDebug() << "Control panel hidden";
            updateStatusBar("Control panel hidden");
        } else {
            controlDockWidget_->show();
            controlDockWidget_->raise();
            qDebug() << "Control panel shown";
            updateStatusBar("Control panel shown");
        }
    } else {
        qDebug() << "Control dock widget not found";
    }
}

void MainWindow::showControlPanel() {
    if (controlDockWidget_) {
        controlDockWidget_->show();
        controlDockWidget_->raise();
        qDebug() << "Control panel shown (explicit)";
        updateStatusBar("Control panel shown");
    }
}

void MainWindow::hideControlPanel() {
    if (controlDockWidget_) {
        controlDockWidget_->hide();
        qDebug() << "Control panel hidden (explicit)";
        updateStatusBar("Control panel hidden");
    }
}

void MainWindow::toggleDebugConsole() {
    if (debugConsoleDock_) {
        if (debugConsoleDock_->isVisible()) {
            debugConsoleDock_->hide();
            qDebug() << "Debug console hidden";
            updateStatusBar("Debug console hidden");
        } else {
            debugConsoleDock_->show();
            debugConsoleDock_->raise();
            qDebug() << "Debug console shown";
            updateStatusBar("Debug console shown");
        }
    } else {
        qDebug() << "Debug console dock widget not found";
    }
}

void MainWindow::resetToDefaultLayout() {
    qDebug() << "Resetting layout to default...";
    
    // Hide control panel (변경: 기본값을 숨김으로 설정)
    if (controlDockWidget_) {
        controlDockWidget_->hide();  // ← show()에서 hide()로 변경
    }
    
    // Hide debug console (기존과 동일)
    if (debugConsoleDock_) {
        debugConsoleDock_->hide();
    }
    
    // Reset dock widget positions
    if (controlDockWidget_) {
        addDockWidget(Qt::LeftDockWidgetArea, controlDockWidget_);
    }
    
    if (debugConsoleDock_) {
        addDockWidget(Qt::BottomDockWidgetArea, debugConsoleDock_);
    }
    
    // Update menu action states (변경: Control Panel을 unchecked로 설정)
    if (controlPanelAction_) {
        controlPanelAction_->setChecked(false);  // ← true에서 false로 변경
    }
    
    if (debugConsoleAction_) {
        debugConsoleAction_->setChecked(false);
    }
    
    updateStatusBar("Layout reset to default");
    qDebug() << "Layout reset completed";
}

// ============================================================================
// Application Functions
// ============================================================================
void MainWindow::openNewViewer() {
    Widget::RobotSelectDialog dlg(this);
    if (dlg.exec() != QDialog::Accepted) return;

    QString robotName = dlg.robotName();
    
    QMetaObject::invokeMethod(this, [this, robotName]{
        auto* win = new Widget::ViewerWindow(robotName, node_, nullptr);
        auto* pcw = win->findChild<Widget::PointCloudWidget*>();
        
        if (pcw) {
            pcw->setRobot(robotName);
            
            qDebug() << "MainWindow: Creating new viewer for" << robotName;
            
            // Connect to data broker first
            connect(broker_.get(), &DataBroker::cloudArrived,
                    pcw, &Widget::PointCloudWidget::onCloudShared,
                    Qt::QueuedConnection);
            connect(broker_.get(), &DataBroker::pathArrived,
                    pcw, &Widget::PointCloudWidget::onPathShared,
                    Qt::QueuedConnection);
            
            // Synchronize settings with existing viewers
            Widget::ViewerSettingsManager::instance()->synchronizeSettings(pcw, robotName);
            
            // Setup cleanup when window closes
            connect(win, &QObject::destroyed, [pcw]() {
                Widget::ViewerSettingsManager::instance()->unregisterWidget(pcw);
            });
            
            qDebug() << "MainWindow: New viewer setup completed for" << robotName;
        }
        
        win->show();
    }, Qt::QueuedConnection);
}