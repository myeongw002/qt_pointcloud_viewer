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
#include <QMenuBar>        // QMenuBar header
#include <QMenu>           // QMenu header
#include <QAction>         // QAction header (may already exist)
#include <QDockWidget>     // QDockWidget header
#include <QKeySequence>    // QKeySequence header
#include <QTabWidget>      // QTabWidget header

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui_(new Ui::MainWindow)  // ui_ initialization fix
    , debugConsole_(nullptr)
    , debugConsoleDock_(nullptr)
    , debugConsoleAction_(nullptr)
{
    try {
        std::cout << "Setting up UI..." << std::endl;
        ui_->setupUi(this);
        
        // Store PointCloudWidgets in hash map
        setupPointCloudWidgets();
        
        // Setup control panel
        setupControlPanel();
        
        // Setup debug console
        setupDebugConsole();
        
    } catch (const std::exception &e) {
        std::cerr << "Exception in setupUi(): " << e.what() << std::endl;
    }

    ui_->dockWidget->setVisible(false);

    connect(ui_->actionShowPanel, &QAction::triggered, this, [this]() {
        ui_->dockWidget->setVisible(!ui_->dockWidget->isVisible());
    });
    connect(ui_->actionNewViewer, &QAction::triggered, this, &MainWindow::openNewViewer);

    broker_ = std::make_shared<DataBroker>(
                QStringList{"TUGV","MUGV","SUGV1","SUGV2","SUAV"},
                this);

    // Existing panels_ setup code...
    setupViewerPanels();
    
    ros_thread_ = std::thread([this](){ rclcpp::spin(broker_); });
}

MainWindow::~MainWindow() {
    // Debug console cleanup
    if (debugConsole_) {
        delete debugConsole_;
    }
    
    if (ros_thread_.joinable()) {
        rclcpp::shutdown();
        ros_thread_.join();
    }
    delete ui_;
}

// Setup PointCloudWidgets
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
            
            // Simply register (proper default values are automatically applied)
            Widget::ViewerSettingsManager::instance()->registerWidget(robotName, viewer);
            
            std::cout << "Found PointCloudWidget for " << robotName.toStdString() 
                      << " at index " << i << std::endl;
        } else {
            std::cerr << "Could not find PointCloudWidget for " << robotName.toStdString() << std::endl;
        }
    }
    
    qDebug() << "All widgets initialized with proper default settings";
}

// Setup control panel
void MainWindow::setupControlPanel() {
    // Get widgets created from Qt Designer
    controlDockWidget_ = ui_->dockWidget;
    controlTabWidget_ = ui_->tabWidget;
    
    if (!controlDockWidget_ || !controlTabWidget_) {
        std::cerr << "Control widgets not found in UI file!" << std::endl;
        return;
    }
    
    // Tab order: COMBINED, TUGV, MUGV, SUGV1, SUGV2, SUAV (from top-left to right)
    QStringList robotNames = {"COMBINED", "TUGV", "MUGV", "SUGV1", "SUGV2", "SUAV"};
    
    // Remove existing tabs
    controlTabWidget_->clear();
    
    for (int i = 0; i < robotNames.size(); ++i) {
        const QString& robotName = robotNames[i];
        
        // Create TreeWidget
        Widget::ControlTreeWidget* treeWidget = new Widget::ControlTreeWidget(this);
        treeWidget->setRobotName(robotName);
        
        // Set MainWindow reference
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
        
        // Create new tab
        QWidget* tabPage = new QWidget();
        QVBoxLayout* layout = new QVBoxLayout(tabPage);
        layout->setContentsMargins(5, 5, 5, 5);
        layout->addWidget(treeWidget);
        
        // Add to tab (order guaranteed)
        controlTabWidget_->addTab(tabPage, robotName);
        
        // Store TreeWidget
        controlTrees_[robotName] = treeWidget;
    }
    
    // Connect tab change event
    connect(controlTabWidget_, &QTabWidget::currentChanged, 
            this, &MainWindow::onControlTabChanged);
    
    std::cout << "Control panel setup complete with " << robotNames.size() << " tabs" << std::endl;
}

// Create control TreeWidgets
void MainWindow::createControlTrees() {
    QStringList robotNames = {"COMBINED", "TUGV", "MUGV", "SUGV1", "SUGV2", "SUAV"};
    
    // Remove existing tabs
    controlTabWidget_->clear();
    
    for (const QString& robotName : robotNames) {
        // Create TreeWidget
        Widget::ControlTreeWidget* treeWidget = new Widget::ControlTreeWidget(this);
        treeWidget->setRobotName(robotName);
        
        // Connect to corresponding PointCloudWidget
        Widget::PointCloudWidget* targetWidget = getWidgetByName(robotName);
        if (targetWidget) {
            treeWidget->setTargetWidget(targetWidget);
            std::cout << "Connected TreeWidget to " << robotName.toStdString() << std::endl;
        }
        
        // Create new tab
        QWidget* tabPage = new QWidget();
        QVBoxLayout* layout = new QVBoxLayout(tabPage);
        layout->setContentsMargins(5, 5, 5, 5);
        layout->addWidget(treeWidget);
        
        // Add to tab
        controlTabWidget_->addTab(tabPage, robotName);
        
        // Store TreeWidget
        controlTrees_[robotName] = treeWidget;
    }
}

// Find PointCloudWidget by robot name
Widget::PointCloudWidget* MainWindow::getWidgetByName(const QString& robotName) {
    // Map according to tab order: COMBINED(0), TUGV(1), MUGV(2), SUGV1(3), SUGV2(4), SUAV(5)
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

// Setup existing viewer panels (separated existing code into function)
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

// Control tab change event
void MainWindow::onControlTabChanged(int index) {
    QString tabName = controlTabWidget_->tabText(index);
    std::cout << "Control tab changed to: " << tabName.toStdString() << std::endl;
    
    // Activate current tab's TreeWidget
    if (controlTrees_.contains(tabName)) {
        Widget::ControlTreeWidget* currentTree = controlTrees_[tabName];
        // Additional settings if needed
    }
}

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
            
            // Synchronize settings with existing viewers (after connections are made)
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

// Setup debug console
void MainWindow::setupDebugConsole() {
    // Create debug console widget
    debugConsole_ = new Widget::DebugConsoleWidget(this);
    
    // Wrap with dock widget
    debugConsoleDock_ = new QDockWidget("Debug Console", this);
    debugConsoleDock_->setWidget(debugConsole_);
    debugConsoleDock_->setAllowedAreas(Qt::BottomDockWidgetArea | Qt::RightDockWidgetArea);
    
    // Add dock widget to main window
    addDockWidget(Qt::BottomDockWidgetArea, debugConsoleDock_);
    
    // Initially hidden
    debugConsoleDock_->hide();
    
    // Improved menu bar handling
    QMenuBar* mainMenuBar = menuBar();  // Get menu bar pointer
    
    if (mainMenuBar) {
        // Find or create View menu
        QMenu* viewMenu = nullptr;
        
        // Find View menu among existing menus
        QList<QAction*> menuActions = mainMenuBar->actions();
        for (QAction* action : menuActions) {
            if (action->text() == "View" || action->text() == "&View") {
                viewMenu = action->menu();
                break;
            }
        }
        
        // Create new View menu if not found
        if (!viewMenu) {
            viewMenu = mainMenuBar->addMenu("&View");
            qDebug() << "Created new View menu";
        } else {
            qDebug() << "Found existing View menu";
        }
        
        // Add debug console toggle action
        debugConsoleAction_ = viewMenu->addAction("&Debug Console");
        debugConsoleAction_->setCheckable(true);
        debugConsoleAction_->setShortcut(QKeySequence("F12"));
        debugConsoleAction_->setStatusTip("Show/hide debug console (F12)");
        
        connect(debugConsoleAction_, &QAction::triggered, 
                this, &MainWindow::toggleDebugConsole);
                
        qDebug() << "Debug console action added to View menu";
    } else {
        qDebug() << "Could not access menu bar";
    }
    
    // Synchronize dock widget show/hide with action state
    connect(debugConsoleDock_, &QDockWidget::visibilityChanged, 
            [this](bool visible) {
        if (debugConsoleAction_) {
            debugConsoleAction_->setChecked(visible);
        }
        qDebug() << "Debug console visibility:" << (visible ? "SHOWN" : "HIDDEN");
    });
    
    qDebug() << "Debug console setup completed";
}

void MainWindow::toggleDebugConsole() {
    if (debugConsoleDock_) {
        if (debugConsoleDock_->isVisible()) {
            debugConsoleDock_->hide();
            qDebug() << "Debug console hidden";
        } else {
            debugConsoleDock_->show();
            debugConsoleDock_->raise();  // Bring to front
            qDebug() << "Debug console shown";
        }
    } else {
        qDebug() << "Debug console dock widget not found";
    }
}

// Additional debug console slots implementation
void MainWindow::showDebugConsole() {
    if (debugConsoleDock_) {
        debugConsoleDock_->show();
        debugConsoleDock_->raise();
        qDebug() << "Debug console shown (explicit)";
    }
}

void MainWindow::hideDebugConsole() {
    if (debugConsoleDock_) {
        debugConsoleDock_->hide();
        qDebug() << "Debug console hidden (explicit)";
    }
}

// Utility functions implementation
void MainWindow::updateStatusBar(const QString& message) {
    if (statusBar()) {
        statusBar()->showMessage(message, 3000);  // Show for 3 seconds
        qDebug() << "Status:" << message;
    }
}

void MainWindow::logToConsole(const QString& message, Widget::DebugConsoleWidget::LogLevel level) {
    if (debugConsole_) {
        debugConsole_->appendLog(message, level);
    } else {
        // Fallback: output to qDebug
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

// Settings management functions implementation
void MainWindow::saveSettings() {
    // Save window state with QSettings
    // To be implemented later
    qDebug() << "Settings saved";
}

void MainWindow::loadSettings() {
    // Restore window state with QSettings
    // To be implemented later
    qDebug() << "Settings loaded";
}

void MainWindow::resetToDefaultLayout() {
    // Reset to default layout
    if (debugConsoleDock_) {
        debugConsoleDock_->hide();
    }
    if (controlDockWidget_) {
        controlDockWidget_->show();
    }
    qDebug() << "Layout reset to default";
}

// Event overrides
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
    // Toggle debug console with F12 key
    if (event->key() == Qt::Key_F12) {
        toggleDebugConsole();
        event->accept();
        return;
    }
    
    QMainWindow::keyPressEvent(event);
}
