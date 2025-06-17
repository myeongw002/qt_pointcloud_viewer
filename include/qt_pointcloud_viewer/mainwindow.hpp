#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDockWidget>      // Added
#include <QTabWidget>       // Added
#include <QHash>
#include <QAction>          // Added
#include <QMenu>            // Added
#include <QMenuBar>         // Added
#include <QVector3D>
#include <rclcpp/rclcpp.hpp>
#include <thread>

// Project headers
#include "pointcloud_widget.hpp"
#include "data_broker.hpp"
#include "control_tree_widget.hpp"
#include "debug_console_widget.hpp"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    // ============================================================================
    // Basic Slots
    // ============================================================================
    void openNewViewer();
    void onControlTabChanged(int index);
    
    // ============================================================================
    // Debug Console Slots
    // ============================================================================
    void toggleDebugConsole();
    void showDebugConsole();
    void hideDebugConsole();

private:
    // ============================================================================
    // UI Structure Members
    // ============================================================================
    Ui::MainWindow *ui_;
    
    // ============================================================================
    // ROS2 Related Members
    // ============================================================================
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<DataBroker> broker_;
    std::thread ros_thread_;
    
    // ============================================================================
    // Viewer Related Members
    // ============================================================================
    Widget::PointCloudWidget *viewer_;                                    // Main viewer
    const int panelCount_ = 6;                                          // Panel count
    QHash<QString, Widget::PointCloudWidget*> pointCloudWidgets_;       // Robot-specific widgets
    
    // ============================================================================
    // Control Panel Related Members
    // ============================================================================
    QTabWidget* controlTabWidget_ = nullptr;                            // Control tab widget
    QDockWidget* controlDockWidget_ = nullptr;                          // Control dock widget
    QHash<QString, Widget::ControlTreeWidget*> controlTrees_;           // Robot-specific TreeWidgets
    
    // ============================================================================
    // Debug Console Related Members
    // ============================================================================
    Widget::DebugConsoleWidget* debugConsole_ = nullptr;                // Debug console widget
    QDockWidget* debugConsoleDock_ = nullptr;                           // Debug console dock widget
    QAction* debugConsoleAction_ = nullptr;                             // Menu action
    
    // ============================================================================
    // Menu Related Members
    // ============================================================================
    QMenu* fileMenu_ = nullptr;                                         // File menu
    QMenu* viewMenu_ = nullptr;                                         // View menu
    QMenu* toolsMenu_ = nullptr;                                        // Tools menu
    QMenu* helpMenu_ = nullptr;                                         // Help menu
    
    // File menu actions
    QAction* newViewerAction_ = nullptr;
    QAction* saveLogAction_ = nullptr;
    QAction* exitAction_ = nullptr;
    
    // View menu actions
    QAction* showControlPanelAction_ = nullptr;
    QAction* resetLayoutAction_ = nullptr;
    
    // Tools menu actions
    QAction* resetAllCamerasAction_ = nullptr;
    QAction* resetAllColorsAction_ = nullptr;
    
    // ============================================================================
    // Initialization Functions
    // ============================================================================
    void setupUI();                              // Overall UI setup
    void setupMenuBar();                         // Menu bar setup
    void setupStatusBar();                       // Status bar setup
    void setupPointCloudWidgets();               // Point cloud widgets setup
    void setupViewerPanels();                    // Viewer panels setup
    void setupControlPanel();                    // Control panel setup
    void setupDebugConsole();                    // Debug console setup
    
    // ============================================================================
    // Helper Functions
    // ============================================================================
    void createControlTrees();                   // Create control trees
    void connectControlSignals();                // Connect control signals
    void connectMenuActions();                   // Connect menu actions
    
    // Widget search functions
    Widget::PointCloudWidget* getWidgetByName(const QString& robotName);
    Widget::PointCloudWidget* findPointCloudWidget(const QString& objectName);
    
    // Layout management functions
    void saveLayout();                           // Save layout
    void restoreLayout();                        // Restore layout
    void resetToDefaultLayout();                 // Reset to default layout
    
    // ============================================================================
    // Utility Functions
    // ============================================================================
    void updateStatusBar(const QString& message);               // Update status bar
    void logToConsole(const QString& message, 
                     Widget::DebugConsoleWidget::LogLevel level = 
                     Widget::DebugConsoleWidget::INFO);         // Console log
    
    // Settings management
    void loadSettings();                         // Load settings
    void saveSettings();                         // Save settings
    
protected:
    // ============================================================================
    // Event Overrides
    // ============================================================================
    void closeEvent(QCloseEvent* event) override;               // Window close event
    void resizeEvent(QResizeEvent* event) override;             // Resize event
    void showEvent(QShowEvent* event) override;                 // Show event
    void keyPressEvent(QKeyEvent* event) override;              // Key press event
};

#endif // MAINWINDOW_H
