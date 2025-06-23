#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDockWidget>
#include <QTabWidget>
#include <QHash>
#include <QAction>
#include <QMenu>
#include <QMenuBar>
#include <QVector3D>
#include <rclcpp/rclcpp.hpp>
#include <thread>

// Project headers
#include "pointcloud_widget.hpp"
#include "data_broker.hpp"
#include "control_tree_widget.hpp"
#include "debug_console_widget.hpp"
#include "interest_object_server.hpp"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class QKeyEvent;
class QCloseEvent;
class QResizeEvent;
class QShowEvent;

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
    // Debug Console Slots (간소화)
    // ============================================================================
    void toggleDebugConsole();
    
    // ============================================================================
    // Control Panel Slots
    // ============================================================================
    void toggleControlPanel();
    void showControlPanel();
    void hideControlPanel();

protected:
    // ============================================================================
    // Event Overrides (구현 필요)
    // ============================================================================
    void closeEvent(QCloseEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;
    void showEvent(QShowEvent* event) override;
    void keyPressEvent(QKeyEvent* event) override;

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
    const int panelCount_ = 6;
    QHash<QString, Widget::PointCloudWidget*> pointCloudWidgets_;
    
    // ============================================================================
    // Control Panel Related Members
    // ============================================================================
    QTabWidget* controlTabWidget_ = nullptr;
    QDockWidget* controlDockWidget_ = nullptr;
    QHash<QString, Widget::ControlTreeWidget*> controlTrees_;
    
    // ============================================================================
    // Debug Console Related Members
    // ============================================================================
    Widget::DebugConsoleWidget* debugConsole_;
    QDockWidget* debugConsoleDock_;
    QAction* debugConsoleAction_;
    QAction* controlPanelAction_;
    
    // ============================================================================
    // 서비스 서버 (이름 통일)
    // ============================================================================
    std::shared_ptr<Service::InterestObjectServer> interestObjectServer_;

    // ============================================================================
    // Initialization Functions
    // ============================================================================
    void setupPointCloudWidgets();
    void setupViewerPanels();
    void setupControlPanel();
    void setupDebugConsole();
    void setupViewMenu();
    
    // ============================================================================
    // Helper Functions
    // ============================================================================
    void createControlTrees();
    Widget::PointCloudWidget* getWidgetByName(const QString& robotName);
    void resetToDefaultLayout();
    void updateStatusBar(const QString& message);
};

#endif // MAINWINDOW_H
