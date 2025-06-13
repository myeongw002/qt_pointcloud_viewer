#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QVector3D>
#include <QDockWidget>
#include <QMainWindow>
#include <QHash>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include "pointcloud_widget.hpp"
#include "viewer_panel.hpp"
#include "data_broker.hpp"
#include "control_tree_widget.hpp"



QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE




class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void openNewViewer();
    void onControlTabChanged(int index);

private:
    Ui::MainWindow *ui_;
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<DataBroker> broker_;
    std::thread ros_thread_;
    Widget::PointCloudWidget *viewer_;
    std::vector<Widget::ViewerPanel*> panels_;
    const int panelCount_ = 6;
    QTabWidget* controlTabWidget_ = nullptr;           // 제어 탭 위젯
    QDockWidget* controlDockWidget_ = nullptr;         // 제어 독 위젯
    QHash<QString, Widget::ControlTreeWidget*> controlTrees_;  // 로봇별 TreeWidget들
    QHash<QString, Widget::PointCloudWidget*> pointCloudWidgets_;  // 로봇별 위젯들
    
    void setupPointCloudWidgets();
    void setupViewerPanels();
    void setupControlPanel();
    void createControlTrees();
    Widget::PointCloudWidget* getWidgetByName(const QString& robotName);
    void connectControlSignals();
};

#endif // MAINWINDOW_H
