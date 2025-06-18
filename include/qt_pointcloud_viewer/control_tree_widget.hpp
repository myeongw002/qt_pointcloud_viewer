#ifndef CONTROL_TREE_WIDGET_HPP
#define CONTROL_TREE_WIDGET_HPP

#include <QWidget>
#include <QMainWindow>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QCheckBox>
#include <QSlider>
#include <QComboBox>
#include <QPushButton>
#include <QColorDialog>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <functional>

#include "pointcloud_widget.hpp"

namespace Widget {

class ControlTreeWidget : public QTreeWidget {
    Q_OBJECT

public:
    explicit ControlTreeWidget(QWidget* parent = nullptr);
    ~ControlTreeWidget() = default;
    
    void setRobotName(const QString& robotName);
    void setTargetWidget(PointCloudWidget* widget);
    void setMainWindow(QMainWindow* mainWindow);

private slots:
    void onItemChanged(QTreeWidgetItem* item, int column);

private:
    // ============================================================================
    // Core Members
    // ============================================================================
    QString robotName_;
    PointCloudWidget* targetWidget_;
    QMainWindow* mainWindow_;
    QComboBox* targetRobotCombo_;
    
    // Tree group items (2-group structure only)
    QTreeWidgetItem* viewGroup_;
    QTreeWidgetItem* robotGroup_;
    
    // ============================================================================
    // Core Functions
    // ============================================================================
    void setupTreeStructure();
    void addViewerSettings(QTreeWidgetItem* parent);
    void addRobotControls(QTreeWidgetItem* parent);
    void syncWithWidget();
    void updateColorButtons();
    
    // ============================================================================
    // Helper Functions
    // ============================================================================
    QCheckBox* createCheckBox(bool checked, std::function<void(bool)> callback);
    QPushButton* createButton(const QString& text, std::function<void()> callback);
    PointCloudWidget* findRobotWidget(const QString& robotName);
    
    // ============================================================================
    // Data Members
    // ============================================================================
    QHash<QString, QPushButton*> colorButtons_;
    
    // Constants
    static const QStringList ROBOT_NAMES;
    static const QStringList ROBOT_COLORS;
};

} // namespace Widget

#endif // CONTROL_TREE_WIDGET_HPP