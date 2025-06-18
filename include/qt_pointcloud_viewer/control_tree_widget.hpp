#ifndef CONTROL_TREE_WIDGET_HPP
#define CONTROL_TREE_WIDGET_HPP

#include <QWidget>
#include <QMainWindow>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QCheckBox>
#include <QSlider>
#include <QComboBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QColorDialog>
#include <QGroupBox>
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
    void onColorButtonClicked();
    void onResetColorsClicked();
    void onCameraPresetClicked();

private:
    QString robotName_;
    PointCloudWidget* targetWidget_;
    QMainWindow* mainWindow_;
    QComboBox* targetRobotCombo_;  // Target robot selection combo box
    
    // Tree group items
    QTreeWidgetItem* viewGroup_;
    QTreeWidgetItem* robotGroup_;
    QTreeWidgetItem* displayGroup_;
    QTreeWidgetItem* cameraGroup_;
    QTreeWidgetItem* indicatorGroup_;
    
    // Initialization functions
    void setupTreeStructure();
    void setupSingleRobotTree();
    void setupCombinedModeTree();
    
    // Tree composition functions (updated for 2-group structure)
    void addViewerSettings(QTreeWidgetItem* parent);  // New function
    void addRobotControls(QTreeWidgetItem* parent);   // Updated function
    
    // Legacy functions (kept for compatibility)
    void addViewControls(QTreeWidgetItem* parent);
    void addDisplayControls(QTreeWidgetItem* parent);
    void addCameraControls(QTreeWidgetItem* parent);
    void addIndicatorControls(QTreeWidgetItem* parent);
    
    // Widget creation helper functions
    QWidget* createSliderWidget(const QString& label, double min, double max, double value, 
                               std::function<void(double)> callback);
    QWidget* createComboWidget(const QString& label, const QStringList& items, 
                              const QString& current, std::function<void(const QString&)> callback);
    QWidget* createColorWidget(const QString& label, const QColor& color, 
                              std::function<void(const QColor&)> callback);
    QCheckBox* createCheckBox(bool checked, std::function<void(bool)> callback);
    QPushButton* createButton(const QString& text, std::function<void()> callback);
    
    // Added missing functions
    void updateColorButtons();
    void syncWithWidget();
    PointCloudWidget* findRobotWidget(const QString& robotName);
    
    // Color management
    QHash<QString, QPushButton*> colorButtons_;
    
    // Constants
    static const QStringList ROBOT_NAMES;
    static const QStringList ROBOT_COLORS;
};

} // namespace Widget

#endif // CONTROL_TREE_WIDGET_HPP