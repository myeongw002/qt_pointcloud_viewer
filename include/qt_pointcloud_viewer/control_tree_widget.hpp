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
    
    // 트리 그룹 아이템들
    QTreeWidgetItem* viewGroup_;
    QTreeWidgetItem* robotGroup_;
    QTreeWidgetItem* displayGroup_;
    QTreeWidgetItem* cameraGroup_;
    QTreeWidgetItem* indicatorGroup_;
    
    // 초기화 함수들
    void setupTreeStructure();
    void setupSingleRobotTree();    // 단일 로봇용 트리
    void setupCombinedModeTree();   // COMBINED 모드용 트리
    
    // 트리 구성 함수들
    void addViewControls(QTreeWidgetItem* parent);
    void addRobotControls(QTreeWidgetItem* parent);
    void addDisplayControls(QTreeWidgetItem* parent);
    void addCameraControls(QTreeWidgetItem* parent);
    void addIndicatorControls(QTreeWidgetItem* parent);
    
    // 위젯 생성 헬퍼 함수들
    QWidget* createSliderWidget(const QString& label, double min, double max, double value, 
                               std::function<void(double)> callback);
    QWidget* createComboWidget(const QString& label, const QStringList& items, 
                              const QString& current, std::function<void(const QString&)> callback);
    QWidget* createColorWidget(const QString& label, const QColor& color, 
                              std::function<void(const QColor&)> callback);
    QCheckBox* createCheckBox(bool checked, std::function<void(bool)> callback);
    QPushButton* createButton(const QString& text, std::function<void()> callback);
    
    // ✅ 누락된 함수들 추가
    void updateColorButtons();
    void syncWithWidget();
    PointCloudWidget* findRobotWidget(const QString& robotName);
    
    // 색상 관리
    QHash<QString, QPushButton*> colorButtons_;
    
    // 상수들
    static const QStringList ROBOT_NAMES;
    static const QStringList ROBOT_COLORS;
};

} // namespace Widget

#endif // CONTROL_TREE_WIDGET_HPP