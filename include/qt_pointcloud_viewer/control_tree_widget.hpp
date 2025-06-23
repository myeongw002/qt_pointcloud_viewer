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
#include "interest_object_manager.hpp"
#include "common_types.hpp"

namespace Widget {

class ControlTreeWidget : public QTreeWidget {
    Q_OBJECT

public:
    explicit ControlTreeWidget(QWidget* parent = nullptr);
    
    void setRobotName(const QString& robotName);
    void setTargetWidget(PointCloudWidget* widget);
    void setMainWindow(QMainWindow* mainWindow);

private slots:
    void onItemChanged(QTreeWidgetItem* item, int column);
    void onInterestObjectRegistered(const QString& objectId, const QString& objectName);
    void onInterestObjectRemoved(const QString& objectId);

private:
    // ============================================================================
    // Core Members
    // ============================================================================
    QString robotName_;
    QTreeWidgetItem* viewGroup_;
    QTreeWidgetItem* robotGroup_;
    PointCloudWidget* targetWidget_;
    PointCloudWidget* pointCloudWidget_;  // 이 라인 추가
    QMainWindow* mainWindow_;
    // Point Size / Resolution 슬라이더 참조 추가
    QSlider* pointSizeSlider_ = nullptr;
    QLabel* pointSizeLabel_ = nullptr;
    QSlider* pathWidthSlider_ = nullptr;     // ← Path Width 슬라이더 참조 추가
    QLabel* pathWidthLabel_ = nullptr;       // ← Path Width 레이블 참조 추가
    QString currentMapStyle_ = "pointcloud";  // 현재 맵 스타일 추적
    // Interest Objects 관련 UI 요소들 (안전한 멤버 변수로 관리)
    QTreeWidgetItem* interestObjectsGroup_;
    QCheckBox* showInterestObjectsCheck_;
    QTreeWidgetItem* objectListGroup_;
    QPushButton* clearObjectsBtn_;
    // 새로 추가: 라벨들을 멤버 변수로 관리
    QLabel* objectCountLabel_;
    QLabel* serviceStatusLabel_;
    QComboBox* mapStyleCombo_ = nullptr;  // New combo box for map style selection
    QComboBox* targetRobotCombo_ = nullptr;  // New combo box for target robot selection
    // ============================================================================
    // Core Functions
    // ============================================================================
    void setupTreeStructure();
    void addViewerSettings(QTreeWidgetItem* parent);
    void addRobotControls(QTreeWidgetItem* parent);
    void syncWithWidget();
    void updateColorButtons();
    void setupInterestObjectsGroup();
    void addInterestObjectsControls(QTreeWidgetItem* parent);
    void clearAllObjects();
    QString getCurrentRobot() const;  // 추가
    
    // ============================================================================
    // Helper Functions
    // ============================================================================
    QCheckBox* createCheckBox(bool checked, std::function<void(bool)> callback);
    QPushButton* createButton(const QString& text, std::function<void()> callback);
    PointCloudWidget* findRobotWidget(const QString& robotName);
    QCheckBox* findCheckBoxInTree(const QString& itemName);
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