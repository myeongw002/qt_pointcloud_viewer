// control_tree_widget.cpp
#include "control_tree_widget.hpp"
#include "pointcloud_widget.hpp"  // 이 줄 추가!
#include "viewer_settings_manager.hpp"
#include "interest_object_manager.hpp"
#include <QApplication>
#include <QStyle>
#include <QDebug>
#include <QColorDialog>
#include <QSlider>
#include <QSpinBox>
#include <QComboBox>
#include <QCheckBox>
#include <QPushButton>
#include <QLabel>
#include <QHBoxLayout>
#include <QLineEdit>      // 추가
#include <QMessageBox>    // 추가
#include <QDateTime>      // 추가

namespace Widget {

// ============================================================================
// Constants
// ============================================================================
const QStringList ControlTreeWidget::ROBOT_NAMES = {"COMBINED", "TUGV", "MUGV", "SUGV1", "SUGV2", "SUAV"};
const QStringList ControlTreeWidget::ROBOT_COLORS = {"#888888", "#FF0000", "#00FF00", "#0000FF", "#FFFF00", "#FF00FF"};

// ============================================================================
// Constructor
// ============================================================================
ControlTreeWidget::ControlTreeWidget(QWidget* parent) 
    : QTreeWidget(parent), targetWidget_(nullptr), pointCloudWidget_(nullptr), mainWindow_(nullptr) {
    
    // Tree basic configuration
    setHeaderLabels({"Property", "Value"});
    setColumnWidth(0, 200);
    setColumnWidth(1, 150);
    setAlternatingRowColors(true);
    setRootIsDecorated(true);
    setIndentation(15);
    
    // Initial tree structure setup
    setupTreeStructure();
    
    // Signal connections
    connect(this, &QTreeWidget::itemChanged, this, &ControlTreeWidget::onItemChanged);
}

// ============================================================================
// Public Interface
// ============================================================================
void ControlTreeWidget::setRobotName(const QString& robotName) {
    robotName_ = robotName;
    clear();
    setupTreeStructure();
}

void ControlTreeWidget::setTargetWidget(PointCloudWidget* widget) {
    qDebug() << "Setting target widget for robot:" << robotName_;
    
    targetWidget_ = widget;
    pointCloudWidget_ = widget;  // Interest Objects용으로 동일한 위젯 설정
    
    if (targetWidget_) {
        qDebug() << "Target widget set successfully, starting synchronization...";
        
        // Interest Objects 시그널 연결 (위젯이 설정된 후)
        if (interestObjectsGroup_) {
            auto& manager = ObjectManager::InterestObjectManager::instance();
            connect(&manager, &ObjectManager::InterestObjectManager::interestObjectRegistered,
                    this, &ControlTreeWidget::onInterestObjectRegistered);
            connect(&manager, &ObjectManager::InterestObjectManager::interestObjectRemoved,
                    this, &ControlTreeWidget::onInterestObjectRemoved);
        }
        
        // 약간의 지연 후 동기화 (위젯이 완전히 초기화될 때까지 대기)
        QTimer::singleShot(100, [this]() {
            syncWithWidget();
        });
    } else {
        qDebug() << "Target widget set to nullptr";
        pointCloudWidget_ = nullptr;
    }
}

void ControlTreeWidget::setMainWindow(QMainWindow* mainWindow) {
    mainWindow_ = mainWindow;
}

// ============================================================================
// Core Tree Structure
// ============================================================================
void ControlTreeWidget::setupTreeStructure() {
    // Create main groups
    viewGroup_ = new QTreeWidgetItem(this, {"Viewer Settings"});
    robotGroup_ = new QTreeWidgetItem(this, {"Robot Controls"});
    
    // Interest Objects 그룹 추가
    setupInterestObjectsGroup();
    
    // Set group expansion state
    viewGroup_->setExpanded(true);
    robotGroup_->setExpanded(true);
    
    // Add controls to each group
    addViewerSettings(viewGroup_);
    addRobotControls(robotGroup_);
}

void ControlTreeWidget::addViewerSettings(QTreeWidgetItem* parent) {
    // ============================================================================
    // Show Robot Labels
    // ============================================================================
    auto labelItem = new QTreeWidgetItem(parent, {"Show Robot Labels"});
    auto labelCheck = createCheckBox(true, [this](bool checked) {
        if (targetWidget_) {
            targetWidget_->setShowRobotLabel(checked);
            ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
            qDebug() << "Robot labels:" << (checked ? "ON" : "OFF");
        }
    });
    setItemWidget(labelItem, 1, labelCheck);
    
    // ============================================================================
    // Axes Controls Group (닫힌 상태로 시작)
    // ============================================================================
    auto axesGroup = new QTreeWidgetItem(parent, {"Axes Controls"});
    axesGroup->setExpanded(false);  // ✅ 변경: true → false
    
    // Show Axes checkbox
    auto axesItem = new QTreeWidgetItem(axesGroup, {"Show Axes"});
    auto axesCheck = createCheckBox(true, [this](bool checked) {
        if (targetWidget_) {
            targetWidget_->setShowAxes(checked);
            ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
            qDebug() << "Axes display:" << (checked ? "ON" : "OFF");
        }
    });
    setItemWidget(axesItem, 1, axesCheck);
    
    // Axes Size slider
    auto axesSizeItem = new QTreeWidgetItem(axesGroup, {"Axes Size"});
    auto axesSizeSlider = new QSlider(Qt::Horizontal);
    axesSizeSlider->setRange(1, 50);
    axesSizeSlider->setValue(10);
    
    auto axesSizeWidget = new QWidget();
    auto axesSizeLayout = new QHBoxLayout(axesSizeWidget);
    auto axesSizeLabel = new QLabel("1.0m");
    axesSizeLabel->setMinimumWidth(40);
    axesSizeLabel->setAlignment(Qt::AlignCenter);
    
    connect(axesSizeSlider, &QSlider::valueChanged, [this, axesSizeLabel](int value) {
        if (targetWidget_) {
            float size = value / 10.0f;
            targetWidget_->setAxesSize(size);
            ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
            axesSizeLabel->setText(QString("%1m").arg(size, 0, 'f', 1));
            qDebug() << "Axes size:" << size << "meters";
        }
    });
    
    axesSizeLayout->addWidget(axesSizeSlider);
    axesSizeLayout->addWidget(axesSizeLabel);
    axesSizeLayout->setContentsMargins(0, 0, 0, 0);
    axesSizeLayout->setSpacing(5);
    setItemWidget(axesSizeItem, 1, axesSizeWidget);
    
    // ============================================================================
    // Grid Controls Group (닫힌 상태로 시작)
    // ============================================================================
    auto gridGroup = new QTreeWidgetItem(parent, {"Grid Controls"});
    gridGroup->setExpanded(false);  // ✅ 변경: true → false
    
    // Show Grid checkbox
    auto gridItem = new QTreeWidgetItem(gridGroup, {"Show Grid"});
    auto gridCheck = createCheckBox(true, [this](bool checked) {
        if (targetWidget_) {
            targetWidget_->setShowGrid(checked);
            ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
            qDebug() << "Grid display:" << (checked ? "ON" : "OFF");
        }
    });
    setItemWidget(gridItem, 1, gridCheck);
    
    // Grid Cell Count slider
    auto gridCountItem = new QTreeWidgetItem(gridGroup, {"Grid Cell Count"});
    auto gridCountSlider = new QSlider(Qt::Horizontal);
    gridCountSlider->setRange(4, 50);
    gridCountSlider->setValue(10);
    
    auto gridCountWidget = new QWidget();
    auto gridCountLayout = new QHBoxLayout(gridCountWidget);
    auto gridCountLabel = new QLabel("10x10");
    gridCountLabel->setMinimumWidth(50);
    gridCountLabel->setAlignment(Qt::AlignCenter);
    
    connect(gridCountSlider, &QSlider::valueChanged, [this, gridCountLabel](int value) {
        if (targetWidget_) {
            targetWidget_->setGridCellCount(value);
            ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
            gridCountLabel->setText(QString("%1x%1").arg(value));
            qDebug() << "Grid cell count:" << value << "x" << value;
        }
    });
    
    gridCountLayout->addWidget(gridCountSlider);
    gridCountLayout->addWidget(gridCountLabel);
    gridCountLayout->setContentsMargins(0, 0, 0, 0);
    gridCountLayout->setSpacing(5);
    setItemWidget(gridCountItem, 1, gridCountWidget);
    
    // Grid Cell Size slider
    auto gridSizeItem = new QTreeWidgetItem(gridGroup, {"Grid Cell Size"});
    auto gridSizeSlider = new QSlider(Qt::Horizontal);
    gridSizeSlider->setRange(1, 100);
    gridSizeSlider->setValue(10);
    
    auto gridSizeWidget = new QWidget();
    auto gridSizeLayout = new QHBoxLayout(gridSizeWidget);
    auto gridSizeLabel = new QLabel("1.0m");
    gridSizeLabel->setMinimumWidth(40);
    gridSizeLabel->setAlignment(Qt::AlignCenter);
    
    connect(gridSizeSlider, &QSlider::valueChanged, [this, gridSizeLabel](int value) {
        if (targetWidget_) {
            float size = value / 10.0f;
            targetWidget_->setGridSize(size);
            ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
            gridSizeLabel->setText(QString("%1m").arg(size, 0, 'f', 1));
            qDebug() << "Grid cell size:" << size << "meters";
        }
    });
    
    gridSizeLayout->addWidget(gridSizeSlider);
    gridSizeLayout->addWidget(gridSizeLabel);
    gridSizeLayout->setContentsMargins(0, 0, 0, 0);
    gridSizeLayout->setSpacing(5);
    setItemWidget(gridSizeItem, 1, gridSizeWidget);
    
    // ============================================================================
    // Map Styling Group (확장된 상태 유지)
    // ============================================================================
    auto mapStylingGroup = new QTreeWidgetItem(parent, {"Map Styling"});
    mapStylingGroup->setExpanded(true);  // Map Styling은 열린 상태 유지
    
    // ============================================================================
    // 1. Map Display Settings (맵 표시 관련)
    // ============================================================================

    // Map Style 콤보박스
    auto mapStyleItem = new QTreeWidgetItem(mapStylingGroup, {"Map Style"});
    auto mapStyleCombo = new QComboBox();
    mapStyleCombo->addItems({"PointCloud", "GridMap"});
    mapStyleCombo->setCurrentText("PointCloud");
    connect(mapStyleCombo, QOverload<const QString&>::of(&QComboBox::currentTextChanged),
            [this, mapStyleCombo](const QString& style) {
                if (!targetWidget_) return;
                
                qDebug() << "Map style changing to:" << style;
                
                // 멤버 변수 업데이트
                currentMapStyle_ = style.toLower();
                
                if (style == "pointcloud") {
                    targetWidget_->setMapStyle("pointcloud");
                    
                    // Point Size 모드로 슬라이더 레이블 업데이트
                    if (pointSizeSlider_ && pointSizeLabel_) {
                        int currentValue = pointSizeSlider_->value();
                        float pointSize = currentValue / 10.0f;
                        pointSizeLabel_->setText(QString("%1").arg(pointSize, 0, 'f', 1));
                    }
                    
                } else if (style == "gridmap") {
                    targetWidget_->setMapStyle("gridmap");
                    
                    // Resolution 모드로 슬라이더 레이블 업데이트
                    if (pointSizeSlider_ && pointSizeLabel_) {
                        int currentValue = pointSizeSlider_->value();
                        float resolution = 0.01f + (currentValue - 5) * (1.0f - 0.01f) / (100 - 5);
                        resolution = std::round(resolution * 100.0f) / 100.0f;
                        pointSizeLabel_->setText(QString("%1m").arg(resolution, 0, 'f', 2));
                        
                        // 즉시 해상도 적용
                        targetWidget_->setGridMapResolution(resolution);
                    }
                }
                
                // Show Map 체크박스 업데이트
                auto showMapCheck = findCheckBoxInTree("Show Map");
                if (showMapCheck) {
                    showMapCheck->setChecked(true);
                    showMapCheck->setEnabled(true);
                }
                
                ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
                qDebug() << "Map style changed to:" << style;
            });
    setItemWidget(mapStyleItem, 1, mapStyleCombo);
    mapStyleCombo_ = mapStyleCombo;  // 멤버 변수로 저장 (동기화용)

    // Show Map checkbox
    auto showMapItem = new QTreeWidgetItem(mapStylingGroup, {"Show Map"});
    auto showMapCheck = createCheckBox(true, [this](bool checked) {
        if (targetWidget_) {
            QString currentStyle = targetWidget_->getMapStyle();
            
            if (currentStyle == "pointcloud") {
                // PointCloud 모드: Show Points 제어
                targetWidget_->setShowPoints(checked);
                qDebug() << "Points display:" << (checked ? "ON" : "OFF");
            } else if (currentStyle == "gridmap") {
                // GridMap 모드: Show GridMap 제어
                targetWidget_->setShowGridMap(checked);
                qDebug() << "GridMap display:" << (checked ? "ON" : "OFF");
            }
            
            ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
        }
    });
    setItemWidget(showMapItem, 1, showMapCheck);

    // Show Path checkbox
    auto showPathItem = new QTreeWidgetItem(mapStylingGroup, {"Show Path"});
    auto showPathCheck = createCheckBox(true, [this](bool checked) {
        if (targetWidget_) {
            targetWidget_->setShowPath(checked);
            ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
            qDebug() << "Path display:" << (checked ? "ON" : "OFF");
        }
    });
    setItemWidget(showPathItem, 1, showPathCheck);

    // ============================================================================
    // 2. Size Settings (크기 관련 설정)
    // ============================================================================

    // Point Size / Resolution slider
    auto pointSizeItem = new QTreeWidgetItem(mapStylingGroup, {"Point Size / Resolution"});
    auto pointSizeSlider = new QSlider(Qt::Horizontal);
    pointSizeSlider->setRange(5, 100);
    pointSizeSlider->setValue(20);

    // 값 표시 레이블 추가
    auto pointSizeWidget = new QWidget();
    auto pointSizeLayout = new QHBoxLayout(pointSizeWidget);
    auto pointSizeLabel = new QLabel("2.0");
    pointSizeLabel->setMinimumWidth(50);
    pointSizeLabel->setAlignment(Qt::AlignCenter);

    connect(pointSizeSlider, &QSlider::valueChanged, [this, pointSizeLabel](int value) {
        if (targetWidget_) {
            QString currentStyle = targetWidget_->getMapStyle();
            
            if (currentStyle == "pointcloud") {
                // PointCloud 모드: Point Size 제어 (기존 동작)
                float size = value / 10.0f;  // 0.5 ~ 10.0
                targetWidget_->setPointSize(size);
                pointSizeLabel->setText(QString("%1").arg(size, 0, 'f', 1));
                qDebug() << "Point size:" << size;
                
            } else if (currentStyle == "gridmap") {
                // GridMap 모드: Resolution 제어 (새로운 기능)
                // 5~100 → 0.01~1.0 매핑
                float resolution = 0.01f + (value - 5) * (1.0f - 0.01f) / (100 - 5);
                resolution = std::round(resolution * 100.0f) / 100.0f;  // 소수점 2자리 반올림
                
                targetWidget_->setGridMapResolution(resolution);
                pointSizeLabel->setText(QString("%1m").arg(resolution, 0, 'f', 2));
                qDebug() << "GridMap resolution:" << resolution << "meters";
            }
            
            ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
        }
    });

    pointSizeLayout->addWidget(pointSizeSlider);
    pointSizeLayout->addWidget(pointSizeLabel);
    pointSizeLayout->setContentsMargins(0, 0, 0, 0);
    pointSizeLayout->setSpacing(5);
    setItemWidget(pointSizeItem, 1, pointSizeWidget);

    // 슬라이더 참조 저장 (동기화용)
    pointSizeSlider_ = pointSizeSlider;
    pointSizeLabel_ = pointSizeLabel;

    // Path Width slider
    auto pathWidthItem = new QTreeWidgetItem(mapStylingGroup, {"Path Width"});

    // 값 표시 레이블과 함께 위젯 생성
    auto pathWidthWidget = new QWidget();
    auto pathWidthLayout = new QHBoxLayout(pathWidthWidget);
    auto pathWidthSlider = new QSlider(Qt::Horizontal);
    auto pathWidthLabel = new QLabel("3.0");

    pathWidthSlider->setRange(5, 100);  // 0.5 ~ 10.0 범위
    pathWidthSlider->setValue(30);      // 기본값 3.0
    pathWidthLabel->setMinimumWidth(50);
    pathWidthLabel->setAlignment(Qt::AlignCenter);

    connect(pathWidthSlider, &QSlider::valueChanged, [this, pathWidthLabel](int value) {
        if (targetWidget_) {
            float width = value / 10.0f;  // 5~100 → 0.5~10.0
            targetWidget_->setPathWidth(width);
            pathWidthLabel->setText(QString("%1").arg(width, 0, 'f', 1));
            ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
            qDebug() << "Path width:" << width;
        }
    });

    pathWidthLayout->addWidget(pathWidthSlider);
    pathWidthLayout->addWidget(pathWidthLabel);
    pathWidthLayout->setContentsMargins(0, 0, 0, 0);
    pathWidthLayout->setSpacing(5);
    setItemWidget(pathWidthItem, 1, pathWidthWidget);

    // 멤버 변수로 저장 (동기화용)
    pathWidthSlider_ = pathWidthSlider;
    pathWidthLabel_ = pathWidthLabel;

    // ============================================================================
    // 3. Position Marker Settings (위치 마커 관련)
    // ============================================================================

    // Show Current Position
    auto positionItem = new QTreeWidgetItem(mapStylingGroup, {"Show Position"});
    auto positionCheck = createCheckBox(true, [this](bool checked) {
        if (targetWidget_) {
            targetWidget_->setShowPosition(checked);
            ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
            qDebug() << "Position display:" << (checked ? "ON" : "OFF");
        }
    });
    setItemWidget(positionItem, 1, positionCheck);

    // Show Position Names checkbox
    auto positionNamesItem = new QTreeWidgetItem(mapStylingGroup, {"Show Names"});
    auto positionNamesCheck = createCheckBox(true, [this](bool checked) {
        if (targetWidget_) {
            targetWidget_->setShowPositionNames(checked);
            ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
            qDebug() << "Position names:" << (checked ? "ON" : "OFF");
        }
    });
    setItemWidget(positionNamesItem, 1, positionNamesCheck);

    // Position Marker Type
    auto markerTypeItem = new QTreeWidgetItem(mapStylingGroup, {"Marker Type"});
    auto markerTypeCombo = new QComboBox();
    markerTypeCombo->addItems({"Cylinder", "Axes"});
    markerTypeCombo->setCurrentIndex(1);
    connect(markerTypeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            [this, markerTypeCombo](int index) {
                if (!targetWidget_) return;
                
                // RenderHelper 타입으로 변환
                RenderHelper::PositionMarkerType type = 
                    (index == 0) ? RenderHelper::PositionMarkerType::CYLINDER 
                                 : RenderHelper::PositionMarkerType::AXES;
                
                targetWidget_->setPositionMarkerType(type);
                qDebug() << "Marker type changed to:" << markerTypeCombo->currentText();
            });
    setItemWidget(markerTypeItem, 1, markerTypeCombo);

    // Marker Size slider
    auto markerSizeItem = new QTreeWidgetItem(mapStylingGroup, {"Marker Size"});
    auto markerSizeSlider = new QSlider(Qt::Horizontal);
    markerSizeSlider->setRange(5, 100);  // 0.05 ~ 1.0 범위로 확장
    markerSizeSlider->setValue(30);      // 기본값 0.3

    auto markerSizeWidget = new QWidget();
    auto markerSizeLayout = new QHBoxLayout(markerSizeWidget);
    auto markerSizeLabel = new QLabel("0.30");
    markerSizeLabel->setMinimumWidth(50);
    markerSizeLabel->setAlignment(Qt::AlignCenter);

    connect(markerSizeSlider, &QSlider::valueChanged, [this, markerSizeLabel](int value) {
        if (targetWidget_) {
            float radius = value / 100.0f;  // 0.05 ~ 1.0 범위
            targetWidget_->setPositionRadius(radius);
            ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
            markerSizeLabel->setText(QString("%1").arg(radius, 0, 'f', 2));
            qDebug() << "Marker size:" << radius;
        }
    });

    markerSizeLayout->addWidget(markerSizeSlider);
    markerSizeLayout->addWidget(markerSizeLabel);
    markerSizeLayout->setContentsMargins(0, 0, 0, 0);
    markerSizeLayout->setSpacing(5);
    setItemWidget(markerSizeItem, 1, markerSizeWidget);

    // ============================================================================
    // 4. Robot Colors Section (닫힌 상태로 시작)
    // ============================================================================
    auto colorsGroup = new QTreeWidgetItem(mapStylingGroup, {"Robot Colors"});
    colorsGroup->setExpanded(false);  // ✅ 변경: true → false

    for (int i = 1; i < ROBOT_NAMES.size(); ++i) {
        const QString& robot = ROBOT_NAMES[i];
        
        auto robotColorGroup = new QTreeWidgetItem(colorsGroup, {robot});
        
        // Point color button
        auto pointColorItem = new QTreeWidgetItem(robotColorGroup, {"Points"});
        auto pointColorBtn = createButton("", [this, robot]() {
            qDebug() << "Opening color dialog for" << robot << "points...";
            
            QColor currentColor;
            if (targetWidget_) {
                auto glmColor = targetWidget_->getRobotPointsColor(robot);
                currentColor = QColor(static_cast<int>(glmColor.r * 255), 
                                    static_cast<int>(glmColor.g * 255), 
                                    static_cast<int>(glmColor.b * 255));
                qDebug() << "Current points color for" << robot << ":" << currentColor.name();
            }
            
            QColor newColor = QColorDialog::getColor(currentColor);
            if (newColor.isValid() && targetWidget_) {
                glm::vec3 glmColor(newColor.redF(), newColor.greenF(), newColor.blueF());
                targetWidget_->setRobotPointsColor(robot, glmColor);
                ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
                
                // 버튼 색상 즉시 업데이트
                auto btn = qobject_cast<QPushButton*>(sender());
                if (btn) {
                    QString styleSheet = QString("background-color: %1; border: 2px solid white;")
                                       .arg(newColor.name());
                    btn->setStyleSheet(styleSheet);
                    qDebug() << "Points button style updated to:" << styleSheet;
                }
                
                qDebug() << robot << "points color changed to:" << newColor.name();
                
                // 전체 컬러 버튼 업데이트 (다른 로봇들도 영향받을 수 있음)
                QTimer::singleShot(100, [this]() {
                    updateColorButtons();
                });
            } else {
                qDebug() << "Color selection cancelled or no target widget";
            }
        });
        pointColorBtn->setFixedSize(40, 25);
        
        // 초기 색상은 기본값으로 설정 (나중에 syncWithWidget에서 업데이트)
        const QString& defaultColorHex = ROBOT_COLORS[i];
        pointColorBtn->setStyleSheet(QString("background-color: %1; border: 2px solid white;")
                                   .arg(defaultColorHex));
        
        setItemWidget(pointColorItem, 1, pointColorBtn);
        colorButtons_[robot + "_points"] = pointColorBtn;
        
        qDebug() << "Created points color button for" << robot << "with default color" << defaultColorHex;
        
        // Path color button
        auto pathColorItem = new QTreeWidgetItem(robotColorGroup, {"Path"});
        auto pathColorBtn = createButton("", [this, robot]() {
            qDebug() << "Opening color dialog for" << robot << "path...";
            
            QColor currentColor;
            if (targetWidget_) {
                auto glmColor = targetWidget_->getRobotPathColor(robot);
                currentColor = QColor(static_cast<int>(glmColor.r * 255), 
                                    static_cast<int>(glmColor.g * 255), 
                                    static_cast<int>(glmColor.b * 255));
                qDebug() << "Current path color for" << robot << ":" << currentColor.name();
            }
            
            QColor newColor = QColorDialog::getColor(currentColor);
            if (newColor.isValid() && targetWidget_) {
                glm::vec3 glmColor(newColor.redF(), newColor.greenF(), newColor.blueF());
                targetWidget_->setRobotPathColor(robot, glmColor);
                ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
                
                // 버튼 색상 즉시 업데이트
                auto btn = qobject_cast<QPushButton*>(sender());
                if (btn) {
                    QString styleSheet = QString("background-color: %1; border: 2px solid white;")
                                       .arg(newColor.name());
                    btn->setStyleSheet(styleSheet);
                    qDebug() << "Path button style updated to:" << styleSheet;
                }
                
                qDebug() << robot << "path color changed to:" << newColor.name();
                
                // 전체 컬러 버튼 업데이트
                QTimer::singleShot(100, [this]() {
                    updateColorButtons();
                });
            } else {
                qDebug() << "Color selection cancelled or no target widget";
            }
        });
        pathColorBtn->setFixedSize(40, 25);
        
        // 초기 경로 색상은 포인트 색상의 밝은 버전으로 설정
        QColor defaultPathColor = QColor(defaultColorHex).lighter(150);
        pathColorBtn->setStyleSheet(QString("background-color: %1; border: 2px solid white;")
                                  .arg(defaultPathColor.name()));
        
        setItemWidget(pathColorItem, 1, pathColorBtn);
        colorButtons_[robot + "_path"] = pathColorBtn;
        
        qDebug() << "Created path color button for" << robot << "with default color" << defaultPathColor.name();
    }

    // ✅ Reset All Colors button을 Robot Colors 그룹 안으로 이동
    auto resetColorsItem = new QTreeWidgetItem(colorsGroup, {"Reset All Colors"});
    auto resetBtn = createButton("Reset", [this]() {
        if (targetWidget_) {
            qDebug() << "Resetting all colors to default...";
            targetWidget_->resetAllColorsToDefault();
            ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
            
            // 색상 리셋 후 버튼 업데이트
            QTimer::singleShot(200, [this]() {
                updateColorButtons();
            });
            
            qDebug() << "Colors reset to default";
        }
    });
    setItemWidget(resetColorsItem, 1, resetBtn);
}

void ControlTreeWidget::addRobotControls(QTreeWidgetItem* parent) {
    // ============================================================================
    // Robot Navigation Section
    // ============================================================================
    
    // Lock to Robot Position
    auto lockIndicatorItem = new QTreeWidgetItem(parent, {"Lock to Robot Position"});
    auto lockCheck = createCheckBox(false, [this](bool checked) {
        if (targetWidget_) {
            targetWidget_->setLockIndicatorToCurrentPosition(checked);
            qDebug() << "Lock indicator:" << (checked ? "ON" : "OFF");
        }
    });
    setItemWidget(lockIndicatorItem, 1, lockCheck);
    
    // Target Robot selection
    auto targetRobotItem = new QTreeWidgetItem(parent, {"Target Robot"});
    targetRobotCombo_ = new QComboBox();
    QStringList realRobots = {"TUGV", "MUGV", "SUGV1", "SUGV2", "SUAV"};
    targetRobotCombo_->addItems(realRobots);
    targetRobotCombo_->setCurrentText("TUGV");
    connect(targetRobotCombo_, QOverload<const QString&>::of(&QComboBox::currentTextChanged), 
            [this](const QString& robot) {
        qDebug() << "Target robot changed to:" << robot;
        
        if (robotName_ == "COMBINED") {
            auto* robotWidget = findRobotWidget(robot);
            if (robotWidget) {
                robotWidget->setIndicatorTargetRobot(robot);
            }
            if (targetWidget_) {
                targetWidget_->setIndicatorTargetRobot(robot);
            }
        } else if (targetWidget_) {
            targetWidget_->setIndicatorTargetRobot(robot);
        }
    });
    setItemWidget(targetRobotItem, 1, targetRobotCombo_);
    
    // ============================================================================
    // Quick Jump Section
    // ============================================================================
    auto jumpGroup = new QTreeWidgetItem(parent, {"Quick Jump"});
    
    for (const QString& robot : realRobots) {
        auto jumpItem = new QTreeWidgetItem(jumpGroup, {"Jump to " + robot});
        auto jumpBtn = createButton("Go", [this, robot]() {
            qDebug() << "Quick jump to" << robot << "initiated...";
            
            PointCloudWidget* activeWidget = nullptr;
            
            if (robotName_ == "COMBINED") {
                activeWidget = findRobotWidget(robot);
                if (!activeWidget) {
                    qDebug() << "Could not find individual widget for" << robot;
                    return;
                }
            } else {
                activeWidget = targetWidget_;
                if (!activeWidget) {
                    qDebug() << "No target widget for quick jump!";
                    return;
                }
            }
            
            // Set target and jump
            activeWidget->setIndicatorTargetRobot(robot);
            if (robotName_ == "COMBINED" && targetWidget_) {
                targetWidget_->setIndicatorTargetRobot(robot);
                glm::vec3 robotPosition = activeWidget->getRobotCurrentPosition(robot);
                targetWidget_->jumpToPosition(robotPosition);
            } else {
                activeWidget->jumpToRobotPosition(robot);
            }
            
            // Enable lock
            activeWidget->setLockIndicatorToCurrentPosition(true);
            if (robotName_ == "COMBINED" && targetWidget_) {
                targetWidget_->setLockIndicatorToCurrentPosition(true);
            }
            
            // Update combo box
            if (targetRobotCombo_ && targetRobotCombo_->currentText() != robot) {
                targetRobotCombo_->setCurrentText(robot);
            }
            
            // Auto-release after 3 seconds
            QTimer* autoReleaseTimer = new QTimer();
            autoReleaseTimer->setSingleShot(true);
            connect(autoReleaseTimer, &QTimer::timeout, [this, autoReleaseTimer, robot, activeWidget]() {
                if (activeWidget) {
                    activeWidget->setLockIndicatorToCurrentPosition(false);
                }
                if (robotName_ == "COMBINED" && targetWidget_) {
                    targetWidget_->setLockIndicatorToCurrentPosition(false);
                }
                autoReleaseTimer->deleteLater();
            });
            autoReleaseTimer->start(3000);
            
            qDebug() << "Quick jump to" << robot << "completed!";
        });
        setItemWidget(jumpItem, 1, jumpBtn);
    }
}

void ControlTreeWidget::syncWithWidget() {
    if (!targetWidget_) {
        qDebug() << "syncWithWidget: No target widget available";
        return;
    }
    
    qDebug() << "syncWithWidget: Starting synchronization for robot:" << robotName_;
    
    // Helper to find checkboxes in tree
    auto findCheckBoxInTree = [this](const QString& itemName) -> QCheckBox* {
        QTreeWidgetItemIterator it(this);
        while (*it) {
            if ((*it)->text(0) == itemName) {
                QWidget* widget = itemWidget(*it, 1);
                return qobject_cast<QCheckBox*>(widget);
            }
            ++it;
        }
        return nullptr;
    };
    
    // Sync checkboxes
    if (auto* showAxesCheck = findCheckBoxInTree("Show Axes")) {
        showAxesCheck->setChecked(targetWidget_->getShowAxes());
    }
    
    if (auto* showGridCheck = findCheckBoxInTree("Show Grid")) {
        showGridCheck->setChecked(targetWidget_->getShowGrid());
    }
    
    // Show Map 체크박스 동기화 (Map Style에 따라 다른 동작)
    if (auto* showMapCheck = findCheckBoxInTree("Show Map")) {
        QString currentStyle = targetWidget_->getMapStyle();
        
        if (currentStyle == "pointcloud") {
            // PointCloud 모드: Show Points 상태 반영
            showMapCheck->setChecked(targetWidget_->getShowPoints());
        } else if (currentStyle == "gridmap") {
            // GridMap 모드: Show GridMap 상태 반영
            showMapCheck->setChecked(targetWidget_->getShowGridMap());
        }
        
        // 모든 모드에서 활성화 상태 유지
        showMapCheck->setEnabled(true);
    }
    
    // Show Path는 항상 동기화 (모든 모드에서 사용 가능)
    if (auto* showPathCheck = findCheckBoxInTree("Show Path")) {
        showPathCheck->setChecked(targetWidget_->getShowPath());
    }
    
    if (auto* showPositionCheck = findCheckBoxInTree("Show Current Position")) {
        showPositionCheck->setChecked(targetWidget_->getShowPosition());
    }
    
    if (auto* showLabelCheck = findCheckBoxInTree("Show Robot Labels")) {
        showLabelCheck->setChecked(targetWidget_->getShowRobotLabel());
    }
    
    // Sync Map Style combo box
    if (mapStyleCombo_) {
        QString currentStyle = targetWidget_->getMapStyle();
        if (currentStyle == "pointcloud") {
            mapStyleCombo_->setCurrentText("PointCloud");
        } else if (currentStyle == "gridmap") {
            mapStyleCombo_->setCurrentText("GridMap");
        }
        
        // Show Map 체크박스는 항상 활성화 (제거된 로직)
        // GridMap 모드에서도 Path 제어가 가능하므로 비활성화하지 않음
    }
    
    // Path Width 슬라이더 동기화
    if (pathWidthSlider_ && pathWidthLabel_) {
        float currentPathWidth = targetWidget_->getPathWidth();
        int sliderValue = static_cast<int>(currentPathWidth * 10);
        pathWidthSlider_->setValue(std::clamp(sliderValue, 5, 100));
        pathWidthLabel_->setText(QString("%1").arg(currentPathWidth, 0, 'f', 1));
        qDebug() << "Synced Path Width:" << currentPathWidth;
    }
    
    // Point Size / Resolution 슬라이더 동기화
    if (pointSizeSlider_ && pointSizeLabel_) {
        QString currentStyle = targetWidget_->getMapStyle();
        
        if (currentStyle == "pointcloud") {
            float currentPointSize = targetWidget_->getPointSize();
            int sliderValue = static_cast<int>(currentPointSize * 10);
            pointSizeSlider_->setValue(std::clamp(sliderValue, 5, 100));
            pointSizeLabel_->setText(QString("%1").arg(currentPointSize, 0, 'f', 1));
            
        } else if (currentStyle == "gridmap") {
            float currentResolution = targetWidget_->getGridMapResolution();
            int sliderValue = static_cast<int>(5 + (currentResolution - 0.01f) * (100 - 5) / (1.0f - 0.01f));
            pointSizeSlider_->setValue(std::clamp(sliderValue, 5, 100));
            pointSizeLabel_->setText(QString("%1m").arg(currentResolution, 0, 'f', 2));
        }
    }
    
    // Sync color buttons
    qDebug() << "syncWithWidget: Starting color button synchronization...";
    updateColorButtons();
    
    qDebug() << "syncWithWidget: Synchronization completed for robot:" << robotName_;
}

void ControlTreeWidget::updateColorButtons() {
    if (!targetWidget_) {
        qDebug() << "No target widget for color button update";
        return;
    }
    
    qDebug() << "Updating color buttons for robot:" << robotName_;
    
    int updatedCount = 0;
    for (const QString& robot : ROBOT_NAMES) {
        if (robot == "COMBINED") continue;
        
        try {
            auto pointsColor = targetWidget_->getRobotPointsColor(robot);
            auto pathColor = targetWidget_->getRobotPathColor(robot);
            
            QString pointsKey = robot + "_points";
            QString pathKey = robot + "_path";
            
            // qDebug() << "Processing colors for" << robot 
            //          << "- Points:" << pointsColor.r << pointsColor.g << pointsColor.b
            //          << "- Path:" << pathColor.r << pathColor.g << pathColor.b;
            
            // Update points color button
            if (colorButtons_.contains(pointsKey)) {
                QColor qColor(static_cast<int>(pointsColor.r * 255), 
                             static_cast<int>(pointsColor.g * 255), 
                             static_cast<int>(pointsColor.b * 255));
                
                QString styleSheet = QString("background-color: %1; border: 2px solid white;")
                                   .arg(qColor.name());
                colorButtons_[pointsKey]->setStyleSheet(styleSheet);
                
                // qDebug() << robot << "points button updated to:" << qColor.name();
                updatedCount++;
            } else {
                qDebug() << "Points button not found for" << robot;
            }
            
            // Update path color button
            if (colorButtons_.contains(pathKey)) {
                QColor qColor(static_cast<int>(pathColor.r * 255), 
                             static_cast<int>(pathColor.g * 255), 
                             static_cast<int>(pathColor.b * 255));
                
                QString styleSheet = QString("background-color: %1; border: 2px solid white;")
                                   .arg(qColor.name());
                colorButtons_[pathKey]->setStyleSheet(styleSheet);
                
                // qDebug() << robot << "path button updated to:" << qColor.name();
                updatedCount++;
            } else {
                qDebug() << "Path button not found for" << robot;
            }
            
        } catch (const std::exception& e) {
            qDebug() << "Error updating colors for" << robot << ":" << e.what();
        }
    }
    
    qDebug() << "Color buttons update completed. Updated" << updatedCount << "buttons.";
    
    // 컬러 버튼 맵 상태 디버깅
    // qDebug() << "Current color buttons in map:";
    // for (auto it = colorButtons_.begin(); it != colorButtons_.end(); ++it) {
    //     qDebug() << "  -" << it.key() << ":" << (it.value() ? "valid" : "null");
    // }
}

// ============================================================================
// Helper Functions
// ============================================================================
PointCloudWidget* ControlTreeWidget::findRobotWidget(const QString& robotName) {
    if (!mainWindow_) {
        qDebug() << "MainWindow reference not set!";
        return nullptr;
    }
    
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
        auto* widget = qobject_cast<PointCloudWidget*>(
            mainWindow_->findChild<QWidget*>(QString("openGLWidget_%1").arg(widgetIndex))
        );
        
        if (widget) {
            qDebug() << "Found widget for" << robotName << "at openGLWidget_" << widgetIndex;
            return widget;
        } else {
            qDebug() << "Could not find openGLWidget_" << widgetIndex << "for" << robotName;
        }
    } else {
        qDebug() << "Invalid robot name:" << robotName;
    }
    
    return nullptr;
}

void ControlTreeWidget::onItemChanged(QTreeWidgetItem* item, int column) {
    // Handle any item changes if needed
}

QCheckBox* ControlTreeWidget::createCheckBox(bool checked, std::function<void(bool)> callback) {
    auto checkBox = new QCheckBox();
    checkBox->setChecked(checked);
    connect(checkBox, &QCheckBox::toggled, callback);
    return checkBox;
}

QPushButton* ControlTreeWidget::createButton(const QString& text, std::function<void()> callback) {
    auto button = new QPushButton(text);
    connect(button, &QPushButton::clicked, callback);
    return button;
}

// ============================================================================
// Helper Functions (추가)
// ============================================================================
QCheckBox* ControlTreeWidget::findCheckBoxInTree(const QString& itemName) {
    QTreeWidgetItemIterator it(this);
    while (*it) {
        if ((*it)->text(0) == itemName) {
            QWidget* widget = itemWidget(*it, 1);
            return qobject_cast<QCheckBox*>(widget);
        }
        ++it;
    }
    return nullptr;
}

void ControlTreeWidget::setupInterestObjectsGroup() {
    interestObjectsGroup_ = new QTreeWidgetItem(this);
    interestObjectsGroup_->setText(0, "Interest Objects");
    interestObjectsGroup_->setExpanded(true);
    
    addInterestObjectsControls(interestObjectsGroup_);
    
    // 전역 매니저와 시그널 연결
    auto& manager = ObjectManager::InterestObjectManager::instance();
    connect(&manager, &ObjectManager::InterestObjectManager::interestObjectRegistered,
            this, &ControlTreeWidget::onInterestObjectRegistered);
    connect(&manager, &ObjectManager::InterestObjectManager::interestObjectRemoved,
            this, &ControlTreeWidget::onInterestObjectRemoved);
}

void ControlTreeWidget::addInterestObjectsControls(QTreeWidgetItem* parent) {
    // ============================================================================
    // Interest Objects 표시 제어만 유지
    // ============================================================================
    
    // Show Interest Objects checkbox
    auto showObjectsItem = new QTreeWidgetItem(parent, {"Show Interest Objects"});
    showInterestObjectsCheck_ = createCheckBox(true, [this](bool checked) {
        if (pointCloudWidget_) {
            pointCloudWidget_->setShowObjects(checked);
            qDebug() << "Show Interest Objects set to:" << checked;
            
            // 전역 매니저에도 설정
            auto& manager = ObjectManager::InterestObjectManager::instance();
            manager.setShowInterestObjects(checked);
        }
    });
    setItemWidget(showObjectsItem, 1, showInterestObjectsCheck_);
    
    // Show Object Labels checkbox
    auto showLabelsItem = new QTreeWidgetItem(parent, {"Show Object Labels"});
    auto showLabelsCheck = createCheckBox(true, [this](bool checked) {
        if (pointCloudWidget_) {
            pointCloudWidget_->setShowObjectLabels(checked);
            qDebug() << "Show Interest Object Labels set to:" << checked;
        }
    });
    setItemWidget(showLabelsItem, 1, showLabelsCheck);
    
    // ============================================================================
    // 서비스 상태 표시 (읽기 전용) - 안전한 구현
    // ============================================================================
    
    // 현재 등록된 객체 수 표시
    auto objectCountItem = new QTreeWidgetItem(parent, {"Registered Objects"});
    objectCountLabel_ = new QLabel("0 objects");  // 멤버 변수로 저장
    objectCountLabel_->setAlignment(Qt::AlignCenter);
    objectCountLabel_->setStyleSheet("color: #00aa00; font-weight: bold;");
    setItemWidget(objectCountItem, 1, objectCountLabel_);
    
    // 서비스 상태 표시
    auto serviceStatusItem = new QTreeWidgetItem(parent, {"Service Status"});
    serviceStatusLabel_ = new QLabel("Ready");  // 멤버 변수로 저장
    serviceStatusLabel_->setAlignment(Qt::AlignCenter);
    serviceStatusLabel_->setStyleSheet("color: #00aa00; font-weight: bold;");
    setItemWidget(serviceStatusItem, 1, serviceStatusLabel_);
    
    // Clear All Objects 버튼은 유지 (관리용)
    auto clearItem = new QTreeWidgetItem(parent, {"Clear All Objects"});
    clearObjectsBtn_ = createButton("Clear All", [this]() {
        clearAllObjects();
    });
    setItemWidget(clearItem, 1, clearObjectsBtn_);
    
    // ============================================================================
    // Object List 표시 (서비스로 등록된 객체들)
    // ============================================================================
    objectListGroup_ = new QTreeWidgetItem(parent, {"Received Objects"});
    objectListGroup_->setExpanded(false);
    
    // ============================================================================
    // 전역 매니저 시그널 연결 - 안전한 구현
    // ============================================================================
    auto& manager = ObjectManager::InterestObjectManager::instance();
    
    // 기존 연결 해제 (중복 방지)
    disconnect(&manager, nullptr, this, nullptr);
    
    // 새로운 연결 설정 (this 포인터 유효성 확인)
    connect(&manager, &ObjectManager::InterestObjectManager::interestObjectRegistered,
            this, &ControlTreeWidget::onInterestObjectRegistered,
            Qt::QueuedConnection);  // 큐 연결로 안전성 증대
    
    connect(&manager, &ObjectManager::InterestObjectManager::interestObjectRemoved,
            this, &ControlTreeWidget::onInterestObjectRemoved,
            Qt::QueuedConnection);
    
    // 객체 수 업데이트 연결 - 안전한 구현
    connect(&manager, &ObjectManager::InterestObjectManager::interestObjectRegistered,
            this, [this](const QString&, const QString&) {
        // this 포인터와 라벨 유효성 확인
        if (this && objectCountLabel_) {
            auto& mgr = ObjectManager::InterestObjectManager::instance();
            int count = mgr.getAllObjectIds().size();
            objectCountLabel_->setText(QString("%1 objects").arg(count));
        }
    }, Qt::QueuedConnection);
    
    connect(&manager, &ObjectManager::InterestObjectManager::interestObjectRemoved,
            this, [this](const QString&) {
        // this 포인터와 라벨 유효성 확인
        if (this && objectCountLabel_) {
            auto& mgr = ObjectManager::InterestObjectManager::instance();
            int count = mgr.getAllObjectIds().size();
            objectCountLabel_->setText(QString("%1 objects").arg(count));
        }
    }, Qt::QueuedConnection);
    
    // 초기 객체 수 설정
    if (objectCountLabel_) {
        int initialCount = manager.getAllObjectIds().size();
        objectCountLabel_->setText(QString("%1 objects").arg(initialCount));
    }
}

// 안전한 객체 등록 핸들러
void ControlTreeWidget::onInterestObjectRegistered(const QString& objectId, const QString& objectType) {
    // this와 objectListGroup_ 유효성 확인
    if (!this || !objectListGroup_) {
        qDebug() << "ControlTreeWidget or objectListGroup_ is null in onInterestObjectRegistered";
        return;
    }
    
    QString displayText = QString("%1 (%2)").arg(objectType, objectId);
    auto objectItem = new QTreeWidgetItem(objectListGroup_, {displayText});
    
    // 제거 버튼 추가 - 안전한 구현
    auto removeBtn = createButton("Remove", [this, objectId]() {
        // 람다 실행 시점의 this 유효성 확인
        if (this) {
            auto& manager = ObjectManager::InterestObjectManager::instance();
            manager.removeInterestObject(objectId);
        }
    });
    setItemWidget(objectItem, 1, removeBtn);
    
    qDebug() << "Added object to UI list:" << displayText;
}

// 안전한 객체 제거 핸들러
void ControlTreeWidget::onInterestObjectRemoved(const QString& objectId) {
    // this와 objectListGroup_ 유효성 확인
    if (!this || !objectListGroup_) {
        qDebug() << "ControlTreeWidget or objectListGroup_ is null in onInterestObjectRemoved";
        return;
    }
    
    // 해당 객체 항목을 찾아서 제거
    for (int i = 0; i < objectListGroup_->childCount(); ++i) {
        auto child = objectListGroup_->child(i);
        if (child && child->text(0).contains(objectId)) {
            delete child;
            qDebug() << "Removed object from UI list:" << objectId;
            break;
        }
    }
}

void ControlTreeWidget::clearAllObjects() {
    auto& manager = ObjectManager::InterestObjectManager::instance();
    manager.clearAllInterestObjects();
    
    qDebug() << "All Interest Objects cleared by user request";
}

} // namespace ControlTreeWidget