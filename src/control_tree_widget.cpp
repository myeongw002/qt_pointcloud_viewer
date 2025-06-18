// control_tree_widget.cpp
#include "control_tree_widget.hpp"
#include "viewer_settings_manager.hpp"
#include <QApplication>
#include <QStyle>
#include <QDebug>
#include <QColorDialog>
#include <QSlider>
#include <QComboBox>
#include <QCheckBox>
#include <QPushButton>
#include <QLabel>
#include <QHBoxLayout>

namespace Widget {

// Constant definitions
const QStringList ControlTreeWidget::ROBOT_NAMES = {"COMBINED", "TUGV", "MUGV", "SUGV1", "SUGV2", "SUAV"};
const QStringList ControlTreeWidget::ROBOT_COLORS = {"#888888", "#FF0000", "#00FF00", "#0000FF", "#FFFF00", "#FF00FF"};

ControlTreeWidget::ControlTreeWidget(QWidget* parent) 
    : QTreeWidget(parent), targetWidget_(nullptr), mainWindow_(nullptr) {
    
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

void ControlTreeWidget::setRobotName(const QString& robotName) {
    robotName_ = robotName;
    
    // Clear existing tree
    clear();
    
    // Setup new 2-group structure
    setupTreeStructure();
}

void ControlTreeWidget::setTargetWidget(PointCloudWidget* widget) {
    targetWidget_ = widget;
    
    // Synchronize UI with current setting values
    if (targetWidget_) {
        syncWithWidget();
    }
}

void ControlTreeWidget::setMainWindow(QMainWindow* mainWindow) {
    mainWindow_ = mainWindow;
}

void ControlTreeWidget::setupTreeStructure() {
    // Create 2 main groups only
    viewGroup_ = new QTreeWidgetItem(this, {"Viewer Settings"});
    robotGroup_ = new QTreeWidgetItem(this, {"Robot Controls"});
    
    // Set group expansion state - both expanded by default
    viewGroup_->setExpanded(true);
    robotGroup_->setExpanded(true);
    
    // Add controls to each group
    addViewerSettings(viewGroup_);
    addRobotControls(robotGroup_);
}

void ControlTreeWidget::setupSingleRobotTree() {
    setupTreeStructure();
}

void ControlTreeWidget::setupCombinedModeTree() {
    setupTreeStructure();
}

void ControlTreeWidget::addViewerSettings(QTreeWidgetItem* parent) {
    // ============================================================================
    // Show Robot Labels (Display Elements 밖으로 이동)
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
    // Axes Controls Group
    // ============================================================================
    auto axesGroup = new QTreeWidgetItem(parent, {"Axes Controls"});
    axesGroup->setExpanded(true);
    
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
    axesSizeSlider->setRange(1, 50);  // 0.1m ~ 5.0m (x10 스케일)
    axesSizeSlider->setValue(10);     // 기본값 1.0m
    
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
    // Grid Controls Group
    // ============================================================================
    auto gridGroup = new QTreeWidgetItem(parent, {"Grid Controls"});
    gridGroup->setExpanded(true);
    
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
    // Position Marker Group (Show Current Position 추가)
    // ============================================================================
    auto markerGroup = new QTreeWidgetItem(parent, {"Position Marker"});
    markerGroup->setExpanded(true); // 기본 확장으로 변경
    
    // Show Current Position (Position Marker 아래로 이동)
    auto positionItem = new QTreeWidgetItem(markerGroup, {"Show Current Position"});
    auto positionCheck = createCheckBox(true, [this](bool checked) {
        if (targetWidget_) {
            targetWidget_->setShowPosition(checked);
            ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
            qDebug() << "Position display:" << (checked ? "ON" : "OFF");
        }
    });
    setItemWidget(positionItem, 1, positionCheck);
    
    // Position Marker Type
    auto markerTypeItem = new QTreeWidgetItem(markerGroup, {"Marker Type"});
    auto markerTypeCombo = new QComboBox();
    markerTypeCombo->addItems({"Cylinder", "Axes"});
    markerTypeCombo->setCurrentIndex(1); // Default to Axes
    connect(markerTypeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            [this, markerTypeCombo](int index) {
        if (targetWidget_) {
            Widget::PointCloudWidget::PositionMarkerType markerType = 
                (index == 0) ? Widget::PointCloudWidget::PositionMarkerType::CYLINDER 
                             : Widget::PointCloudWidget::PositionMarkerType::AXES;
            targetWidget_->setPositionMarkerType(markerType);
            ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
            qDebug() << "Position marker type:" << markerTypeCombo->currentText();
        }
    });
    setItemWidget(markerTypeItem, 1, markerTypeCombo);
    
    // Marker Size slider
    auto markerSizeItem = new QTreeWidgetItem(markerGroup, {"Marker Size"});
    auto markerSizeSlider = new QSlider(Qt::Horizontal);
    markerSizeSlider->setRange(10, 200);
    markerSizeSlider->setValue(30); // Default 0.3
    connect(markerSizeSlider, &QSlider::valueChanged, [this](int value) {
        if (targetWidget_) {
            float radius = value / 100.0f;
            targetWidget_->setPositionRadius(radius);
            ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
            qDebug() << "Marker radius:" << radius;
        }
    });
    setItemWidget(markerSizeItem, 1, markerSizeSlider);
    
    // ============================================================================
    // Point Cloud Styling Group (Show Points, Show Path 추가)
    // ============================================================================
    auto pointCloudGroup = new QTreeWidgetItem(parent, {"Point Cloud Styling"});
    pointCloudGroup->setExpanded(true); // 기본 확장으로 변경
    
    // Show Points checkbox (Point Cloud Styling 아래로 이동)
    auto showPointsItem = new QTreeWidgetItem(pointCloudGroup, {"Show Points"});
    auto showPointsCheck = createCheckBox(true, [this](bool checked) {
        if (targetWidget_) {
            targetWidget_->setShowPoints(checked);
            ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
            qDebug() << "Points display:" << (checked ? "ON" : "OFF");
        }
    });
    setItemWidget(showPointsItem, 1, showPointsCheck);
    
    // Show Path checkbox (Point Cloud Styling 아래로 이동)
    auto showPathItem = new QTreeWidgetItem(pointCloudGroup, {"Show Path"});
    auto showPathCheck = createCheckBox(true, [this](bool checked) {
        if (targetWidget_) {
            targetWidget_->setShowPath(checked);
            ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
            qDebug() << "Path display:" << (checked ? "ON" : "OFF");
        }
    });
    setItemWidget(showPathItem, 1, showPathCheck);
    
    // Point Size slider
    auto pointSizeItem = new QTreeWidgetItem(pointCloudGroup, {"Point Size"});
    auto pointSizeSlider = new QSlider(Qt::Horizontal);
    pointSizeSlider->setRange(5, 100);
    pointSizeSlider->setValue(20); // Default 2.0
    connect(pointSizeSlider, &QSlider::valueChanged, [this](int value) {
        if (targetWidget_) {
            float size = value / 10.0f;
            targetWidget_->setPointSize(size);
            ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
            qDebug() << "Point size:" << size;
        }
    });
    setItemWidget(pointSizeItem, 1, pointSizeSlider);
    
    // Path Width slider
    auto pathWidthItem = new QTreeWidgetItem(pointCloudGroup, {"Path Width"});
    auto pathWidthSlider = new QSlider(Qt::Horizontal);
    pathWidthSlider->setRange(5, 100);
    pathWidthSlider->setValue(30); // Default 3.0
    connect(pathWidthSlider, &QSlider::valueChanged, [this](int value) {
        if (targetWidget_) {
            float width = value / 10.0f;
            targetWidget_->setPathWidth(width);
            ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
            qDebug() << "Path width:" << width;
        }
    });
    setItemWidget(pathWidthItem, 1, pathWidthSlider);
    
    // ============================================================================
    // Camera Controls Group
    // ============================================================================
    auto cameraGroup = new QTreeWidgetItem(parent, {"Camera Controls"});
    cameraGroup->setExpanded(false); // 기본 접힘
    
    // Top View Mode
    auto topViewItem = new QTreeWidgetItem(cameraGroup, {"Top View Mode"});
    auto topViewCheck = createCheckBox(false, [this](bool checked) {
        if (targetWidget_) {
            targetWidget_->setTopView(checked);
            ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
            qDebug() << "Top view:" << (checked ? "ON" : "OFF");
        }
    });
    setItemWidget(topViewItem, 1, topViewCheck);
    
    // Rotation Sensitivity slider
    auto sensitivityItem = new QTreeWidgetItem(cameraGroup, {"Rotation Sensitivity"});
    auto sensitivitySlider = new QSlider(Qt::Horizontal);
    sensitivitySlider->setRange(10, 100);
    sensitivitySlider->setValue(30); // Default 0.3
    connect(sensitivitySlider, &QSlider::valueChanged, [this](int value) {
        if (targetWidget_) {
            float sensitivity = value / 100.0f;
            targetWidget_->setRotationSensitivity(sensitivity);
            ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
            qDebug() << "Rotation sensitivity:" << sensitivity;
        }
    });
    setItemWidget(sensitivityItem, 1, sensitivitySlider);
    
    // Reset Camera button
    auto resetCameraItem = new QTreeWidgetItem(cameraGroup, {"Reset Camera"});
    auto resetCameraBtn = createButton("Reset", [this]() {
        if (targetWidget_) {
            targetWidget_->resetCamera();
            qDebug() << "Camera reset";
        }
    });
    setItemWidget(resetCameraItem, 1, resetCameraBtn);
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
    
    // ============================================================================
    // Robot Colors Section
    // ============================================================================
    auto colorsGroup = new QTreeWidgetItem(parent, {"Robot Colors"});
    
    for (int i = 1; i < ROBOT_NAMES.size(); ++i) {
        const QString& robot = ROBOT_NAMES[i];
        const QString& colorHex = ROBOT_COLORS[i];
        QColor color(colorHex);
        QColor pathColor = color.lighter(150);
        
        // Create robot sub-group for each robot
        auto robotColorGroup = new QTreeWidgetItem(colorsGroup, {robot});
        
        // Point color
        auto pointColorItem = new QTreeWidgetItem(robotColorGroup, {"Points"});
        auto pointColorBtn = createButton("", [this, robot]() {
            QColor newColor = QColorDialog::getColor();
            if (newColor.isValid() && targetWidget_) {
                glm::vec3 glmColor(newColor.redF(), newColor.greenF(), newColor.blueF());
                targetWidget_->setRobotPointsColor(robot, glmColor);
                ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
                
                auto btn = qobject_cast<QPushButton*>(sender());
                if (btn) {
                    btn->setStyleSheet(QString("background-color: %1; border: 2px solid white;")
                                     .arg(newColor.name()));
                }
                qDebug() << robot << "points color changed to:" << newColor.name();
            }
        });
        pointColorBtn->setFixedSize(40, 25);
        pointColorBtn->setStyleSheet(QString("background-color: %1; border: 2px solid white;")
                                   .arg(color.name()));
        setItemWidget(pointColorItem, 1, pointColorBtn);
        colorButtons_[robot + "_points"] = pointColorBtn;
        
        // Path color
        auto pathColorItem = new QTreeWidgetItem(robotColorGroup, {"Path"});
        auto pathColorBtn = createButton("", [this, robot]() {
            QColor newColor = QColorDialog::getColor();
            if (newColor.isValid() && targetWidget_) {
                glm::vec3 glmColor(newColor.redF(), newColor.greenF(), newColor.blueF());
                targetWidget_->setRobotPathColor(robot, glmColor);
                ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
                
                auto btn = qobject_cast<QPushButton*>(sender());
                if (btn) {
                    btn->setStyleSheet(QString("background-color: %1; border: 2px solid white;")
                                     .arg(newColor.name()));
                }
                qDebug() << robot << "path color changed to:" << newColor.name();
            }
        });
        pathColorBtn->setFixedSize(40, 25);
        pathColorBtn->setStyleSheet(QString("background-color: %1; border: 2px solid white;")
                                  .arg(pathColor.name()));
        setItemWidget(pathColorItem, 1, pathColorBtn);
        colorButtons_[robot + "_path"] = pathColorBtn;
    }
    
    // Reset All Colors button
    auto resetColorsItem = new QTreeWidgetItem(parent, {"Reset All Colors"});
    auto resetBtn = createButton("Reset", [this]() {
        if (targetWidget_) {
            targetWidget_->resetAllColorsToDefault();
            ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
            updateColorButtons();
            qDebug() << "Colors reset to default";
        }
    });
    setItemWidget(resetColorsItem, 1, resetBtn);
}

// Keep existing helper functions unchanged
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

void ControlTreeWidget::syncWithWidget() {
    if (!targetWidget_) {
        return;
    }
    
    qDebug() << "Syncing controls with widget for robot:" << robotName_;
    
    // 모든 컨트롤의 현재 값을 위젯과 동기화
    auto findSliderInTree = [this](const QString& itemName) -> QSlider* {
        QTreeWidgetItemIterator it(this);
        while (*it) {
            if ((*it)->text(0) == itemName) {
                QWidget* widget = itemWidget(*it, 1);
                if (auto* slider = qobject_cast<QSlider*>(widget)) {
                    return slider;
                } else if (widget) {
                    // 복합 위젯인 경우 슬라이더 찾기
                    return widget->findChild<QSlider*>();
                }
            }
            ++it;
        }
        return nullptr;
    };
    
    // Axes Size 슬라이더 동기화
    if (auto* axesSizeSlider = findSliderInTree("Axes Size")) {
        float currentAxesSize = targetWidget_->getAxesSize();
        axesSizeSlider->setValue(static_cast<int>(currentAxesSize * 10));
        
        // 레이블도 업데이트
        if (auto* parent = axesSizeSlider->parentWidget()) {
            if (auto* label = parent->findChild<QLabel*>()) {
                label->setText(QString("%1m").arg(currentAxesSize, 0, 'f', 1));
            }
        }
    }
    
    // Grid Size 슬라이더 동기화 (기존 코드)
    if (auto* gridSizeSlider = findSliderInTree("Grid Size")) {
        float currentGridSize = targetWidget_->getGridSize();
        gridSizeSlider->setValue(static_cast<int>(currentGridSize * 10));
        
        // 레이블도 업데이트
        if (auto* parent = gridSizeSlider->parentWidget()) {
            if (auto* label = parent->findChild<QLabel*>()) {
                label->setText(QString("%1m").arg(currentGridSize, 0, 'f', 1));
            }
        }
    }
}

void ControlTreeWidget::updateColorButtons() {
    if (!targetWidget_) {
        return;
    }
    
    for (const QString& robot : ROBOT_NAMES) {
        if (robot == "COMBINED") continue;
        
        auto pointsColor = targetWidget_->getRobotPointsColor(robot);
        auto pathColor = targetWidget_->getRobotPathColor(robot);
        
        QString pointsKey = robot + "_points";
        QString pathKey = robot + "_path";
        
        if (colorButtons_.contains(pointsKey)) {
            QColor qColor(static_cast<int>(pointsColor.r * 255), 
                         static_cast<int>(pointsColor.g * 255), 
                         static_cast<int>(pointsColor.b * 255));
            colorButtons_[pointsKey]->setStyleSheet(
                QString("background-color: %1; border: 2px solid white;").arg(qColor.name()));
        }
        
        if (colorButtons_.contains(pathKey)) {
            QColor qColor(static_cast<int>(pathColor.r * 255), 
                         static_cast<int>(pathColor.g * 255), 
                         static_cast<int>(pathColor.b * 255));
            colorButtons_[pathKey]->setStyleSheet(
                QString("background-color: %1; border: 2px solid white;").arg(qColor.name()));
        }
    }
    
    qDebug() << "Color buttons updated";
}

void ControlTreeWidget::onColorButtonClicked() {
    // Implementation if needed
}

void ControlTreeWidget::onResetColorsClicked() {
    if (targetWidget_) {
        targetWidget_->resetAllColorsToDefault();
        updateColorButtons();
        qDebug() << "Colors reset to default";
    }
}

void ControlTreeWidget::onCameraPresetClicked() {
    // Implementation if needed
}

// Add missing required functions to prevent compilation errors
void ControlTreeWidget::addViewControls(QTreeWidgetItem* parent) {
    // This function is called from legacy code, redirect to new function
    addViewerSettings(parent);
}

void ControlTreeWidget::addDisplayControls(QTreeWidgetItem* parent) {
    // Legacy function - no longer used in 2-group structure
}

void ControlTreeWidget::addCameraControls(QTreeWidgetItem* parent) {
    // Legacy function - no longer used in 2-group structure
}

void ControlTreeWidget::addIndicatorControls(QTreeWidgetItem* parent) {
    // Legacy function - no longer used in 2-group structure
}

QWidget* ControlTreeWidget::createSliderWidget(const QString& label, double min, double max, double value, 
                                             std::function<void(double)> callback) {
    // Legacy function - keep for compatibility
    auto widget = new QWidget();
    auto layout = new QHBoxLayout(widget);
    
    auto labelWidget = new QLabel(label);
    auto slider = new QSlider(Qt::Horizontal);
    auto valueLabel = new QLabel(QString::number(value, 'f', 1));
    
    slider->setRange(static_cast<int>(min * 10), static_cast<int>(max * 10));
    slider->setValue(static_cast<int>(value * 10));
    
    connect(slider, &QSlider::valueChanged, [callback, valueLabel](int val) {
        double doubleVal = val / 10.0;
        valueLabel->setText(QString::number(doubleVal, 'f', 1));
        callback(doubleVal);
    });
    
    layout->addWidget(labelWidget);
    layout->addWidget(slider);
    layout->addWidget(valueLabel);
    layout->setContentsMargins(0, 0, 0, 0);
    
    return widget;
}

QWidget* ControlTreeWidget::createComboWidget(const QString& label, const QStringList& items, 
                                            const QString& current, std::function<void(const QString&)> callback) {
    // Legacy function - keep for compatibility
    auto widget = new QWidget();
    auto layout = new QHBoxLayout(widget);
    
    auto labelWidget = new QLabel(label);
    auto combo = new QComboBox();
    combo->addItems(items);
    combo->setCurrentText(current);
    
    connect(combo, &QComboBox::currentTextChanged, callback);
    
    layout->addWidget(labelWidget);
    layout->addWidget(combo);
    layout->setContentsMargins(0, 0, 0, 0);
    
    return widget;
}

QWidget* ControlTreeWidget::createColorWidget(const QString& label, const QColor& color, 
                                            std::function<void(const QColor&)> callback) {
    // Legacy function - keep for compatibility
    auto widget = new QWidget();
    auto layout = new QHBoxLayout(widget);
    
    auto labelWidget = new QLabel(label);
    auto colorButton = new QPushButton();
    colorButton->setFixedSize(40, 25);
    colorButton->setStyleSheet(QString("background-color: %1; border: 2px solid gray;").arg(color.name()));
    
    connect(colorButton, &QPushButton::clicked, [this, callback, colorButton]() {
        QColor newColor = QColorDialog::getColor(Qt::white, this, "Select Color");
        if (newColor.isValid()) {
            colorButton->setStyleSheet(QString("background-color: %1; border: 2px solid gray;").arg(newColor.name()));
            callback(newColor);
        }
    });
    
    layout->addWidget(labelWidget);
    layout->addWidget(colorButton);
    layout->setContentsMargins(0, 0, 0, 0);
    
    return widget;
}

} // namespace Widget