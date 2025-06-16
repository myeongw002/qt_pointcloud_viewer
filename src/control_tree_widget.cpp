// mainwindow.cpp
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

// ✅ 상수 정의 수정
const QStringList ControlTreeWidget::ROBOT_NAMES = {"COMBINED", "TUGV", "MUGV", "SUGV1", "SUGV2", "SUAV"};
const QStringList ControlTreeWidget::ROBOT_COLORS = {"#888888", "#FF0000", "#00FF00", "#0000FF", "#FFFF00", "#FF00FF"};

ControlTreeWidget::ControlTreeWidget(QWidget* parent) 
    : QTreeWidget(parent), targetWidget_(nullptr), mainWindow_(nullptr) {
    
    // 트리 기본 설정
    setHeaderLabels({"Property", "Value"});
    setColumnWidth(0, 200);
    setColumnWidth(1, 150);
    setAlternatingRowColors(true);
    setRootIsDecorated(true);
    setIndentation(15);
    
    // ✅ 스타일 설정 제거 (Qt 기본 테마 사용)
    // setStyleSheet(...) 코드 삭제
    
    // 초기 트리 구조 설정
    setupTreeStructure();
    
    // 시그널 연결
    connect(this, &QTreeWidget::itemChanged, this, &ControlTreeWidget::onItemChanged);
}

void ControlTreeWidget::setRobotName(const QString& robotName) {
    robotName_ = robotName;
    
    // 기존 트리 클리어
    clear();
    
    // 로봇 타입에 따라 다른 트리 구성
    if (robotName == "COMBINED") {
        setupCombinedModeTree();
    } else {
        setupSingleRobotTree();
    }
}

void ControlTreeWidget::setTargetWidget(PointCloudWidget* widget) {
    targetWidget_ = widget;
    
    // 현재 설정값들로 UI 동기화
    if (targetWidget_) {
        syncWithWidget();
    }
}

// ✅ MainWindow 참조 설정
void ControlTreeWidget::setMainWindow(QMainWindow* mainWindow) {
    mainWindow_ = mainWindow;
}

void ControlTreeWidget::setupTreeStructure() {
    // 메인 그룹들 생성
    viewGroup_ = new QTreeWidgetItem(this, {"🎭 View Settings"});
    robotGroup_ = new QTreeWidgetItem(this, {"🤖 Robot Settings"});
    displayGroup_ = new QTreeWidgetItem(this, {"🎨 Display Settings"});
    cameraGroup_ = new QTreeWidgetItem(this, {"📷 Camera Controls"});
    indicatorGroup_ = new QTreeWidgetItem(this, {"📍 Indicator Controls"});
    
    // 그룹들 확장 상태 설정
    viewGroup_->setExpanded(true);
    robotGroup_->setExpanded(true);
    displayGroup_->setExpanded(false);
    cameraGroup_->setExpanded(false);
    indicatorGroup_->setExpanded(false);
    
    // 각 그룹에 컨트롤들 추가
    addViewControls(viewGroup_);
    addRobotControls(robotGroup_);
    addDisplayControls(displayGroup_);
    addCameraControls(cameraGroup_);
    addIndicatorControls(indicatorGroup_);
}

void ControlTreeWidget::setupSingleRobotTree() {
    setupTreeStructure();
    
    // 단일 로봇 모드에서는 로봇 선택 콤보박스 숨기기
    if (robotGroup_) {
        for (int i = 0; i < robotGroup_->childCount(); ++i) {
            QTreeWidgetItem* child = robotGroup_->child(i);
            if (child && child->text(0).contains("Current Robot")) {
                child->setHidden(true);
                break;
            }
        }
    }
}

void ControlTreeWidget::setupCombinedModeTree() {
    setupTreeStructure();
    
    // COMBINED 모드에서는 모든 컨트롤 표시
    // 특별한 설정이 필요한 경우 여기서 처리
}

void ControlTreeWidget::addViewControls(QTreeWidgetItem* parent) {
    // 탑뷰 모드 토글
    auto topViewItem = new QTreeWidgetItem(parent, {"Top View Mode"});
    auto topViewCheck = createCheckBox(false, [this](bool checked) {
        if (targetWidget_) {
            targetWidget_->setTopView(checked);
            // Add synchronization
            ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
            qDebug() << "Top view:" << (checked ? "ON" : "OFF");
        }
    });
    setItemWidget(topViewItem, 1, topViewCheck);
    
    // 축 표시 토글
    auto axesItem = new QTreeWidgetItem(parent, {"Show Axes"});
    auto axesCheck = createCheckBox(true, [this](bool checked) {
        if (targetWidget_) {
            targetWidget_->setShowAxes(checked);
            // Add synchronization
            ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
            qDebug() << "Axes display:" << (checked ? "ON" : "OFF");
        }
    });
    setItemWidget(axesItem, 1, axesCheck);
    
    // 그리드 표시 토글
    auto gridItem = new QTreeWidgetItem(parent, {"Show Grid"});
    auto gridCheck = createCheckBox(true, [this](bool checked) {
        if (targetWidget_) {
            targetWidget_->setShowGrid(checked);
            // Add synchronization
            ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
            qDebug() << "Grid display:" << (checked ? "ON" : "OFF");
        }
    });
    setItemWidget(gridItem, 1, gridCheck);
    
    // 로봇 라벨 표시 토글
    auto labelItem = new QTreeWidgetItem(parent, {"Show Robot Labels"});
    auto labelCheck = createCheckBox(true, [this](bool checked) {
        if (targetWidget_) {
            targetWidget_->setShowRobotLabel(checked);
            // Add synchronization
            ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
            qDebug() << "Robot labels:" << (checked ? "ON" : "OFF");
        }
    });
    setItemWidget(labelItem, 1, labelCheck);
    
    // 현재 위치 마커 표시
    auto positionItem = new QTreeWidgetItem(parent, {"Show Current Position"});
    auto positionCheck = createCheckBox(true, [this](bool checked) {
        if (targetWidget_) {
            targetWidget_->setShowPosition(checked);
            // Add synchronization
            ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
            qDebug() << "Position display:" << (checked ? "ON" : "OFF");
        }
    });
    setItemWidget(positionItem, 1, positionCheck);
}

void ControlTreeWidget::addRobotControls(QTreeWidgetItem* parent) {
    // 현재 로봇 선택 (COMBINED 모드가 아닐 때만)
    if (robotName_ != "COMBINED") {
        auto robotSelectItem = new QTreeWidgetItem(parent, {"Current Robot"});
        auto robotCombo = new QComboBox();
        robotCombo->addItems(ROBOT_NAMES);
        robotCombo->addItem("COMBINED");
        robotCombo->setCurrentText(robotName_);
        connect(robotCombo, QOverload<const QString&>::of(&QComboBox::currentTextChanged), 
                [this](const QString& robot) {
            if (targetWidget_) targetWidget_->setRobot(robot);
        });
        setItemWidget(robotSelectItem, 1, robotCombo);
    }
    
    // 로봇별 색상 설정
    auto colorsSubGroup = new QTreeWidgetItem(parent, {"🎨 Robot Colors"});
    
    for (int i = 1; i < ROBOT_NAMES.size(); ++i) {
        const QString& robot = ROBOT_NAMES[i];
        const QString& colorHex = ROBOT_COLORS[i];
        QColor color(colorHex);
        
        // ✅ Calculate path color (lighter version of points color)
        QColor pathColor = color.lighter(150); // Make path color 50% lighter
        
        // 포인트 색상
        auto pointColorItem = new QTreeWidgetItem(colorsSubGroup, {robot + " Points"});
        auto pointColorBtn = createButton("", [this, robot]() {
            QColor newColor = QColorDialog::getColor();
            if (newColor.isValid() && targetWidget_) {
                glm::vec3 glmColor(newColor.redF(), newColor.greenF(), newColor.blueF());
                targetWidget_->setRobotPointsColor(robot, glmColor);
                
                // Add synchronization
                ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
                
                // 버튼 색상 업데이트
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
        
        // 경로 색상
        auto pathColorItem = new QTreeWidgetItem(colorsSubGroup, {robot + " Path"});
        auto pathColorBtn = createButton("", [this, robot]() {
            QColor newColor = QColorDialog::getColor();
            if (newColor.isValid() && targetWidget_) {
                glm::vec3 glmColor(newColor.redF(), newColor.greenF(), newColor.blueF());
                targetWidget_->setRobotPathColor(robot, glmColor);
                
                // Add synchronization
                ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
                
                // 버튼 색상 업데이트
                auto btn = qobject_cast<QPushButton*>(sender());
                if (btn) {
                    btn->setStyleSheet(QString("background-color: %1; border: 2px solid white;")
                                     .arg(newColor.name()));
                }
                qDebug() << robot << "path color changed to:" << newColor.name();
            }
        });
        pathColorBtn->setFixedSize(40, 25);
        // ✅ Fixed: Use pathColor variable instead of undefined pathColor
        pathColorBtn->setStyleSheet(QString("background-color: %1; border: 2px solid white;")
                                  .arg(pathColor.name()));
        setItemWidget(pathColorItem, 1, pathColorBtn);
        colorButtons_[robot + "_path"] = pathColorBtn;
    }
    
    // 색상 리셋 버튼
    auto resetColorsItem = new QTreeWidgetItem(parent, {"Reset All Colors"});
    auto resetBtn = createButton("Reset", [this]() {
        if (targetWidget_) {
            targetWidget_->resetAllColorsToDefault();
            // Add synchronization
            ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
            updateColorButtons();
            qDebug() << "Colors reset to default";
        }
    });
    setItemWidget(resetColorsItem, 1, resetBtn);
}

void ControlTreeWidget::addDisplayControls(QTreeWidgetItem* parent) {
    // Show Points checkbox - ✅ 기본값을 true로 설정
    auto showPointsItem = new QTreeWidgetItem(parent, {"Show Points"});
    QCheckBox* showPointsCheck = createCheckBox(true, [this](bool checked) {
        if (targetWidget_) {
            targetWidget_->setShowPoints(checked);
            ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
            qDebug() << robotName_ << "Points display:" << (checked ? "ON" : "OFF");
        }
    });
    setItemWidget(showPointsItem, 1, showPointsCheck);
    
    // Show Path checkbox - ✅ 기본값을 true로 설정
    auto showPathItem = new QTreeWidgetItem(parent, {"Show Path"});
    QCheckBox* showPathCheck = createCheckBox(true, [this](bool checked) {
        if (targetWidget_) {
            targetWidget_->setShowPath(checked);
            ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
            qDebug() << robotName_ << "Path display:" << (checked ? "ON" : "OFF");
        }
    });
    setItemWidget(showPathItem, 1, showPathCheck);
    
    // Position Marker Type selection
    auto markerTypeItem = new QTreeWidgetItem(parent, {"Position Marker Type"});
    auto markerTypeCombo = new QComboBox();
    markerTypeCombo->addItems({"Cylinder", "Axes"});
    markerTypeCombo->setCurrentIndex(1);
    
    connect(markerTypeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            [this, markerTypeCombo](int index) {
        if (targetWidget_) {
            Widget::PointCloudWidget::PositionMarkerType markerType;
            if (index == 0) {
                markerType = Widget::PointCloudWidget::PositionMarkerType::CYLINDER;
            } else {
                markerType = Widget::PointCloudWidget::PositionMarkerType::AXES;
            }
            
            targetWidget_->setPositionMarkerType(markerType);
            
            // Save settings and notify manager
            Widget::ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
            
            qDebug() << "Position marker type changed to:" << markerTypeCombo->currentText();
        }
    });
    setItemWidget(markerTypeItem, 1, markerTypeCombo);
    
    // Point Size slider
    auto pointSizeItem = new QTreeWidgetItem(parent, {"Point Size"});
    auto pointSizeSlider = new QSlider(Qt::Horizontal);
    pointSizeSlider->setRange(5, 100);
    pointSizeSlider->setValue(20);
    connect(pointSizeSlider, &QSlider::valueChanged, [this](int value) {
        if (targetWidget_) {
            float size = value / 10.0f;
            targetWidget_->setPointSize(size);
            
            // Save settings and notify manager
            Widget::ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
            
            qDebug() << "Point size:" << size;
        }
    });
    setItemWidget(pointSizeItem, 1, pointSizeSlider);
    
    // Path Width slider
    auto pathWidthItem = new QTreeWidgetItem(parent, {"Path Width"});
    auto pathWidthSlider = new QSlider(Qt::Horizontal);
    pathWidthSlider->setRange(5, 100);
    pathWidthSlider->setValue(30);
    connect(pathWidthSlider, &QSlider::valueChanged, [this](int value) {
        if (targetWidget_) {
            float width = value / 10.0f;
            targetWidget_->setPathWidth(width);
            
            // Save settings and notify manager
            Widget::ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
            
            qDebug() << "Path width:" << width;
        }
    });
    setItemWidget(pathWidthItem, 1, pathWidthSlider);
    
    // Marker Size slider
    auto markerSizeItem = new QTreeWidgetItem(parent, {"Marker Size"});
    auto sizeSlider = new QSlider(Qt::Horizontal);
    sizeSlider->setRange(10, 200);
    sizeSlider->setValue(30);
    connect(sizeSlider, &QSlider::valueChanged, [this](int value) {
        if (targetWidget_) {
            float radius = value / 100.0f;
            targetWidget_->setPositionRadius(radius);
            // Add missing synchronization
            ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
            qDebug() << "Marker radius:" << radius;
        }
    });
    setItemWidget(markerSizeItem, 1, sizeSlider);
    
    // Rotation Sensitivity slider
    auto sensitivityItem = new QTreeWidgetItem(parent, {"Rotation Sensitivity"});
    auto sensitivitySlider = new QSlider(Qt::Horizontal);
    sensitivitySlider->setRange(10, 100);
    sensitivitySlider->setValue(30);
    connect(sensitivitySlider, &QSlider::valueChanged, [this](int value) {
        if (targetWidget_) {
            float sensitivity = value / 100.0f;
            targetWidget_->setRotationSensitivity(sensitivity);
            // Add missing synchronization
            ViewerSettingsManager::instance()->saveSettings(robotName_, targetWidget_);
            qDebug() << "Rotation sensitivity:" << sensitivity;
        }
    });
    setItemWidget(sensitivityItem, 1, sensitivitySlider);
}

void ControlTreeWidget::addCameraControls(QTreeWidgetItem* parent) {
    // 카메라 리셋 버튼
    auto resetCameraItem = new QTreeWidgetItem(parent, {"Reset Camera"});
    auto resetCameraBtn = createButton("Reset", [this]() {
        if (targetWidget_) targetWidget_->resetCamera();
    });
    setItemWidget(resetCameraItem, 1, resetCameraBtn);
    
    // 카메라 프리셋들
    auto presetsGroup = new QTreeWidgetItem(parent, {"📐 View Presets"});
    
    // 프론트 뷰
    auto frontViewItem = new QTreeWidgetItem(presetsGroup, {"Front View"});
    auto frontBtn = createButton("Apply", [this]() {
        if (targetWidget_) {
            targetWidget_->setFocusPoint(glm::vec3(0, 0, 0));
            targetWidget_->resetCamera();
        }
    });
    setItemWidget(frontViewItem, 1, frontBtn);
}

PointCloudWidget* ControlTreeWidget::findRobotWidget(const QString& robotName) {
    if (!mainWindow_) {
        qDebug() << "❌ MainWindow reference not set!";
        return nullptr;
    }
    
    // MainWindow에서 해당 로봇의 위젯을 찾기
    int widgetIndex = -1;
    
    if (robotName == "COMBINED") {
        widgetIndex = 0;  // COMBINED는 인덱스 0
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
    
    // ✅ 수정: 모든 유효한 인덱스에 대해 처리
    if (widgetIndex >= 0) {  // ✅ >= 0으로 변경
        auto* widget = qobject_cast<PointCloudWidget*>(
            mainWindow_->findChild<QWidget*>(QString("openGLWidget_%1").arg(widgetIndex))
        );
        
        if (widget) {
            qDebug() << "✅ Found widget for" << robotName << "at openGLWidget_" << widgetIndex;
            return widget;
        } else {
            qDebug() << "❌ Could not find openGLWidget_" << widgetIndex << "for" << robotName;
        }
    } else {
        qDebug() << "❌ Invalid robot name:" << robotName;
    }
    
    return nullptr;
}

void ControlTreeWidget::addIndicatorControls(QTreeWidgetItem* parent) {
    // 인디케이터 현재 위치 고정
    auto lockIndicatorItem = new QTreeWidgetItem(parent, {"Lock to Robot Position"});
    auto lockCheck = createCheckBox(false, [this](bool checked) {
        if (targetWidget_) {
            targetWidget_->setLockIndicatorToCurrentPosition(checked);
            qDebug() << "🔒 Lock indicator:" << (checked ? "ON" : "OFF");
        }
    });
    setItemWidget(lockIndicatorItem, 1, lockCheck);
    
    // 추적 대상 로봇 선택
    auto targetRobotItem = new QTreeWidgetItem(parent, {"Target Robot"});
    targetRobotCombo_ = new QComboBox();  // ✅ 멤버 변수에 저장
    
    QStringList realRobots = {"TUGV", "MUGV", "SUGV1", "SUGV2", "SUAV"};
    targetRobotCombo_->addItems(realRobots);
    targetRobotCombo_->setCurrentText("TUGV");
    
    connect(targetRobotCombo_, QOverload<const QString&>::of(&QComboBox::currentTextChanged), 
            [this](const QString& robot) {
        qDebug() << "🎯 Target robot combo changed to:" << robot;
        
        // ✅ COMBINED 모드에서는 개별 위젯과 COMBINED 위젯 모두에 설정
        if (robotName_ == "COMBINED") {
            // 1. 개별 위젯에 설정
            auto* robotWidget = findRobotWidget(robot);
            if (robotWidget) {
                robotWidget->setIndicatorTargetRobot(robot);  // ✅ 개별 위젯에 설정
                qDebug() << "🎯 Individual widget target changed to:" << robot;
            }
            
            // 2. COMBINED 위젯에도 설정
            if (targetWidget_) {
                targetWidget_->setIndicatorTargetRobot(robot);  // ✅ COMBINED 위젯에도 설정
                qDebug() << "🎯 COMBINED widget target changed to:" << robot;
            }
        } else if (targetWidget_) {
            // 일반 모드에서는 현재 위젯에만 설정
            targetWidget_->setIndicatorTargetRobot(robot);
            qDebug() << "🎯 Target robot changed to:" << robot;
        }
    });
    setItemWidget(targetRobotItem, 1, targetRobotCombo_);
    
    // ✅ Quick Jump 버튼들 (qDebug 사용)
    auto jumpGroup = new QTreeWidgetItem(parent, {"🎯 Quick Jump"});
    
    for (const QString& robot : realRobots) {
        auto jumpItem = new QTreeWidgetItem(jumpGroup, {"Jump to " + robot});
        auto jumpBtn = createButton("Go", [this, robot]() {
            qDebug() << "🚀 Quick jump to" << robot << "initiated...";
            
            PointCloudWidget* activeWidget = nullptr;
            
            // ✅ COMBINED 모드에서는 항상 해당 로봇의 개별 위젯 찾기
            if (robotName_ == "COMBINED") {
                activeWidget = findRobotWidget(robot);  // ✅ 매번 새로 찾기
                if (!activeWidget) {
                    qDebug() << "❌ Could not find individual widget for" << robot;
                    return;
                }
                qDebug() << "📡 Found individual widget for" << robot << "in COMBINED mode";
            } else {
                activeWidget = targetWidget_;
                if (!activeWidget) {
                    qDebug() << "❌ No target widget for quick jump!";
                    return;
                }
            }
            
            // ✅ 1. 개별 위젯에 타겟 로봇 설정 (강제로 설정)
            activeWidget->setIndicatorTargetRobot(robot);
            qDebug() << "🎯 Set target robot" << robot << "on individual widget";
            
            // ✅ 2. COMBINED 모드에서는 COMBINED 위젯에도 타겟 설정
            if (robotName_ == "COMBINED" && targetWidget_) {
                targetWidget_->setIndicatorTargetRobot(robot);
                qDebug() << "🎯 Set target robot" << robot << "on COMBINED widget";
            }
            
            // ✅ 3. 개별 위젯에서 로봇 위치 가져오기
            glm::vec3 robotPosition(0.0f, 0.0f, 0.0f);
            bool positionFound = false;
            
            if (robotName_ == "COMBINED") {
                // 개별 위젯에서 로봇의 현재 위치 가져오기
                robotPosition = activeWidget->getRobotCurrentPosition(robot);
                positionFound = true;
                qDebug() << "📍 Robot" << robot << "position: (" 
                         << robotPosition.x << "," << robotPosition.y << "," << robotPosition.z << ")";
            }
            
            // ✅ 4. 카메라 이동
            if (robotName_ == "COMBINED" && positionFound && targetWidget_) {
                // COMBINED 위젯의 카메라를 로봇 위치로 이동
                targetWidget_->jumpToPosition(robotPosition);
                qDebug() << "📷 COMBINED camera jumped to" << robot << "position";
            } else if (activeWidget) {
                // 일반 모드에서는 기존 방식 사용
                activeWidget->jumpToRobotPosition(robot);
                qDebug() << "📷 Individual camera jumped to" << robot << "position";
            }
            
            // ✅ 5. 위치 고정 활성화
            activeWidget->setLockIndicatorToCurrentPosition(true);
            
            if (robotName_ == "COMBINED" && targetWidget_) {
                targetWidget_->setLockIndicatorToCurrentPosition(true);
                qDebug() << "🔗 COMBINED widget also locked to" << robot;
            }
            
            // ✅ 6. 콤보박스도 업데이트 (멤버 변수 사용)
            if (targetRobotCombo_ && targetRobotCombo_->currentText() != robot) {
                targetRobotCombo_->setCurrentText(robot);  // UI 동기화
                qDebug() << "🔄 Updated combo box to:" << robot;
            }
            
            // ✅ 7. 안전한 타이머로 자동 해제 (3초 후)
            QTimer* autoReleaseTimer = new QTimer();
            autoReleaseTimer->setSingleShot(true);
            
            connect(autoReleaseTimer, &QTimer::timeout, [this, autoReleaseTimer, robot, activeWidget]() {
                if (activeWidget) {
                    activeWidget->setLockIndicatorToCurrentPosition(false);
                    qDebug() << "🔓 Auto-release lock for" << robot;
                }
                
                if (robotName_ == "COMBINED" && targetWidget_) {
                    targetWidget_->setLockIndicatorToCurrentPosition(false);
                    qDebug() << "🔓 Auto-release lock for COMBINED widget";
                }
                
                autoReleaseTimer->deleteLater();
            });
            
            autoReleaseTimer->start(3000);
            
            qDebug() << "✅ Quick jump to" << robot << "completed!";
        });
        
        setItemWidget(jumpItem, 1, jumpBtn);
    }
    
    // ✅ 전체 리셋 버튼 (qDebug 사용)
    auto resetIndicatorItem = new QTreeWidgetItem(parent, {"Reset Indicator"});
    auto resetBtn = createButton("Reset", [this]() {
        // 개별 위젯들 리셋
        if (robotName_ == "COMBINED") {
            QStringList realRobots = {"TUGV", "MUGV", "SUGV1", "SUGV2", "SUAV"};
            for (const QString& robot : realRobots) {
                auto* robotWidget = findRobotWidget(robot);
                if (robotWidget) {
                    robotWidget->setLockIndicatorToCurrentPosition(false);
                    robotWidget->setIndicatorTargetRobot("TUGV");  // 기본 로봇으로 리셋
                    qDebug() << "🔄 Reset individual widget for" << robot;
                }
            }
        }
        
        // COMBINED 위젯 리셋
        if (targetWidget_) {
            targetWidget_->setLockIndicatorToCurrentPosition(false);
            targetWidget_->setIndicatorTargetRobot("TUGV");  // 기본 로봇으로 리셋
            qDebug() << "🔄 Reset COMBINED widget";
        }
    });
    setItemWidget(resetIndicatorItem, 1, resetBtn);
}

void ControlTreeWidget::onItemChanged(QTreeWidgetItem* item, int column) {
    // 아이템 변경 시 처리 (현재는 로봇 색상 변경에만 사용)
    if (robotGroup_ && item->parent() == robotGroup_) {
        QString itemName = item->text(0);
        
        // 포인트 색상 아이템인지 확인
        if (itemName.endsWith("Points")) {
            QString robotName = itemName.split(" ")[0];
            QColor newColor = colorButtons_[robotName + "_points"]->palette().button().color();
            if (targetWidget_) {
                glm::vec3 glmColor(newColor.redF(), newColor.greenF(), newColor.blueF());
                targetWidget_->setRobotPointsColor(robotName, glmColor);
                qDebug() << "Changed" << robotName << "points color to" << newColor.name();
            }
        }
        // 경로 색상 아이템인지 확인
        else if (itemName.endsWith("Path")) {
            QString robotName = itemName.split(" ")[0];
            QColor newColor = colorButtons_[robotName + "_path"]->palette().button().color();
            if (targetWidget_) {
                glm::vec3 glmColor(newColor.redF(), newColor.greenF(), newColor.blueF());
                targetWidget_->setRobotPathColor(robotName, glmColor);
                qDebug() << "Changed" << robotName << "path color to" << newColor.name();
            }
        }
    }
}

// Add these function implementations at the end of the file, before the closing namespace bracket

// Helper widget creation functions
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

QWidget* ControlTreeWidget::createSliderWidget(const QString& label, double min, double max, double value, 
                                             std::function<void(double)> callback) {
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

// Synchronize controls with widget state
void ControlTreeWidget::syncWithWidget() {
    if (!targetWidget_) {
        return;
    }
    
    // Sync display settings
    // Note: You might need to add getter functions to PointCloudWidget
    // to properly sync the current state
    
    qDebug() << "Syncing controls with widget for robot:" << robotName_;
    
    // This function should update UI controls to match current widget state
    // Implementation depends on what getter functions are available in PointCloudWidget
}

// Update color buttons to reflect current colors
void ControlTreeWidget::updateColorButtons() {
    if (!targetWidget_) {
        return;
    }
    
    // Update color buttons for each robot
    for (const QString& robot : ROBOT_NAMES) {
        if (robot == "COMBINED") continue;
        
        auto pointsColor = targetWidget_->getRobotPointsColor(robot);
        auto pathColor = targetWidget_->getRobotPathColor(robot);
        
        // Update button colors if they exist
        QString pointsKey = robot + "_points";
        QString pathKey = robot + "_path";
        
        if (colorButtons_.contains(pointsKey)) {
            QColor qColor(static_cast<int>(pointsColor.r * 255), 
                         static_cast<int>(pointsColor.g * 255), 
                         static_cast<int>(pointsColor.b * 255));
            colorButtons_[pointsKey]->setStyleSheet(
                QString("background-color: %1; border: 2px solid gray;").arg(qColor.name()));
        }
        
        if (colorButtons_.contains(pathKey)) {
            QColor qColor(static_cast<int>(pathColor.r * 255), 
                         static_cast<int>(pathColor.g * 255), 
                         static_cast<int>(pathColor.b * 255));
            colorButtons_[pathKey]->setStyleSheet(
                QString("background-color: %1; border: 2px solid gray;").arg(qColor.name()));
        }
    }
    
    qDebug() << "Color buttons updated";
}




void ControlTreeWidget::onColorButtonClicked() {
    // Handle color button clicks
    QPushButton* button = qobject_cast<QPushButton*>(sender());
    if (!button || !targetWidget_) {
        return;
    }
    
    QColor newColor = QColorDialog::getColor(Qt::white, this, "Select Color");
    if (newColor.isValid()) {
        button->setStyleSheet(QString("background-color: %1; border: 2px solid gray;").arg(newColor.name()));
        
        // Apply color to appropriate robot/component
        // This would need to be implemented based on button identification
        qDebug() << "Color changed to:" << newColor.name();
    }
}

void ControlTreeWidget::onResetColorsClicked() {
    if (targetWidget_) {
        targetWidget_->resetAllColorsToDefault();
        updateColorButtons();
        qDebug() << "Colors reset to default";
    }
}

void ControlTreeWidget::onCameraPresetClicked() {
    QPushButton* button = qobject_cast<QPushButton*>(sender());
    if (!button || !targetWidget_) {
        return;
    }
    
    QString presetName = button->text();
    
    if (presetName == "Reset Camera") {
        targetWidget_->resetCamera();
        qDebug() << "Camera reset";
    } else if (presetName == "Top View") {
        targetWidget_->setTopView(true);
        qDebug() << "Top view activated";
    }
    // Add more camera presets as needed
}

} // namespace Widget