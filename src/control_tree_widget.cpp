// mainwindow.cpp
#include "control_tree_widget.hpp"
#include <QApplication>
#include <QStyle>

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
        if (targetWidget_) targetWidget_->setTopView(checked);
    });
    setItemWidget(topViewItem, 1, topViewCheck);
    
    // 축 표시 토글
    auto axesItem = new QTreeWidgetItem(parent, {"Show Axes"});
    auto axesCheck = createCheckBox(true, [this](bool checked) {
        if (targetWidget_) targetWidget_->setShowAxes(checked);
    });
    setItemWidget(axesItem, 1, axesCheck);
    
    // 그리드 표시 토글
    auto gridItem = new QTreeWidgetItem(parent, {"Show Grid"});
    auto gridCheck = createCheckBox(true, [this](bool checked) {
        if (targetWidget_) targetWidget_->setShowGrid(checked);
    });
    setItemWidget(gridItem, 1, gridCheck);
    
    // 로봇 라벨 표시 토글
    auto labelItem = new QTreeWidgetItem(parent, {"Show Robot Labels"});
    auto labelCheck = createCheckBox(true, [this](bool checked) {
        if (targetWidget_) targetWidget_->setShowRobotLabel(checked);
    });
    setItemWidget(labelItem, 1, labelCheck);
    
    // 현재 위치 마커 표시
    auto positionItem = new QTreeWidgetItem(parent, {"Show Current Position"});
    auto positionCheck = createCheckBox(true, [this](bool checked) {
        if (targetWidget_) targetWidget_->setShowPosition(checked);
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
    
    for (int i = 0; i < ROBOT_NAMES.size(); ++i) {
        const QString& robot = ROBOT_NAMES[i];
        const QString& colorHex = ROBOT_COLORS[i];
        QColor color(colorHex);
        
        // 포인트 색상
        auto pointColorItem = new QTreeWidgetItem(colorsSubGroup, {robot + " Points"});
        auto pointColorBtn = createButton("", [this, robot]() {
            QColor newColor = QColorDialog::getColor();
            if (newColor.isValid() && targetWidget_) {
                glm::vec3 glmColor(newColor.redF(), newColor.greenF(), newColor.blueF());
                targetWidget_->setRobotPointsColor(robot, glmColor);
                
                // 버튼 색상 업데이트
                auto btn = qobject_cast<QPushButton*>(sender());
                if (btn) {
                    btn->setStyleSheet(QString("background-color: %1; border: 2px solid white;")
                                     .arg(newColor.name()));
                }
            }
        });
        pointColorBtn->setFixedSize(40, 25);
        pointColorBtn->setStyleSheet(QString("background-color: %1; border: 2px solid white;")
                                   .arg(color.name()));
        setItemWidget(pointColorItem, 1, pointColorBtn);
        colorButtons_[robot + "_points"] = pointColorBtn;
        
        // 경로 색상
        auto pathColorItem = new QTreeWidgetItem(colorsSubGroup, {robot + " Path"});
        QColor pathColor = color.lighter(150);
        auto pathColorBtn = createButton("", [this, robot]() {
            QColor newColor = QColorDialog::getColor();
            if (newColor.isValid() && targetWidget_) {
                glm::vec3 glmColor(newColor.redF(), newColor.greenF(), newColor.blueF());
                targetWidget_->setRobotPathColor(robot, glmColor);
                
                // 버튼 색상 업데이트
                auto btn = qobject_cast<QPushButton*>(sender());
                if (btn) {
                    btn->setStyleSheet(QString("background-color: %1; border: 2px solid white;")
                                     .arg(newColor.name()));
                }
            }
        });
        pathColorBtn->setFixedSize(40, 25);
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
            updateColorButtons();
        }
    });
    setItemWidget(resetColorsItem, 1, resetBtn);
}

void ControlTreeWidget::addDisplayControls(QTreeWidgetItem* parent) {
    // 마커 타입 선택
    auto markerTypeItem = new QTreeWidgetItem(parent, {"Position Marker Type"});
    auto markerCombo = new QComboBox();
    markerCombo->addItems({"Cylinder", "Axes"});
    markerCombo->setCurrentText("Axes");
    connect(markerCombo, QOverload<const QString&>::of(&QComboBox::currentTextChanged), 
            [this](const QString& type) {
        if (targetWidget_) {
            // ✅ PositionMarkerType이 정의되지 않은 경우 일단 주석 처리
            // if (type == "Cylinder") {
            //     targetWidget_->setPositionMarkerType(PositionMarkerType::CYLINDER);
            // } else {
            //     targetWidget_->setPositionMarkerType(PositionMarkerType::AXES);
            // }
            
            // ✅ 임시로 문자열로 처리하거나 다른 방법 사용
            // targetWidget_->setPositionMarkerType(type);
        }
    });
    setItemWidget(markerTypeItem, 1, markerCombo);
    
    // 마커 크기 조절
    auto markerSizeItem = new QTreeWidgetItem(parent, {"Marker Size"});
    auto sizeSlider = new QSlider(Qt::Horizontal);
    sizeSlider->setRange(10, 200);
    sizeSlider->setValue(30);
    connect(sizeSlider, &QSlider::valueChanged, [this](int value) {
        if (targetWidget_) {
            float size = value / 100.0f;
            targetWidget_->setPositionRadius(size);
        }
    });
    setItemWidget(markerSizeItem, 1, sizeSlider);
    
    // 회전 민감도
    auto sensitivityItem = new QTreeWidgetItem(parent, {"Rotation Sensitivity"});
    auto sensitivitySlider = new QSlider(Qt::Horizontal);
    sensitivitySlider->setRange(10, 100);
    sensitivitySlider->setValue(30);
    connect(sensitivitySlider, &QSlider::valueChanged, [this](int value) {
        if (targetWidget_) {
            float sensitivity = value / 100.0f;
            targetWidget_->setRotationSensitivity(sensitivity);
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
        std::cerr << "❌ MainWindow reference not set!" << std::endl;
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
            std::cout << "✅ Found widget for " << robotName.toStdString() 
                      << " at openGLWidget_" << widgetIndex << std::endl;
            return widget;
        } else {
            std::cerr << "❌ Could not find openGLWidget_" << widgetIndex 
                      << " for " << robotName.toStdString() << std::endl;
        }
    } else {
        std::cerr << "❌ Invalid robot name: " << robotName.toStdString() << std::endl;
    }
    
    return nullptr;
}

void ControlTreeWidget::addIndicatorControls(QTreeWidgetItem* parent) {
    // 인디케이터 현재 위치 고정
    auto lockIndicatorItem = new QTreeWidgetItem(parent, {"Lock to Robot Position"});
    auto lockCheck = createCheckBox(false, [this](bool checked) {
        if (targetWidget_) {
            targetWidget_->setLockIndicatorToCurrentPosition(checked);
            std::cout << "🔒 Lock indicator: " << (checked ? "ON" : "OFF") << std::endl;
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
        std::cout << "🎯 Target robot combo changed to: " << robot.toStdString() << std::endl;
        
        // ✅ COMBINED 모드에서는 개별 위젯과 COMBINED 위젯 모두에 설정
        if (robotName_ == "COMBINED") {
            // 1. 개별 위젯에 설정
            auto* robotWidget = findRobotWidget(robot);
            if (robotWidget) {
                robotWidget->setIndicatorTargetRobot(robot);  // ✅ 개별 위젯에 설정
                std::cout << "🎯 Individual widget target changed to: " << robot.toStdString() << std::endl;
            }
            
            // 2. COMBINED 위젯에도 설정
            if (targetWidget_) {
                targetWidget_->setIndicatorTargetRobot(robot);  // ✅ COMBINED 위젯에도 설정
                std::cout << "🎯 COMBINED widget target changed to: " << robot.toStdString() << std::endl;
            }
        } else if (targetWidget_) {
            // 일반 모드에서는 현재 위젯에만 설정
            targetWidget_->setIndicatorTargetRobot(robot);
            std::cout << "🎯 Target robot changed to: " << robot.toStdString() << std::endl;
        }
    });
    setItemWidget(targetRobotItem, 1, targetRobotCombo_);
    
    // ✅ Quick Jump 버튼들 (수정된 버전)
    auto jumpGroup = new QTreeWidgetItem(parent, {"🎯 Quick Jump"});
    
    for (const QString& robot : realRobots) {
        auto jumpItem = new QTreeWidgetItem(jumpGroup, {"Jump to " + robot});
        auto jumpBtn = createButton("Go", [this, robot]() {
            std::cout << "🚀 Quick jump to " << robot.toStdString() << " initiated..." << std::endl;
            
            PointCloudWidget* activeWidget = nullptr;
            
            // ✅ COMBINED 모드에서는 항상 해당 로봇의 개별 위젯 찾기
            if (robotName_ == "COMBINED") {
                activeWidget = findRobotWidget(robot);  // ✅ 매번 새로 찾기
                if (!activeWidget) {
                    std::cerr << "❌ Could not find individual widget for " << robot.toStdString() << std::endl;
                    return;
                }
                std::cout << "📡 Found individual widget for " << robot.toStdString() << " in COMBINED mode" << std::endl;
            } else {
                activeWidget = targetWidget_;
                if (!activeWidget) {
                    std::cerr << "❌ No target widget for quick jump!" << std::endl;
                    return;
                }
            }
            
            // ✅ 1. 개별 위젯에 타겟 로봇 설정 (강제로 설정)
            activeWidget->setIndicatorTargetRobot(robot);
            std::cout << "🎯 Set target robot " << robot.toStdString() << " on individual widget" << std::endl;
            
            // ✅ 2. COMBINED 모드에서는 COMBINED 위젯에도 타겟 설정
            if (robotName_ == "COMBINED" && targetWidget_) {
                targetWidget_->setIndicatorTargetRobot(robot);
                std::cout << "🎯 Set target robot " << robot.toStdString() << " on COMBINED widget" << std::endl;
            }
            
            // ✅ 3. 개별 위젯에서 로봇 위치 가져오기
            glm::vec3 robotPosition(0.0f, 0.0f, 0.0f);
            bool positionFound = false;
            
            if (robotName_ == "COMBINED") {
                // 개별 위젯에서 로봇의 현재 위치 가져오기
                robotPosition = activeWidget->getRobotCurrentPosition(robot);
                positionFound = true;
                std::cout << "📍 Robot " << robot.toStdString() 
                         << " position: (" << robotPosition.x << ", " 
                         << robotPosition.y << ", " << robotPosition.z << ")" << std::endl;
            }
            
            // ✅ 4. 카메라 이동
            if (robotName_ == "COMBINED" && positionFound && targetWidget_) {
                // COMBINED 위젯의 카메라를 로봇 위치로 이동
                targetWidget_->jumpToPosition(robotPosition);
                std::cout << "📷 COMBINED camera jumped to " << robot.toStdString() << " position" << std::endl;
            } else if (activeWidget) {
                // 일반 모드에서는 기존 방식 사용
                activeWidget->jumpToRobotPosition(robot);
                std::cout << "📷 Individual camera jumped to " << robot.toStdString() << " position" << std::endl;
            }
            
            // ✅ 5. 위치 고정 활성화
            activeWidget->setLockIndicatorToCurrentPosition(true);
            
            if (robotName_ == "COMBINED" && targetWidget_) {
                targetWidget_->setLockIndicatorToCurrentPosition(true);
                std::cout << "🔗 COMBINED widget also locked to " << robot.toStdString() << std::endl;
            }
            
            // ✅ 6. 콤보박스도 업데이트 (멤버 변수 사용)
            if (targetRobotCombo_ && targetRobotCombo_->currentText() != robot) {
                targetRobotCombo_->setCurrentText(robot);  // UI 동기화
                std::cout << "🔄 Updated combo box to: " << robot.toStdString() << std::endl;
            }
            
            // ✅ 7. 안전한 타이머로 자동 해제 (3초 후)
            QTimer* autoReleaseTimer = new QTimer();
            autoReleaseTimer->setSingleShot(true);
            
            connect(autoReleaseTimer, &QTimer::timeout, [this, autoReleaseTimer, robot, activeWidget]() {
                if (activeWidget) {
                    activeWidget->setLockIndicatorToCurrentPosition(false);
                    std::cout << "🔓 Auto-release lock for " << robot.toStdString() << std::endl;
                }
                
                if (robotName_ == "COMBINED" && targetWidget_) {
                    targetWidget_->setLockIndicatorToCurrentPosition(false);
                    std::cout << "🔓 Auto-release lock for COMBINED widget" << std::endl;
                }
                
                autoReleaseTimer->deleteLater();
            });
            
            autoReleaseTimer->start(3000);
            
            std::cout << "✅ Quick jump to " << robot.toStdString() << " completed!" << std::endl;
        });
        
        setItemWidget(jumpItem, 1, jumpBtn);
    }
    
    // ✅ 전체 리셋 버튼 (COMBINED 모드 지원)
    auto resetIndicatorItem = new QTreeWidgetItem(parent, {"Reset Indicator"});
    auto resetBtn = createButton("Reset", [this]() {
        // 개별 위젯들 리셋
        if (robotName_ == "COMBINED") {
            QStringList realRobots = {"TUGV", "MUGV", "SUGV1", "SUGV2", "SUAV"};
            for (const QString& robot : realRobots) {
                auto* robotWidget = findRobotWidget(robot);
                if (robotWidget) {
                    robotWidget->setLockIndicatorToCurrentPosition(false);
                    robotWidget->setIndicatorTargetRobot("TUGV");  // 기본값으로 리셋
                }
            }
            std::cout << "🔄 All individual robot indicators reset to default state" << std::endl;
        }
        
        // 현재 위젯도 리셋
        if (targetWidget_) {
            targetWidget_->setLockIndicatorToCurrentPosition(false);
            targetWidget_->setIndicatorTargetRobot("TUGV");  // 기본값으로 리셋
            std::cout << "🔄 Current widget indicator reset to default state" << std::endl;
        }
    });
    setItemWidget(resetIndicatorItem, 1, resetBtn);
    
    // ✅ 현재 상태 표시 (모드별 표시)
    auto statusItem = new QTreeWidgetItem(parent, {"Current Status"});
    auto statusLabel = new QLabel(robotName_ == "COMBINED" ? "COMBINED Mode" : "Individual Mode");
    statusLabel->setStyleSheet(robotName_ == "COMBINED" ? 
                              "color: blue; font-weight: bold;" : 
                              "color: green; font-weight: bold;");
    setItemWidget(statusItem, 1, statusLabel);
}

// ✅ 헬퍼 함수들
QCheckBox* ControlTreeWidget::createCheckBox(bool checked, std::function<void(bool)> callback) {
    auto checkBox = new QCheckBox();
    checkBox->setChecked(checked);
    connect(checkBox, &QCheckBox::toggled, callback);
    return checkBox;
}

QPushButton* ControlTreeWidget::createButton(const QString& text, std::function<void()> callback) {
    auto button = new QPushButton(text);
    button->setMaximumHeight(25);
    connect(button, &QPushButton::clicked, callback);
    return button;
}

void ControlTreeWidget::updateColorButtons() {
    // 색상 버튼들을 현재 설정값으로 업데이트
    if (!targetWidget_) return;
    
    for (const QString& robot : ROBOT_NAMES) {
        // 포인트 색상 버튼 업데이트
        if (colorButtons_.contains(robot + "_points")) {
            glm::vec3 color = targetWidget_->getRobotPointsColor(robot);
            QColor qcolor(color.x * 255, color.y * 255, color.z * 255);
            colorButtons_[robot + "_points"]->setStyleSheet(
                QString("background-color: %1; border: 2px solid white;").arg(qcolor.name()));
        }
        
        // 경로 색상 버튼 업데이트
        if (colorButtons_.contains(robot + "_path")) {
            glm::vec3 color = targetWidget_->getRobotPathColor(robot);
            QColor qcolor(color.x * 255, color.y * 255, color.z * 255);
            colorButtons_[robot + "_path"]->setStyleSheet(
                QString("background-color: %1; border: 2px solid white;").arg(qcolor.name()));
        }
    }
}

void ControlTreeWidget::syncWithWidget() {
    // targetWidget_의 현재 설정값들을 읽어서 UI 동기화
    updateColorButtons();
    // TODO: 다른 설정값들도 동기화
}

// ✅ 슬롯 함수들 (필요한 경우 구현)
void ControlTreeWidget::onItemChanged(QTreeWidgetItem* item, int column) {
    // 아이템 변경 시 처리
}

void ControlTreeWidget::onColorButtonClicked() {
    // 색상 버튼 클릭 시 처리 (위에서 람다로 처리했으므로 비워둠)
}

void ControlTreeWidget::onResetColorsClicked() {
    // 색상 리셋 버튼 클릭 시 처리 (위에서 람다로 처리했으므로 비워둠)
}

void ControlTreeWidget::onCameraPresetClicked() {
    // 카메라 프리셋 버튼 클릭 시 처리 (위에서 람다로 처리했으므로 비워둠)
}

} // namespace Widget

// ✅ MOC 파일 포함 (Q_OBJECT 매크로 사용 시 필요)
#include "control_tree_widget.moc"