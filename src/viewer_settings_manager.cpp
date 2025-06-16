#include "viewer_settings_manager.hpp"
#include <QDebug>

namespace Widget {


    
ViewerSettingsManager* ViewerSettingsManager::instance_ = nullptr;

ViewerSettingsManager* ViewerSettingsManager::instance() {
    if (!instance_) {
        instance_ = new ViewerSettingsManager();
    }
    return instance_;
}

ViewerSettingsManager::ViewerSettingsManager(QObject* parent)
    : QObject(parent) {
    initializeDefaultSettings();
}

void ViewerSettingsManager::initializeDefaultSettings() {
    // ✅ 기본 생성자만 호출하면 됨 (ViewerSettings의 기본값 사용)
    globalSettings_ = ViewerSettings();
    
    qDebug() << "ViewerSettingsManager: Default settings initialized with proper values";
}

void ViewerSettingsManager::registerWidget(const QString& robotName, PointCloudWidget* widget) {
    if (!widget) return;
    
    // ✅ 중복 등록 방지
    if (registeredWidgets_.contains(widget)) {
        qDebug() << "ViewerSettingsManager: Widget already registered for" << robotName;
        return;
    }
    
    registeredWidgets_[widget] = robotName;
    
    // ✅ 글로벌 기본 설정 적용 (이미 올바른 값들)
    applySettings(widget, globalSettings_);
    
    qDebug() << "ViewerSettingsManager: Registered widget for" << robotName;
}

void ViewerSettingsManager::unregisterWidget(PointCloudWidget* widget) {
    if (registeredWidgets_.contains(widget)) {
        QString robotName = registeredWidgets_[widget];
        registeredWidgets_.remove(widget);
        qDebug() << "ViewerSettingsManager: Unregistered widget for" << robotName;
    }
}

void ViewerSettingsManager::saveSettings(const QString& robotName, PointCloudWidget* widget) {
    if (!widget) return;
    
    ViewerSettings settings = extractSettings(widget);
    robotSettings_[robotName] = settings;
    
    qDebug() << "ViewerSettingsManager: Settings saved for" << robotName;
    
    // ✅ 개별 로봇 설정만 저장, 전파하지 않음
    // globalSettings_ = settings;  // ❌ 제거
    
    // ✅ 다른 위젯에 전파하지 않음
    /*
    for (auto it = registeredWidgets_.begin(); it != registeredWidgets_.end(); ++it) {
        PointCloudWidget* otherWidget = it.key();
        QString otherRobot = it.value();
        
        if (otherWidget == widget) continue;
        
        applySettings(otherWidget, settings);  // ❌ 제거
        qDebug() << "ViewerSettingsManager: Settings propagated to" << otherRobot;
    }
    */
    
    emit settingsChanged(robotName, settings);
}

void ViewerSettingsManager::synchronizeSettings(PointCloudWidget* newWidget, const QString& robotName) {
    if (!newWidget) return;
    
    qDebug() << "ViewerSettingsManager: Starting synchronization for" << robotName;
    
    ViewerSettings settingsToApply;
    
    // ✅ 해당 로봇의 설정이 있으면 사용
    if (robotSettings_.contains(robotName)) {
        settingsToApply = robotSettings_[robotName];
        qDebug() << "ViewerSettingsManager: Using existing settings for" << robotName;
    } else {
        // ✅ 다른 좋은 설정을 찾아보기
        bool foundGoodSettings = false;
        
        for (auto it = registeredWidgets_.begin(); it != registeredWidgets_.end(); ++it) {
            PointCloudWidget* widget = it.key();
            QString widgetRobot = it.value();
            
            if (widget == newWidget) continue;
            
            ViewerSettings candidateSettings = extractSettings(widget);
            
            // ✅ 올바른 설정인지 검증 (기본적인 표시 항목들이 켜져있는지)
            if (candidateSettings.showPoints && candidateSettings.showPath && 
                candidateSettings.pointSize > 0 && candidateSettings.pathWidth > 0) {
                settingsToApply = candidateSettings;
                foundGoodSettings = true;
                qDebug() << "ViewerSettingsManager: Using good settings from" << widgetRobot;
                break;
            }
        }
        
        // ✅ 좋은 설정을 못 찾았으면 글로벌 기본값 사용
        if (!foundGoodSettings) {
            settingsToApply = globalSettings_;  // ✅ 이미 올바른 기본값들
            qDebug() << "ViewerSettingsManager: Using global default settings";
        }
    }
    
    // 새 위젯에 설정 적용
    applySettings(newWidget, settingsToApply);
    
    // 등록
    registerWidget(robotName, newWidget);
    
    qDebug() << "ViewerSettingsManager: Synchronization completed for" << robotName;
}

ViewerSettings ViewerSettingsManager::extractSettings(PointCloudWidget* widget) {
    ViewerSettings settings;
    
    if (!widget) return settings;
    
    // Extract all display settings - ✅ 이제 모든 함수가 사용 가능
    settings.showPoints = widget->getShowPoints();
    settings.showPath = widget->getShowPath();
    settings.showPosition = widget->getShowPosition();
    settings.showAxes = widget->getShowAxes();
    settings.showGrid = widget->getShowGrid();
    settings.showRobotLabel = widget->getShowRobotLabel();
    
    // Extract style settings - ✅ 이제 모든 함수가 사용 가능
    settings.pointSize = widget->getPointSize();
    settings.pathWidth = widget->getPathWidth();
    settings.positionRadius = widget->getPositionRadius();
    settings.rotationSensitivity = widget->getRotationSensitivity();
    settings.markerType = widget->getPositionMarkerType();
    
    // Extract camera settings - ✅ 이제 모든 함수가 사용 가능
    settings.isTopView = widget->isTopView();
    settings.focusPoint = widget->getFocusPoint();
    
    // Extract colors for all robots
    QStringList robots = {"TUGV", "MUGV", "SUGV1", "SUGV2", "SUAV"};
    for (const QString& robot : robots) {
        settings.robotPointsColors[robot] = widget->getRobotPointsColor(robot);
        settings.robotPathColors[robot] = widget->getRobotPathColor(robot);
    }
    
    return settings;
}

void ViewerSettingsManager::applySettings(PointCloudWidget* widget, const ViewerSettings& settings) {
    if (!widget) return;
    
    qDebug() << "ViewerSettingsManager: Applying settings...";
    qDebug() << "  - showPoints:" << settings.showPoints;
    qDebug() << "  - showPath:" << settings.showPath;
    qDebug() << "  - showPosition:" << settings.showPosition;
    qDebug() << "  - showAxes:" << settings.showAxes;
    qDebug() << "  - showGrid:" << settings.showGrid;
    qDebug() << "  - pointSize:" << settings.pointSize;
    qDebug() << "  - pathWidth:" << settings.pathWidth;
    
    // Apply display settings
    widget->setShowPoints(settings.showPoints);
    widget->setShowPath(settings.showPath);
    widget->setShowPosition(settings.showPosition);
    widget->setShowAxes(settings.showAxes);
    widget->setShowGrid(settings.showGrid);
    widget->setShowRobotLabel(settings.showRobotLabel);
    
    // Apply style settings
    widget->setPointSize(settings.pointSize);
    widget->setPathWidth(settings.pathWidth);
    widget->setPositionRadius(settings.positionRadius);
    widget->setRotationSensitivity(settings.rotationSensitivity);
    widget->setPositionMarkerType(settings.markerType);
    
    // Apply camera settings
    widget->setTopView(settings.isTopView);
    widget->setFocusPoint(settings.focusPoint);
    
    // Apply colors
    for (auto it = settings.robotPointsColors.begin(); it != settings.robotPointsColors.end(); ++it) {
        widget->setRobotPointsColor(it.key(), it.value());
    }
    for (auto it = settings.robotPathColors.begin(); it != settings.robotPathColors.end(); ++it) {
        widget->setRobotPathColor(it.key(), it.value());
    }
    
    qDebug() << "ViewerSettingsManager: Settings applied successfully";
}

void ViewerSettingsManager::setGlobalSettings(const ViewerSettings& settings) {
    globalSettings_ = settings;
    
    // Apply to all registered widgets
    for (auto it = registeredWidgets_.begin(); it != registeredWidgets_.end(); ++it) {
        PointCloudWidget* widget = it.key();
        QString robotName = it.value();
        applySettings(widget, settings);
    }
    
    emit settingsChanged("GLOBAL", settings);
    qDebug() << "ViewerSettingsManager: Global settings updated and applied to all widgets";
}

void ViewerSettingsManager::onSettingsChanged(const QString& robotName) {
    // Handle settings change notification
    if (robotSettings_.contains(robotName)) {
        emit settingsChanged(robotName, robotSettings_[robotName]);
    }
}

} // namespace Widget