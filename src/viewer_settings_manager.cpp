#include "viewer_settings_manager.hpp"
#include "pointcloud_widget.hpp"  // 이 줄 추가!
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
    // Just call default constructor (use ViewerSettings default values)
    globalSettings_ = ViewerSettings();
    
    qDebug() << "ViewerSettingsManager: Default settings initialized with proper values";
}

void ViewerSettingsManager::registerWidget(const QString& robotName, PointCloudWidget* widget) {
    if (!widget) return;
    
    // Prevent duplicate registration
    if (registeredWidgets_.contains(widget)) {
        qDebug() << "ViewerSettingsManager: Widget already registered for" << robotName;
        return;
    }
    
    registeredWidgets_[widget] = robotName;
    
    // Apply global default settings (already correct values)
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
    
    emit settingsChanged(robotName, settings);
}

void ViewerSettingsManager::synchronizeSettings(PointCloudWidget* newWidget, const QString& robotName) {
    if (!newWidget) return;
    
    qDebug() << "ViewerSettingsManager: Starting synchronization for" << robotName;
    
    ViewerSettings settingsToApply;
    
    // Use existing robot settings if available
    if (robotSettings_.contains(robotName)) {
        settingsToApply = robotSettings_[robotName];
        qDebug() << "ViewerSettingsManager: Using existing settings for" << robotName;
    } else {
        // Try to find other good settings
        bool foundGoodSettings = false;
        
        for (auto it = registeredWidgets_.begin(); it != registeredWidgets_.end(); ++it) {
            PointCloudWidget* widget = it.key();
            QString widgetRobot = it.value();
            
            if (widget == newWidget) continue;
            
            ViewerSettings candidateSettings = extractSettings(widget);
            
            // Validate if settings are correct (basic display items are enabled)
            if (candidateSettings.showPoints && candidateSettings.showPath && 
                candidateSettings.pointSize > 0 && candidateSettings.pathWidth > 0) {
                settingsToApply = candidateSettings;
                foundGoodSettings = true;
                qDebug() << "ViewerSettingsManager: Using good settings from" << widgetRobot;
                break;
            }
        }
        
        // Use global default values if no good settings found
        if (!foundGoodSettings) {
            settingsToApply = globalSettings_;  // Already correct default values
            qDebug() << "ViewerSettingsManager: Using global default settings";
        }
    }
    
    // Apply settings to new widget
    applySettings(newWidget, settingsToApply);
    
    // Register widget
    registerWidget(robotName, newWidget);
    
    qDebug() << "ViewerSettingsManager: Synchronization completed for" << robotName;
}

ViewerSettings ViewerSettingsManager::extractSettings(PointCloudWidget* widget) {
    ViewerSettings settings;
    
    if (!widget) return settings;
    
    // Extract all display settings
    settings.showPoints = widget->getShowPoints();
    settings.showPath = widget->getShowPath();
    settings.showPosition = widget->getShowPosition();
    settings.showAxes = widget->getShowAxes();
    settings.showGrid = widget->getShowGrid();
    settings.showRobotLabel = widget->getShowRobotLabel();
    settings.showGridMap = widget->getShowGridMap();  // 새로 추가
    
    // Extract style settings
    settings.pointSize = widget->getPointSize();
    settings.pathWidth = widget->getPathWidth();
    settings.positionRadius = widget->getPositionRadius();
    settings.rotationSensitivity = widget->getRotationSensitivity();
    settings.gridSize = widget->getGridSize();
    settings.gridCellCount = widget->getGridCellCount();
    settings.axesSize = widget->getAxesSize();
    settings.markerType = widget->getPositionMarkerType();
    settings.mapStyle = widget->getMapStyle();  // 새로 추가
    
    // Extract camera settings
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
    qDebug() << "  - showAxes:" << settings.showAxes;
    qDebug() << "  - axesSize:" << settings.axesSize;
    qDebug() << "  - showGrid:" << settings.showGrid;
    qDebug() << "  - gridSize:" << settings.gridSize;
    qDebug() << "  - gridCellCount:" << settings.gridCellCount;
    
    // Apply display settings
    widget->setShowPoints(settings.showPoints);
    widget->setShowPath(settings.showPath);
    widget->setShowPosition(settings.showPosition);
    widget->setShowAxes(settings.showAxes);
    widget->setShowGrid(settings.showGrid);
    widget->setShowRobotLabel(settings.showRobotLabel);
    widget->setShowGridMap(settings.showGridMap);  // 새로 추가
    
    // Apply style settings
    widget->setPointSize(settings.pointSize);
    widget->setPathWidth(settings.pathWidth);
    widget->setPositionRadius(settings.positionRadius);
    widget->setRotationSensitivity(settings.rotationSensitivity);
    widget->setGridSize(settings.gridSize);
    widget->setGridCellCount(settings.gridCellCount);
    widget->setAxesSize(settings.axesSize);
    widget->setPositionMarkerType(settings.markerType);
    widget->setMapStyle(settings.mapStyle);  // 새로 추가
    
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