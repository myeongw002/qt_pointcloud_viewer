#ifndef VIEWER_SETTINGS_MANAGER_HPP
#define VIEWER_SETTINGS_MANAGER_HPP

#include <QObject>
#include <QHash>
#include <glm/glm.hpp>
#include "pointcloud_widget.hpp"

namespace Widget {

struct ViewerSettings {
    // Display settings
    bool showPoints = true;
    bool showPath = true;
    bool showPosition = true;
    bool showAxes = true;
    bool showGrid = true;
    bool showRobotLabel = true;
    
    // Style settings
    float pointSize = 2.0f;
    float pathWidth = 3.0f;
    float positionRadius = 0.3f;
    float rotationSensitivity = 0.3f;
    float gridSize = 1.0f;          // Grid cell size
    int gridCellCount = 10;         // Grid cell count
    float axesSize = 1.0f;          // Axes size (새로 추가)
    PointCloudWidget::PositionMarkerType markerType = PointCloudWidget::PositionMarkerType::AXES;
    
    // Camera settings
    bool isTopView = false;
    glm::vec3 focusPoint = glm::vec3(0.0f, 0.0f, 0.0f);
    
    // Color settings for each robot
    QHash<QString, glm::vec3> robotPointsColors;
    QHash<QString, glm::vec3> robotPathColors;
};

class ViewerSettingsManager : public QObject {
    Q_OBJECT

public:
    static ViewerSettingsManager* instance();
    
    // Settings management
    void saveSettings(const QString& robotName, PointCloudWidget* widget);
    void loadSettings(const QString& robotName, PointCloudWidget* widget);
    void synchronizeSettings(PointCloudWidget* newWidget, const QString& robotName);
    
    // Global settings for all viewers
    ViewerSettings getGlobalSettings() const { return globalSettings_; }
    void setGlobalSettings(const ViewerSettings& settings);
    
    // Register/unregister widgets for synchronization
    void registerWidget(const QString& robotName, PointCloudWidget* widget);
    void unregisterWidget(PointCloudWidget* widget);

signals:
    void settingsChanged(const QString& robotName, const ViewerSettings& settings);

public slots:
    void onSettingsChanged(const QString& robotName);

private:
    explicit ViewerSettingsManager(QObject* parent = nullptr);
    static ViewerSettingsManager* instance_;
    
    ViewerSettings globalSettings_;
    QHash<QString, ViewerSettings> robotSettings_;
    QHash<PointCloudWidget*, QString> registeredWidgets_;
    
    void initializeDefaultSettings();
    ViewerSettings extractSettings(PointCloudWidget* widget);
    void applySettings(PointCloudWidget* widget, const ViewerSettings& settings);
};

} // namespace Widget

#endif // VIEWER_SETTINGS_MANAGER_HPP