#ifndef POINTCLOUD_WIDGET_H
#define POINTCLOUD_WIDGET_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QTimer>
#include <QObject>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <mutex>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "grid_map_processor.hpp"
#include "pointcloud_widget.hpp"
#include "render_helper.hpp"  // 이 include를 맨 위로 이동

class QPainter;
class QPaintEvent;

namespace Widget {
    using Cloud = pcl::PointCloud<pcl::PointXYZI>;
    using CloudConstPtr = Cloud::ConstPtr;
    using PathConstPtr = std::vector<geometry_msgs::msg::PoseStamped>;
    
    // RenderHelper의 타입을 사용하도록 변경
    using PositionMarkerType = RenderHelper::PositionMarkerType;
    
    class PointCloudWidget : public QOpenGLWidget, protected QOpenGLFunctions {
        Q_OBJECT
    
    public:
        explicit PointCloudWidget(QWidget *parent = nullptr);
        ~PointCloudWidget();

        // 기존 enum 제거하고 using 선언으로 대체
        // enum class PositionMarkerType {
        //     CYLINDER,
        //     AXES
        // }; // 이 부분 제거

        // ============================================================================
        // Robot Related Functions
        // ============================================================================
        void setRobot(const QString& robot);
        
        // ============================================================================
        // Color Management Functions
        // ============================================================================
        void setRobotPointsColor(const QString& robot, const glm::vec3& color);
        void setRobotPathColor(const QString& robot, const glm::vec3& color);
        glm::vec3 getRobotPointsColor(const QString& robot) const;
        glm::vec3 getRobotPathColor(const QString& robot) const;
        void resetAllColorsToDefault();
        
        // ============================================================================
        // Camera Control Functions
        // ============================================================================
        void setFocusPoint(const glm::vec3& focus);
        void setRotationSensitivity(float sensitivity);
        void setTopView(bool enable);
        bool isTopView() const { return isTopView_; }
        void resetCamera();
        void jumpToPosition(const glm::vec3& position);
        void jumpToRobotPosition(const QString& robotName);
        
        // ============================================================================
        // Indicator Control Functions
        // ============================================================================
        void setLockIndicatorToCurrentPosition(bool lock);
        void setIndicatorTargetRobot(const QString& robot);
        glm::vec3 getRobotCurrentPosition(const QString& robotName);
        
        // ============================================================================
        // Display Option Functions
        // ============================================================================
        void setShowAxes(bool show);
        void setShowGrid(bool show);
        void setGridSize(float size);
        void setGridCellCount(int count);
        void setAxesSize(float size);
        void setShowPosition(bool show);
        void setShowRobotLabel(bool show);
        void setPositionRadius(float radius);
        bool isShowPosition() const { return showPosition_; }
        
        // Added Getter functions
        bool getShowAxes() const { return showAxes_; }
        bool getShowGrid() const { return showGrid_; }
        float getGridSize() const { return cellSize_; }
        int getGridCellCount() const { return planeCellCount_; }
        float getAxesSize() const { return axesLength_; }
        bool getShowPosition() const { return showPosition_; }
        bool getShowRobotLabel() const { return showRobotLabel_; }
        float getPositionRadius() const { return currentPositionRadius_; }
        float getRotationSensitivity() const { return rotationSensitivity_; }
        glm::vec3 getFocusPoint() const { return focusPoint_; }

        // Point and path style settings
        void setShowPoints(bool show);
        void setShowPath(bool show);
        void setPointSize(float size);
        void setPathWidth(float width);
        float getPointSize() const { return pointSize_; }
        float getPathWidth() const { return pathWidth_; }
        
        // Added Getter functions
        bool getShowPoints() const { return showPoints_; }
        bool getShowPath() const { return showPath_; }
        
        void setPositionMarkerType(PositionMarkerType type);
        
        // Added Getter functions
        PositionMarkerType getPositionMarkerType() const { return positionMarkerType_; }
    
    public slots:
        // ============================================================================
        // Data Reception Slots
        // ============================================================================
        void onCloudShared(const QString& robot, CloudConstPtr cloud);
        void onPathShared(const QString& robot, PathConstPtr path);
        
    protected:
        // ============================================================================
        // Qt Event Overrides
        // ============================================================================
        void initializeGL() override;
        void paintGL() override;
        void resizeGL(int w, int h) override;
        void paintEvent(QPaintEvent* event) override;
        
        // Mouse/keyboard events
        void mousePressEvent(QMouseEvent *event) override;
        void mouseReleaseEvent(QMouseEvent *event) override;
        void mouseMoveEvent(QMouseEvent *event) override;
        void wheelEvent(QWheelEvent *event) override;
        void keyPressEvent(QKeyEvent *event) override;

    private:
        // ============================================================================
        // Robot Data Management
        // ============================================================================
        QString robotName_ = "";
        QHash<QString, CloudConstPtr> clouds_;
        QHash<QString, PathConstPtr> paths_;
        mutable std::mutex cloudMutex_;
        mutable std::mutex pathMutex_;
        
        // ============================================================================
        // Color Management
        // ============================================================================
        QHash<QString, glm::vec3> robotPointsColors_;
        QHash<QString, glm::vec3> robotPathColors_;
        void initializeDefaultColors();
        
        // ============================================================================
        // Camera System
        // ============================================================================
        // Camera position and orientation
        glm::vec3 cameraPos_;
        glm::vec3 focusPoint_ = glm::vec3(0.0f, 0.0f, 0.0f);
        float distance_ = 10.0f;
        float yaw_ = 0.0f;
        float pitch_ = 0.0f;
        float rotationSensitivity_ = 0.3f;
        
        // Camera matrices
        glm::mat4 viewMatrix_;
        glm::mat4 projectionMatrix_;
        glm::mat4 rvizToOpenGLMatrix_;
        
        // Camera control functions
        void updateCameraPosition();
        void updateTopViewCamera();
        void backupCameraState();
        void restoreCameraState();
        
        // Top view related
        bool isTopView_ = false;
        float topViewHeight_ = 20.0f;
        float topViewZoom_ = 1.0f;
        float backupDistance_ = 10.0f;
        float backupYaw_ = 0.0f;
        float backupPitch_ = 0.0f;
        glm::vec3 backupFocusPoint_ = glm::vec3(0.0f, 0.0f, 0.0f);
        
        // ============================================================================
        // Indicator System
        // ============================================================================
        bool lockIndicatorToCurrentPosition_ = false;
        QString indicatorTargetRobot_ = "";
        glm::vec3 lastKnownPosition_ = glm::vec3(0.0f, 0.0f, 0.0f);
        
        // Indicator UI
        bool showIndicator_ = false;
        QTimer hideTimer_;
        const int timerInterval_ = 100;
        QPoint lastMousePos_;
        
        // Indicator related functions
        void updateIndicatorPosition();
        glm::vec3 getCurrentRobotPosition(const QString& robot) const;
        bool hasValidCurrentPosition(const QString& robot) const;
        void hideIndicator();
        
        // ============================================================================
        // Rendering System
        // ============================================================================
        // Position marker related
        bool showPosition_ = true;
        PositionMarkerType positionMarkerType_ = PositionMarkerType::AXES;  // RenderHelper 타입 사용
        float currentPositionRadius_ = 0.3f;
        float currentPositionHeight_ = 0.2f;
        float positionAxesLength_ = 0.5f;
        float positionAxesRadius_ = 0.03f;
        
        // 그리드 맵 관련
        bool showGridMap_ = false;
        GridMap::GridMapParameters gridParams_;
        QHash<QString, std::shared_ptr<GridMap::GridMapData>> gridMaps_;
        mutable std::mutex gridMapMutex_;
        
        // ============================================================================
        // UI Display Options
        // ============================================================================
        // Axes and grid
        bool showAxes_ = true;
        bool showGrid_ = true;
        int planeCellCount_ = 10;
        float cellSize_ = 1.0f;
        float gridLineWidth_ = 0.1f;
        float axesLength_ = 1.0f;
        float axesRadius_ = 0.05f;
        
        // Robot label
        bool showRobotLabel_ = true;
        const int fontSize_ = 10;
        const int circleSize_ = 12;
        const int circleMargin_ = 5;
        const int horizontalMargin_ = 8;
        const int verticalMargin_ = 4;
        
        // Point and path display settings
        bool showPoints_ = true;
        bool showPath_ = true;
        float pointSize_ = 2.0f;
        float pathWidth_ = 3.0f;

        // ============================================================================
        // Grid Map Functions
        // ============================================================================
        void setShowGridMap(bool show);
        void setGridMapParameters(const GridMap::GridMapParameters& params);
        void updateGridMapForRobot(const QString& robotName, CloudConstPtr cloud);
        bool getShowGridMap() const { return showGridMap_; }
        GridMap::GridMapParameters getGridMapParameters() const { return gridParams_; }
    };
}

Q_DECLARE_METATYPE(Widget::CloudConstPtr)
Q_DECLARE_METATYPE(Widget::PathConstPtr)

#endif // POINTCLOUD_WIDGET_H