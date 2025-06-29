#ifndef POINTCLOUD_WIDGET_H
#define POINTCLOUD_WIDGET_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QTimer>
#include <QObject>
#include <mutex>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "common_types.hpp"
#include "grid_map_processor.hpp"
#include "render_helper.hpp"
#include "interest_object_manager.hpp"

class QPainter;
class QPaintEvent;

namespace Widget {
    // Types 네임스페이스의 타입들 사용
    using CloudConstPtr = Types::CloudConstPtr;
    using PathConstPtr = Types::PathConstPtr;
    using MarkerType = RenderHelper::PositionMarkerType;

    class PointCloudWidget : public QOpenGLWidget, protected QOpenGLFunctions {
        Q_OBJECT
    
    public:
        explicit PointCloudWidget(QWidget *parent = nullptr);
        ~PointCloudWidget();

        // ============================================================================
        // Robot Related Functions
        // ============================================================================
        void setRobot(const QString& robot);
        
        // ============================================================================
        // Color Management Functions (Types::ColorRGB 사용)
        // ============================================================================
        void setRobotPointsColor(const QString& robot, const Types::ColorRGB& color);
        void setRobotPathColor(const QString& robot, const Types::ColorRGB& color);
        Types::ColorRGB getRobotPointsColor(const QString& robot) const;
        Types::ColorRGB getRobotPathColor(const QString& robot) const;
        void resetAllColorsToDefault();
        
        // ============================================================================
        // Camera Control Functions (Types::Vec3 사용)
        // ============================================================================
        void setFocusPoint(const Types::Vec3& focus);
        void setRotationSensitivity(float sensitivity);
        void setTopView(bool enable);
        bool isTopView() const { return isTopView_; }
        void resetCamera();
        void jumpToPosition(const Types::Vec3& position);
        void jumpToRobotPosition(const QString& robotName);
        
        // ============================================================================
        // Indicator Control Functions
        // ============================================================================
        void setLockIndicatorToCurrentPosition(bool lock);
        void setIndicatorTargetRobot(const QString& robot);
        Types::Vec3 getRobotCurrentPosition(const QString& robotName);
        
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
        void setShowObjects(bool show);
        void setShowObjectLabels(bool show);
        // Getter functions
        bool getShowAxes() const { return showAxes_; }
        bool getShowGrid() const { return showGrid_; }
        float getGridSize() const { return cellSize_; }
        int getGridCellCount() const { return planeCellCount_; }
        float getAxesSize() const { return axesLength_; }
        bool getShowPosition() const { return showPosition_; }
        bool getShowRobotLabel() const { return showRobotLabel_; }
        float getPositionRadius() const { return currentPositionRadius_; }
        float getRotationSensitivity() const { return rotationSensitivity_; }
        Types::Vec3 getFocusPoint() const { return focusPoint_; }
        bool getShowObjects() const { return showObjects_; }
        bool getShowObjectLabels() const { return showObjectLabels_; }

        // Point and path style settings
        void setShowGridMap(bool show);
        void setShowPoints(bool show);
        void setShowPath(bool show);
        void setPointSize(float size);
        void setPathWidth(float width);
        float getPointSize() const { return pointSize_; }
        float getPathWidth() const { return pathWidth_; }
        
        // Getter functions
        bool getShowPoints() const { return showPoints_; }
        bool getShowPath() const { return showPath_; }
        bool getShowGridMap() const { return showGridMap_; }
        void setPositionMarkerType(MarkerType type);
        void setGridMapResolution(float resolution);
        float getGridMapResolution() const;
        void setGridMapParameters(const GridMap::GridMapParameters& params);  // Add this line
        void setShowPositionNames(bool show);
        bool getShowPositionNames() const;
        
        // Map Style 제어 함수
        void setMapStyle(const QString& style);
        QString getMapStyle() const { return mapStyle_; }
        
        // Getter functions
        MarkerType getPositionMarkerType() const { return positionMarkerType_; }

        // ============================================================================
        // Interest Objects Functions (전역 매니저 사용으로 간소화)
        // ============================================================================
        void registerInterestObject(const QString& type, const QString& robotName);
        
    public slots:
        // ============================================================================
        // Data Reception Slots
        // ============================================================================
        void onCloudShared(const QString& robot, CloudConstPtr cloud);
        void onPathShared(const QString& robot, PathConstPtr path);
        
        // 전역 매니저 시그널 수신용
        void onGlobalInterestObjectUpdated();
        
    signals:
        // Interest Object 관련 시그널들 제거 (전역 매니저에서 처리)
        // void interestObjectRegistered(...);  // 제거
        // void interestObjectRemoved(...);     // 제거

    protected:
        // Qt Event Overrides
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
        // Robot Data Management (Types 사용)
        // ============================================================================
        QString robotName_;
        QHash<QString, CloudConstPtr> clouds_;
        QHash<QString, PathConstPtr> paths_;
        QHash<QString, Types::Vec3> robotPositions_;  // 로봇 위치 저장용 추가
        mutable std::mutex cloudMutex_;
        mutable std::mutex pathMutex_;
        mutable std::mutex robotPositionsMutex_;      // 로봇 위치 뮤텍스 추가
        
        // ============================================================================
        // Color Management (Types::ColorRGB 사용)
        // ============================================================================
        QHash<QString, Types::ColorRGB> robotPointsColors_;
        QHash<QString, Types::ColorRGB> robotPathColors_;
        void initializeDefaultColors();
        
        // ============================================================================
        // Camera System (Types::Vec3, Mat4 사용)
        // ============================================================================
        Types::Vec3 cameraPos_;
        Types::Vec3 focusPoint_ = Types::Vec3(0.0f, 0.0f, 0.0f);
        float distance_ = 10.0f;
        float yaw_ = 0.0f;
        float pitch_ = 0.0f;
        float rotationSensitivity_ = 0.3f;
        
        // Camera matrices
        Types::Mat4 viewMatrix_;
        Types::Mat4 projectionMatrix_;
        Types::Mat4 rvizToOpenGLMatrix_;
        
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
        Types::Vec3 backupFocusPoint_ = Types::Vec3(0.0f, 0.0f, 0.0f);
        
        // ============================================================================
        // Indicator System
        // ============================================================================
        bool lockIndicatorToCurrentPosition_ = false;
        QString indicatorTargetRobot_ = "";
        Types::Vec3 lastKnownPosition_ = Types::Vec3(0.0f, 0.0f, 0.0f);
        
        // Indicator UI
        bool showIndicator_ = false;
        QTimer hideTimer_;
        const int timerInterval_ = 100;
        QPoint lastMousePos_;
        
        // Indicator related functions
        void updateIndicatorPosition();
        Types::Vec3 getCurrentRobotPosition(const QString& robot) const;
        bool hasValidCurrentPosition(const QString& robot) const;
        void hideIndicator();
        
        // ============================================================================
        // Rendering System
        // ============================================================================
        bool showPosition_ = true;
        MarkerType positionMarkerType_ = MarkerType::AXES;
        float currentPositionRadius_ = 0.3f;
        float currentPositionHeight_ = 0.2f;
        float positionAxesLength_ = 0.5f;
        float positionAxesRadius_ = 0.03f;
        
        // 그리드 맵 관련
        bool showGridMap_ = false;
        QHash<QString, Types::GridMapPtr> gridMaps_;
        mutable std::mutex gridMapMutex_;
        
        // ============================================================================
        // UI Display Options
        // ============================================================================
        bool showAxes_ = true;
        bool showGrid_ = true;
        int planeCellCount_ = 10;
        float cellSize_ = 1.0f;
        float gridLineWidth_ = 0.1f;
        float axesLength_ = 1.0f;
        float axesRadius_ = 0.05f;
        
        bool showRobotLabel_ = true;
        const int fontSize_ = 10;
        const int circleSize_ = 12;
        const int circleMargin_ = 5;
        const int horizontalMargin_ = 8;
        const int verticalMargin_ = 4;
        bool showPositionNames_ = true;
        const float positionNameFontSize_ = 9.0f;
        const float objectLabelFontSize_ = 5.0f;
        bool showPoints_ = true;
        bool showPath_ = true;
        float pointSize_ = 2.0f;
        float pathWidth_ = 3.0f;
        float gridMapResolution_ = 0.05f;
        QString mapStyle_ = "pointcloud";

        
        // ============================================================================
        // Grid Map Functions
        // ============================================================================
        GridMap::GridMapParameters gridParams_;  // Add this line
        void updateGridMapForRobot(const QString& robotName, CloudConstPtr cloud);
        void resampleGridMapResolution(Types::GridMapPtr gridData, float newResolution);

        // ============================================================================
        // Interest Objects
        // ============================================================================
        Types::Vec3 getRobotPosition(const QString& robotName) const;
        void renderInterestObjects();  // 전역 매니저에서 데이터 가져와서 렌더링
        bool showObjects_ = true;
        bool showObjectLabels_ = true;
    };
}

#endif // POINTCLOUD_WIDGET_H