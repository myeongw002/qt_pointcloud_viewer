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

class QPainter;
class QPaintEvent;

namespace Widget {
    using Cloud = pcl::PointCloud<pcl::PointXYZI>;
    using CloudConstPtr = Cloud::ConstPtr;
    using PathConstPtr = std::vector<geometry_msgs::msg::PoseStamped>;
    
    class PointCloudWidget : public QOpenGLWidget, protected QOpenGLFunctions {
        Q_OBJECT
    
    public:
        explicit PointCloudWidget(QWidget *parent = nullptr);
        ~PointCloudWidget();
    
        enum class PositionMarkerType {
            CYLINDER,
            AXES
        };

        // ============================================================================
        // 🤖 로봇 관련 함수들
        // ============================================================================
        void setRobot(const QString& robot);
        
        // ============================================================================
        // 🎨 색상 관리 함수들
        // ============================================================================
        void setRobotPointsColor(const QString& robot, const glm::vec3& color);
        void setRobotPathColor(const QString& robot, const glm::vec3& color);
        glm::vec3 getRobotPointsColor(const QString& robot) const;
        glm::vec3 getRobotPathColor(const QString& robot) const;
        void resetAllColorsToDefault();
        
        // ============================================================================
        // 📷 카메라 제어 함수들
        // ============================================================================
        void setFocusPoint(const glm::vec3& focus);
        void setRotationSensitivity(float sensitivity);
        void setTopView(bool enable);
        bool isTopView() const { return isTopView_; }
        void resetCamera();
        void jumpToPosition(const glm::vec3& position);
        void jumpToRobotPosition(const QString& robotName);
        
        // ============================================================================
        // 🎯 인디케이터 제어 함수들
        // ============================================================================
        void setLockIndicatorToCurrentPosition(bool lock);
        void setIndicatorTargetRobot(const QString& robot);
        glm::vec3 getRobotCurrentPosition(const QString& robotName);
        
        // ============================================================================
        // 🎭 표시 옵션 함수들
        // ============================================================================
        void setShowAxes(bool show);
        void setShowGrid(bool show);
        void setShowPosition(bool show);
        void setShowRobotLabel(bool show);
        void setPositionRadius(float radius);
        bool isShowPosition() const { return showPosition_; }
        // ✅ 포인트 및 경로 스타일 설정
        void setShowPoints(bool show);
        void setShowPath(bool show);
        void setPointSize(float size);
        void setPathWidth(float width);
        float getPointSize() const { return pointSize_; }
        float getPathWidth() const { return pathWidth_; }
        void setPositionMarkerType(PositionMarkerType type);

    public slots:
        // ============================================================================
        // 📡 데이터 수신 슬롯들
        // ============================================================================
        void onCloudShared(const QString& robot, CloudConstPtr cloud);
        void onPathShared(const QString& robot, PathConstPtr path);
        
    protected:
        // ============================================================================
        // 🖥️ Qt 이벤트 오버라이드
        // ============================================================================
        void initializeGL() override;
        void paintGL() override;
        void resizeGL(int w, int h) override;
        void paintEvent(QPaintEvent* event) override;
        
        // 마우스/키보드 이벤트
        void mousePressEvent(QMouseEvent *event) override;
        void mouseReleaseEvent(QMouseEvent *event) override;
        void mouseMoveEvent(QMouseEvent *event) override;
        void wheelEvent(QWheelEvent *event) override;

    private:
        // ============================================================================
        // 🤖 로봇 데이터 관리
        // ============================================================================
        QString robotName_ = "";
        QHash<QString, CloudConstPtr> clouds_;
        QHash<QString, PathConstPtr> paths_;
        mutable std::mutex cloudMutex_;
        mutable std::mutex pathMutex_;
        
        // ============================================================================
        // 🎨 색상 관리
        // ============================================================================
        QHash<QString, glm::vec3> robotPointsColors_;
        QHash<QString, glm::vec3> robotPathColors_;
        void initializeDefaultColors();
        
        // ============================================================================
        // 📷 카메라 시스템
        // ============================================================================
        // 카메라 위치 및 방향
        glm::vec3 cameraPos_;
        glm::vec3 focusPoint_ = glm::vec3(0.0f, 0.0f, 0.0f);
        float distance_ = 10.0f;
        float yaw_ = 0.0f;
        float pitch_ = 0.0f;
        float rotationSensitivity_ = 0.3f;
        
        // 카메라 매트릭스
        glm::mat4 viewMatrix_;
        glm::mat4 projectionMatrix_;
        glm::mat4 rvizToOpenGLMatrix_;
        
        // 카메라 제어 함수들
        void updateCameraPosition();
        void updateTopViewCamera();
        void backupCameraState();
        void restoreCameraState();
        
        // 탑뷰 관련
        bool isTopView_ = false;
        float topViewHeight_ = 20.0f;
        float topViewZoom_ = 1.0f;
        float backupDistance_ = 10.0f;
        float backupYaw_ = 0.0f;
        float backupPitch_ = 0.0f;
        glm::vec3 backupFocusPoint_ = glm::vec3(0.0f, 0.0f, 0.0f);
        
        // ============================================================================
        // 🎯 인디케이터 시스템
        // ============================================================================
        bool lockIndicatorToCurrentPosition_ = false;
        QString indicatorTargetRobot_ = "";
        glm::vec3 lastKnownPosition_ = glm::vec3(0.0f, 0.0f, 0.0f);
        
        // 인디케이터 UI
        bool showIndicator_ = false;
        QTimer hideTimer_;
        const int timerInterval_ = 100;
        QPoint lastMousePos_;
        
        // 인디케이터 관련 함수들
        void updateIndicatorPosition();
        glm::vec3 getCurrentRobotPosition(const QString& robot) const;
        bool hasValidCurrentPosition(const QString& robot) const;
        void hideIndicator();
        
        // ============================================================================
        // 🎭 렌더링 시스템
        // ============================================================================
        // 기본 렌더링 함수들
        void drawPoints();
        void drawPath();
        void drawAxes();
        void drawGrid();
        void drawCameraIndicator();
        
        // 위치 마커 관련

        bool showPosition_ = true;
        PositionMarkerType positionMarkerType_ = PositionMarkerType::AXES;
        float currentPositionRadius_ = 0.3f;
        float currentPositionHeight_ = 0.2f;
        float positionAxesLength_ = 0.5f;
        float positionAxesRadius_ = 0.03f;
        
        void drawPositions();
        void drawCylinderMarker(const glm::vec3& position, const glm::vec3& robotColor, const QString& robotName);
        void drawPositionAxes(const glm::vec3& position, const glm::quat& orientation, const QString& robotName);
        void drawCustomAxes(const glm::vec3& position, const glm::quat& orientation);
        
        // ============================================================================
        // 🎨 UI 표시 옵션
        // ============================================================================
        // 축과 그리드
        bool showAxes_ = true;
        bool showGrid_ = true;
        int planeCellCount_ = 10;
        float cellSize_ = 1.0f;
        float gridLineWidth_ = 0.1f;
        float axesLength_ = 1.0f;
        float axesRadius_ = 0.05f;
        
        // 로봇 라벨
        bool showRobotLabel_ = true;
        const int fontSize_ = 10;
        const int circleSize_ = 12;
        const int circleMargin_ = 5;
        const int horizontalMargin_ = 8;
        const int verticalMargin_ = 4;
        
        void drawRobotLabel(QPainter& painter);
        void drawSingleLabel(QPainter& painter, const QString& text, const QColor& color, const QPoint& pos);
        

        bool showPoints_ = true;  // 포인트 표시 여부
        bool showPath_ = true;    // 경로 표시 여부
        // ✅ 포인트 및 경로 스타일 변수
        float pointSize_ = 2.0f;      // 포인트 크기
        float pathWidth_ = 3.0f;      // 경로 선 두께
    };
}

Q_DECLARE_METATYPE(Widget::CloudConstPtr)
Q_DECLARE_METATYPE(Widget::PathConstPtr)

#endif // POINTCLOUD_WIDGET_H