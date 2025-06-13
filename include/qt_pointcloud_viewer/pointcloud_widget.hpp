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
        
        // UI 제어 함수들 (여전히 사용)
        void setRobot(const QString& robot);
        void setShowAxes(bool show);
        void setShowGrid(bool show);
        void setRotationSensitivity(float sensitivity);
        void setFocusPoint(const glm::vec3& focus);
        void setShowPosition(bool show);
        void setPositionRadius(float radius);
        bool isShowPosition() const { return showPosition_; }
        void setShowRobotLabel(bool show);
        // 탑뷰 제어 함수 추가
        void setTopView(bool enable);
        bool isTopView() const { return isTopView_; }
        void resetCamera();
        // 색상 설정/가져오기 함수들
        void setRobotPointsColor(const QString& robot, const glm::vec3& color);
        void setRobotPathColor(const QString& robot, const glm::vec3& color);
        glm::vec3 getRobotPointsColor(const QString& robot) const;
        glm::vec3 getRobotPathColor(const QString& robot) const;
        
        // 모든 로봇 색상 초기화
        void resetAllColorsToDefault();

        // ✅ 새로 추가할 함수들
        glm::vec3 getRobotCurrentPosition(const QString& robotName);
        void jumpToPosition(const glm::vec3& position);
        void jumpToRobotPosition(const QString& robotName);
    
    public slots:
        // 데이터 수신 슬롯들 (여전히 사용)
        void onCloudShared(const QString& robot, CloudConstPtr cloud);
        void onPathShared(const QString& robot, PathConstPtr path);
        void setLockIndicatorToCurrentPosition(bool lock);
        void setIndicatorTargetRobot(const QString& robot);
        
    protected:
        // OpenGL 렌더링 (여전히 사용)
        void initializeGL() override;
        void paintGL() override;
        void resizeGL(int w, int h) override;
        void mousePressEvent(QMouseEvent *event) override;
        void mouseReleaseEvent(QMouseEvent *event) override;
        void mouseMoveEvent(QMouseEvent *event) override;
        void wheelEvent(QWheelEvent *event) override;
        void hideIndicator();

    private:
        // 멀티 로봇 데이터 (여전히 사용)
        QString robotName_ = "";
        QHash<QString, CloudConstPtr> clouds_;
        QHash<QString, PathConstPtr> paths_;
        mutable std::mutex cloudMutex_;   // ✅ mutable 추가
        mutable std::mutex pathMutex_;    // ✅ mutable 추가

        // 카메라 제어 
        glm::mat4 rvizToOpenGLMatrix_; // RViz 좌표를 OpenGL로 변환하는 행렬
        glm::vec3 focusPoint_ = glm::vec3(0.0f, 0.0f, 0.0f);
        float distance_ = 10.0f;
        float yaw_ = 0.0f;
        float pitch_ = 0.0f;
        float rotationSensitivity_ = 0.3f;
        glm::vec3 cameraPos_;
        QPoint lastMousePos_;
        // 탑뷰 관련 변수 추가
        float topViewHeight_ = 20.0f;       // 탑뷰 높이
        float topViewZoom_ = 1.0f;          // 탑뷰 줌 레벨
        
        // 일반 뷰 복원용 백업 변수들
        float backupDistance_ = 10.0f;
        float backupYaw_ = 0.0f;
        float backupPitch_ = 0.0f;
        glm::vec3 backupFocusPoint_ = glm::vec3(0.0f, 0.0f, 0.0f);

        void updateCameraPosition();
        void updateTopViewCamera();
        void backupCameraState();
        void restoreCameraState();

        // 렌더링 
        glm::mat4 viewMatrix_;
        glm::mat4 projectionMatrix_;
        void drawPoints();
        void drawPath();
        void drawAxes();
        void drawGrid();
        void drawPositions();
        void drawCameraIndicator();
        void paintEvent(QPaintEvent* event);
        void drawRobotLabel(QPainter& painter);
        void drawSingleLabel(QPainter& painter, const QString& text, const QColor& color, const QPoint& pos);
        // UI 상태 
        bool showIndicator_ = false;
        QTimer hideTimer_;
        const int timerInterval_ = 100;
        bool isTopView_ = false;
        bool showRobotLabel_ = true; // 로봇 이름 표시 여부
        bool showAxes_ = true;
        bool showGrid_ = true;
        int planeCellCount_ = 10;
        float cellSize_ = 1.0f;
        float gridLineWidth_ = 0.1f;
        float axesLength_ = 1.0f;
        float axesRadius_ = 0.05f;
        bool showPosition_ = true; // 로봇 위치 표시 여부
        float currentPositionRadius_ = 0.3f;        // 현재 위치 마커 크기
        float currentPositionHeight_ = 0.2f;        // 실린더 높이

        // 마커 유형 변수 추가
        enum class PositionMarkerType {
            CYLINDER,  // 실린더 마커
            AXES       // 축 마커
        };
        PositionMarkerType positionMarkerType_ = PositionMarkerType::AXES;

        void setPositionMarkerType(PositionMarkerType type);
        void drawCylinderMarker(const glm::vec3& position, const glm::vec3& robotColor, const QString& robotName);
        void drawPositionAxes(const glm::vec3& position, const glm::quat& orientation, const QString& robotName);
        void drawCustomAxes(const glm::vec3& position, const glm::quat& orientation);
        // 축 마커 관련 변수
        float positionAxesLength_ = 0.5f;
        float positionAxesRadius_ = 0.03f;

        // 로봇별 색상 관리
        QHash<QString, glm::vec3> robotPointsColors_;
        QHash<QString, glm::vec3> robotPathColors_;
        void initializeDefaultColors();

        //Painter 설정
        const int fontSize_ = 10; // 폰트 크기
        const int circleSize_ = 12;
        const int circleMargin_ = 5;     // 원과 텍스트 사이 간격
        const int horizontalMargin_ = 8;      // 좌우 여백
        const int verticalMargin_ = 4;    // 상하 여백

        // 카메라 인디케이터 관련 변수
        bool lockIndicatorToCurrentPosition_ = false;  // 현재 위치에 고정 여부
        QString indicatorTargetRobot_ = "";            // 추적할 로봇 이름
        glm::vec3 lastKnownPosition_ = glm::vec3(0.0f, 0.0f, 0.0f);  // 마지막 알려진 위치

        void updateIndicatorPosition();
        glm::vec3 getCurrentRobotPosition(const QString& robot) const;
        bool hasValidCurrentPosition(const QString& robot) const;


    };
}

Q_DECLARE_METATYPE(Widget::CloudConstPtr)
Q_DECLARE_METATYPE(Widget::PathConstPtr)

#endif // POINTCLOUD_WIDGET_H