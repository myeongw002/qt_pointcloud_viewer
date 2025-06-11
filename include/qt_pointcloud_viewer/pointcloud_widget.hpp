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
        void setShowAxes(bool show);
        void setShowGrid(bool show);
        void setRotationSensitivity(float sensitivity);
        void setFocusPoint(const glm::vec3& focus);
        void setRobot(const QString& robot);
    
    public slots:
        // 데이터 수신 슬롯들 (여전히 사용)
        void onCloudShared(const QString& robot, CloudConstPtr cloud);
        void onPathShared(const QString& robot, PathConstPtr path);

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
        QHash<QString, CloudConstPtr> clouds_;
        QHash<QString, PathConstPtr> paths_;
        QString robotName_ = "COMBINED";
        std::mutex cloudMutex_;
        std::mutex pathMutex_;

        // 카메라 제어 (여전히 사용)
        glm::mat4 rvizToOpenGLMatrix_; // RViz 좌표를 OpenGL로 변환하는 행렬
        glm::vec3 focusPoint_ = glm::vec3(0.0f, 0.0f, 0.0f);
        float distance_ = 10.0f;
        float yaw_ = 0.0f;
        float pitch_ = 0.0f;
        float rotationSensitivity_ = 0.3f;
        glm::vec3 cameraPos_;
        QPoint lastMousePos_;
        void updateCameraPosition();

        // 렌더링 (여전히 사용)
        glm::mat4 viewMatrix_;
        glm::mat4 projectionMatrix_;
        void drawPoints();
        void drawPath();
        void drawAxes();
        void drawGrid();
        void drawCameraIndicator();
        void setupRvizCoordinateTransform();

        // UI 상태 (여전히 사용)
        bool showIndicator_ = false;
        QTimer hideTimer_;
        const int timerInterval_ = 100;
        bool showAxes_ = true;
        bool showGrid_ = true;
    };
}

Q_DECLARE_METATYPE(Widget::CloudConstPtr)
Q_DECLARE_METATYPE(Widget::PathConstPtr)

#endif // POINTCLOUD_WIDGET_H