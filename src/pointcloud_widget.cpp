#include "pointcloud_widget.hpp"
#include <iostream>
#include <QDebug>

namespace Widget {

PointCloudWidget::PointCloudWidget(QWidget *parent) : QOpenGLWidget(parent) {
    lastMousePos_ = QPoint(0, 0);
    showIndicator_ = false;
    connect(&hideTimer_, &QTimer::timeout, this, &PointCloudWidget::hideIndicator);
    setupRvizCoordinateTransform();
    updateCameraPosition();
}

PointCloudWidget::~PointCloudWidget() {
    makeCurrent();
    doneCurrent();
}

void PointCloudWidget::setRobot(const QString& robot) {
    robotName_ = robot;
}

void PointCloudWidget::onCloudShared(const QString& robot, CloudConstPtr cloud) {
    std::lock_guard<std::mutex> lock(cloudMutex_);
    clouds_[robot] = cloud;
    update();
}

void PointCloudWidget::onPathShared(const QString& robot, PathConstPtr path) {
    std::lock_guard<std::mutex> lock(pathMutex_);
    paths_[robot] = path;
    update();
}

void PointCloudWidget::setShowAxes(bool show) {
    showAxes_ = show;
    update();
}

void PointCloudWidget::setShowGrid(bool show) {
    showGrid_ = show;
    update();
}

void PointCloudWidget::setRotationSensitivity(float sensitivity) {
    rotationSensitivity_ = sensitivity;
}

void PointCloudWidget::setFocusPoint(const glm::vec3& focus) {
    focusPoint_ = focus;
    updateCameraPosition();
    update();
}

void PointCloudWidget::initializeGL() {
    initializeOpenGLFunctions();
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
}

void PointCloudWidget::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
    float aspect = float(w) / h;
    projectionMatrix_ = glm::perspective(glm::radians(45.0f), aspect, 0.1f, 1000.0f);
}

void PointCloudWidget::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // ROS 좌표계의 Z축(위쪽)을 OpenGL Y축으로 변환
    glm::vec3 rosUpVector = glm::vec3(0, 0, 1);  // ROS Z축 (위쪽)
    glm::vec3 openglUpVector = glm::vec3(0, 1, 0);  // OpenGL은 Y축이 위

    viewMatrix_ = glm::lookAt(
        cameraPos_,
        focusPoint_,
        openglUpVector  // OpenGL 기준 up vector
    );

    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(glm::value_ptr(projectionMatrix_));
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(glm::value_ptr(viewMatrix_));

    drawPoints();
    drawPath();
    if (showAxes_) drawAxes();
    if (showGrid_) drawGrid();
    if (showIndicator_) drawCameraIndicator();
}

void PointCloudWidget::updateCameraPosition() {
    // ROS 좌표계 기준으로 카메라 위치 계산
    // ROS: X=forward, Y=left, Z=up
    float rosX = distance_ * cos(glm::radians(pitch_)) * cos(glm::radians(yaw_));  // forward
    float rosY = distance_ * cos(glm::radians(pitch_)) * sin(glm::radians(yaw_));  // left
    float rosZ = distance_ * sin(glm::radians(pitch_));                            // up
    
    // ROS → OpenGL 좌표 변환
    float openglX = -rosY;  // ROS Y(left) → OpenGL -X(right)
    float openglY = rosZ;   // ROS Z(up) → OpenGL Y(up)
    float openglZ = -rosX;  // ROS X(forward) → OpenGL -Z(back)
    
    cameraPos_ = focusPoint_ + glm::vec3(openglX, openglY, openglZ);
}

void PointCloudWidget::setupRvizCoordinateTransform() {
    // RViz (x 앞, y 왼쪽, z 위) → OpenGL (x 오른쪽, y 위, z 안쪽) 변환
    // y와 z 교환, z 반전
    rvizToOpenGLMatrix_ = glm::mat4(1.0f);
    rvizToOpenGLMatrix_ = glm::rotate(rvizToOpenGLMatrix_, glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f)); // z → y
    rvizToOpenGLMatrix_ = glm::rotate(rvizToOpenGLMatrix_, glm::radians(-90.0f), glm::vec3(0.0f, 1.0f, 0.0f)); // y → -z
}

void PointCloudWidget::mousePressEvent(QMouseEvent *event) {
    lastMousePos_ = event->pos();
    showIndicator_ = true;
    hideTimer_.stop();
    update();
}

void PointCloudWidget::mouseMoveEvent(QMouseEvent *event) {
    if (event->buttons() & Qt::LeftButton) {
        float deltaX = event->x() - lastMousePos_.x();
        float deltaY = event->y() - lastMousePos_.y();

        // ROS 좌표계 기준 회전
        // 마우스 X 이동 = ROS Yaw 회전 (Z축 중심)
        // 마우스 Y 이동 = ROS Pitch 회전 (Y축 중심)
        yaw_ -= deltaX * rotationSensitivity_;    // 좌우: Yaw (Z축 회전)
        pitch_ += deltaY * rotationSensitivity_;  // 상하: Pitch (Y축 회전)

        // Pitch 제한 (ROS 기준: -90도(아래) ~ +90도(위))
        pitch_ = std::max(-89.9f, std::min(89.9f, pitch_));
        
        lastMousePos_ = event->pos();
        showIndicator_ = true;
        updateCameraPosition();
        update();
        
    } else if (event->buttons() & Qt::MiddleButton) {
        float deltaX = (event->x() - lastMousePos_.x()) * 0.02f;
        float deltaY = (event->y() - lastMousePos_.y()) * 0.02f;

        // OpenGL 좌표계에서 카메라 방향 벡터 계산
        glm::vec3 cameraDir = glm::normalize(cameraPos_ - focusPoint_);
        
        // OpenGL 좌표계 기준으로 right, up 벡터 계산
        glm::vec3 openglUp = glm::vec3(0, 1, 0);  // OpenGL Y축 (위쪽)
        glm::vec3 openglRight = glm::normalize(glm::cross(openglUp, cameraDir)); // 오른쪽
        glm::vec3 openglActualUp = glm::cross(cameraDir, openglRight); // 실제 위쪽
        
        // 포커스 포인트 이동 (OpenGL 좌표계에서 직접)
        focusPoint_ += -(openglRight * deltaX) + (openglActualUp * deltaY);

        lastMousePos_ = event->pos();
        showIndicator_ = true;
        updateCameraPosition();
        update();
    }
}

void PointCloudWidget::mouseReleaseEvent(QMouseEvent *event) {
    Q_UNUSED(event);
    hideTimer_.start(timerInterval_);
}

void PointCloudWidget::wheelEvent(QWheelEvent *event) {
    distance_ -= event->angleDelta().y() * 0.01f;
    distance_ = std::max(1.0f, distance_);
    updateCameraPosition();
    update();
}

void PointCloudWidget::hideIndicator() {
    showIndicator_ = false;
    update();
}

void PointCloudWidget::drawPoints() {
    std::lock_guard<std::mutex> lock(cloudMutex_);

    glPointSize(2.0f);
    glBegin(GL_POINTS);

    for (auto it = clouds_.cbegin(); it != clouds_.cend(); ++it) {
        const QString& robotName = it.key();
        
        if (robotName_ != "COMBINED" && robotName != robotName_) {
            continue;
        }
        
        const auto& cloud = it.value();

        if (cloud && !cloud->empty()) {
            // 로봇별 색상 설정
            if (robotName == "TUGV") {
                glColor3f(1.0f, 0.0f, 0.0f);
            } else if (robotName == "MUGV") {
                glColor3f(0.0f, 1.0f, 0.0f);
            } else if (robotName == "SUGV1") {
                glColor3f(0.0f, 0.0f, 1.0f);
            } else if (robotName == "SUGV2") {
                glColor3f(1.0f, 1.0f, 0.0f);
            } else if (robotName == "SUAV") {
                glColor3f(1.0f, 0.0f, 1.0f);
            } else {
                glColor3f(0.0f, 1.0f, 0.0f);
            }
            
            for (const auto& point : cloud->points) {
                // ROS → OpenGL 좌표 변환
                // ROS: (x=forward, y=left, z=up) → OpenGL: (x=right, y=up, z=back)
                glVertex3f(-point.y,  // ROS Y(left) → OpenGL -X(right)
                          point.z,   // ROS Z(up) → OpenGL Y(up)  
                          -point.x); // ROS X(forward) → OpenGL -Z(back)
            }
        }
    }
    glEnd();
}

void PointCloudWidget::drawPath() {
    std::lock_guard<std::mutex> lock(pathMutex_);
    
    glLineWidth(3.0f);
    glBegin(GL_LINES);
    
    for (auto it = paths_.cbegin(); it != paths_.cend(); ++it) {
        const QString& robotName = it.key();
        
        if (robotName_ != "COMBINED" && robotName != robotName_) {
            continue;
        }
        
        const auto& path = it.value();
        
        if (!path.empty()) {
            // 로봇별 색상 설정 (동일)
            if (robotName == "TUGV") {
                glColor3f(1.0f, 0.0f, 0.0f);
            } else if (robotName == "MUGV") {
                glColor3f(0.0f, 1.0f, 0.0f);
            } else if (robotName == "SUGV1") {
                glColor3f(0.0f, 0.0f, 1.0f);
            } else if (robotName == "SUGV2") {
                glColor3f(1.0f, 1.0f, 0.0f);
            } else if (robotName == "SUAV") {
                glColor3f(1.0f, 0.0f, 1.0f);
            } else {
                glColor3f(0.0f, 1.0f, 0.0f);
            }
            
            for (size_t i = 1; i < path.size(); ++i) {
                const auto& prev_pose = path[i-1];
                const auto& curr_pose = path[i];
                
                // 이전 점 (ROS → OpenGL 변환)
                glVertex3f(-prev_pose.pose.position.y,  // Y → -X
                          prev_pose.pose.position.z,   // Z → Y
                          -prev_pose.pose.position.x); // X → -Z
                
                // 현재 점 (ROS → OpenGL 변환)
                glVertex3f(-curr_pose.pose.position.y,  // Y → -X
                          curr_pose.pose.position.z,   // Z → Y
                          -curr_pose.pose.position.x); // X → -Z
            }
        }
    }
    
    glEnd();
}

void PointCloudWidget::drawAxes() {
    glBegin(GL_LINES);
    
    // ROS 표준 좌표축 (REP-103)
    // X축: 빨간색, 앞방향 (OpenGL에서는 -Z)
    glColor3f(1.0f, 0.0f, 0.0f); 
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, -1.0f); // X(forward) → -Z
    
    // Y축: 초록색, 왼쪽 (OpenGL에서는 -X)
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(-1.0f, 0.0f, 0.0f); // Y(left) → -X
    
    // Z축: 파란색, 위쪽 (OpenGL에서도 +Y)
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 1.0f, 0.0f); // Z(up) → Y
    
    glEnd();
}

void PointCloudWidget::drawGrid() {
    glColor3f(0.3f, 0.3f, 0.3f);
    glBegin(GL_LINES);
    
    // XY 평면 그리드 (ROS 기준: X=forward, Y=left)
    for (float i = -10.0f; i <= 10.0f; i += 1.0f) {
        // Y 방향 선들 (OpenGL -X 방향)
        glVertex3f(-10.0f, 0.0f, -i);  // X 라인
        glVertex3f(10.0f, 0.0f, -i);
        
        // X 방향 선들 (OpenGL -Z 방향)  
        glVertex3f(-i, 0.0f, -10.0f);  // Y 라인
        glVertex3f(-i, 0.0f, 10.0f);
    }
    glEnd();
}

void PointCloudWidget::drawCameraIndicator() {
    glDisable(GL_DEPTH_TEST);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();

    // focusPoint_는 이미 OpenGL 좌표계 값이므로 직접 사용
    glTranslatef(focusPoint_.x, focusPoint_.y, focusPoint_.z);

    float cylinderRadius = 0.3f;
    float cylinderHeight = 0.1f;
    float alpha = 0.7f;
    int segments = 30;

    glColor4f(1.0f, 1.0f, 0.0f, alpha); 
    
    // Top Disk (ROS Z축 기준으로 위쪽, OpenGL에서는 Y축)
    glBegin(GL_TRIANGLE_FAN);
    glVertex3f(0.0f, cylinderHeight / 2.0f, 0.0f);
    for (int i = 0; i <= segments; ++i) {
        float angle = 2.0f * M_PI * float(i) / float(segments);
        float x = cylinderRadius * cos(angle);
        float z = cylinderRadius * sin(angle);
        glVertex3f(x, cylinderHeight / 2.0f, z);
    }
    glEnd();

    // Bottom Disk (ROS Z축 기준으로 아래쪽, OpenGL에서는 -Y축)
    glBegin(GL_TRIANGLE_FAN);
    glVertex3f(0.0f, -cylinderHeight / 2.0f, 0.0f);
    for (int i = 0; i <= segments; ++i) {
        float angle = 2.0f * M_PI * float(i) / float(segments);
        float x = cylinderRadius * cos(angle);
        float z = cylinderRadius * sin(angle);
        glVertex3f(x, -cylinderHeight / 2.0f, z);
    }
    glEnd();

    // Side Surface (ROS Z축 방향으로 연결되는 측면, OpenGL에서는 Y축)
    glBegin(GL_QUAD_STRIP);
    for (int i = 0; i <= segments; ++i) {
        float angle = 2.0f * M_PI * float(i) / float(segments);
        float x = cylinderRadius * cos(angle);
        float z = cylinderRadius * sin(angle);

        // 아래쪽 점
        glVertex3f(x, -cylinderHeight / 2.0f, z);
        // 위쪽 점  
        glVertex3f(x, cylinderHeight / 2.0f, z);
    }
    glEnd();

    glPopMatrix();
    glEnable(GL_DEPTH_TEST);
}

} // namespace Widget