#include "pointcloud_widget.hpp"
#include "shape_helper.hpp"
#include <iostream>
#include <QDebug>
#include <QEvent>
#include <QPainter>
#include <QPaintEvent>
#include <QPen>
#include <QBrush>
#include <QFont>
#include <QColor>
#include <QRect>


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
    bool blendWasEnabled = glIsEnabled(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glClearColor(0.15f, 0.15f, 0.15f, 1.0f);
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
    
    // X축: 빨간색 실린더
    ShapeHelper::SimpleShape::drawCylinder(
        glm::vec3(0, 0, -axesLength_/2),  // 중심점
        glm::vec3(0, 0, -1),             // 방향 (ROS X → OpenGL -Z)
        axesLength_, 
        axesRadius_, 
        glm::vec4(1.0f, 0.0f, 0.0f, 1.0f)     // 빨간색
    );
    
    // Y축: 초록색 실린더
    ShapeHelper::SimpleShape::drawCylinder(
        glm::vec3(-axesLength_/2, 0, 0),
        glm::vec3(-1, 0, 0),             // ROS Y → OpenGL -X
        axesLength_,
        axesRadius_,
        glm::vec4(0.0f, 1.0f, 0.0f, 1.0f)     // 초록색
    );
    
    // Z축: 파란색 실린더
    ShapeHelper::SimpleShape::drawCylinder(
        glm::vec3(0, axesLength_/2, 0),
        glm::vec3(0, 1, 0),              // ROS Z → OpenGL Y
        axesLength_,
        axesRadius_,
        glm::vec4(0.0f, 0.0f, 1.0f, 1.0f)     // 파란색
    );
}

void PointCloudWidget::drawGrid() {
    glColor3f(0.3f, 0.3f, 0.3f);  // 회색 그리드
    glLineWidth(1.0f);
    
    glBegin(GL_LINES);
    
    // gridSize_: 그리드 전체 크기 (예: 10.0f = 10x10 영역)
    // gridSpacing_: 그리드 선 간격 (예: 1.0f = 1미터마다 선)
    
    float halfSize = gridSize_ / 2.0f;  // 중심에서 양쪽으로 그리드 크기
    
    // X축 방향 선들 (Y축을 따라 그어지는 선들)
    for (float x = -halfSize; x <= halfSize; x += gridSpacing_) {
        // ROS 좌표계 기준으로 그리고 OpenGL로 변환
        // ROS: X축 선 = Y축을 따라 그어지는 선
        glVertex3f(-halfSize, 0.0f, -x);  // 시작점 (ROS Y → OpenGL -X)
        glVertex3f(halfSize, 0.0f, -x);   // 끝점   (ROS Y → OpenGL -X)
    }
    
    // Y축 방향 선들 (X축을 따라 그어지는 선들)  
    for (float y = -halfSize; y <= halfSize; y += gridSpacing_) {
        // ROS: Y축 선 = X축을 따라 그어지는 선
        glVertex3f(-y, 0.0f, -halfSize);  // 시작점 (ROS X → OpenGL -Z)
        glVertex3f(-y, 0.0f, halfSize);   // 끝점   (ROS X → OpenGL -Z)
    }
    
    glEnd();
}

void PointCloudWidget::drawCameraIndicator() {
    glDisable(GL_DEPTH_TEST);  // 투명 객체는 깊이 테스트 비활성화
    
    // 투명한 노란색 실린더
    ShapeHelper::SimpleShape::drawCylinder(
        focusPoint_,                           // 위치
        glm::vec3(0, 1, 0),                   // 위쪽 방향
        0.1f,                                 // 높이
        0.3f,                                 // 반지름
        glm::vec4(1.0f, 1.0f, 0.0f, 0.7f)    // 투명한 노란색 (70% 불투명)
    );
    
    glEnable(GL_DEPTH_TEST);
}

void PointCloudWidget::paintEvent(QPaintEvent* event) {
    // 1단계: 먼저 OpenGL 렌더링 수행
    QOpenGLWidget::paintEvent(event);
    
    // 2단계: QPainter로 2D 오버레이 그리기
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    
    // 로봇 정보 라벨 그리기
    drawRobotLabel(painter);
}

void PointCloudWidget::drawRobotLabel(QPainter& painter) {
    // 로봇별 색상과 이름 결정 (먼저 실행)
    QString robotText;
    QColor robotColor;
    
    if (robotName_ == "TUGV") {
        robotText = "TUGV";
        robotColor = QColor(255, 0, 0);
    } else if (robotName_ == "MUGV") {
        robotText = "MUGV";
        robotColor = QColor(0, 255, 0);
    } else if (robotName_ == "SUGV1") {
        robotText = "SUGV1";
        robotColor = QColor(0, 0, 255);
    } else if (robotName_ == "SUGV2") {
        robotText = "SUGV2";
        robotColor = QColor(255, 255, 0);
    } else if (robotName_ == "SUAV") {
        robotText = "SUAV";
        robotColor = QColor(255, 0, 255);
    } else if (robotName_ == "COMBINED") {
        robotText = "ALL ROBOTS";
        robotColor = QColor(255, 255, 255);
    }
    
    // 폰트 설정 및 텍스트 크기 측정
    painter.setFont(QFont("Arial", fontSize_, QFont::Bold));
    QFontMetrics fm(painter.font());
    
    // 텍스트 크기 계산
    QRect textBounds = fm.boundingRect(robotText);
    int textWidth = textBounds.width();
    int textHeight = fm.height();
    

    
    int boxWidth = horizontalMargin_ + circleSize_ + circleMargin_ + textWidth + horizontalMargin_;
    int boxHeight = std::max(circleSize_ + verticalMargin_ * 2, textHeight + verticalMargin_ * 2);
    
    // 동적 크기의 배경 박스
    QRect labelRect(10, 10, boxWidth, boxHeight);
    painter.fillRect(labelRect, QColor(0, 0, 0, 150));
    
    // 테두리
    painter.setPen(QPen(Qt::white, 2));
    painter.drawRect(labelRect);
    
    // 박스 중앙 Y 좌표 계산
    int boxCenterY = labelRect.top() + labelRect.height() / 2;
    
    // 색상 인디케이터 원 (중앙 정렬)
    painter.setBrush(robotColor);
    painter.setPen(Qt::NoPen);
    int circleX = labelRect.left() + horizontalMargin_;
    painter.drawEllipse(circleX, boxCenterY - circleSize_/2, circleSize_, circleSize_);
    
    // 로봇 이름 텍스트 (중앙 정렬)
    painter.setPen(Qt::white);
    int textX = circleX + circleSize_ + circleMargin_;
    int textY = boxCenterY + fm.ascent()/2 - fm.descent()/2;  // 정확한 중앙 정렬
    
    painter.drawText(textX, textY, robotText);
}


} // namespace Widget