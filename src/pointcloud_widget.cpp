#include "pointcloud_widget.hpp"
#include "shape_helper.hpp"
#include <QDebug>  // ✅ qDebug 사용을 위해 추가
#include <QEvent>
#include <QPainter>
#include <QPaintEvent>
#include <QPen>
#include <QBrush>
#include <QFont>
#include <QColor>
#include <QRect>

namespace Widget {

// ============================================================================
// 🏗️ 생성자 및 소멸자
// ============================================================================

PointCloudWidget::PointCloudWidget(QWidget *parent) : QOpenGLWidget(parent) {
    lastMousePos_ = QPoint(0, 0);
    showIndicator_ = false;
    connect(&hideTimer_, &QTimer::timeout, this, &PointCloudWidget::hideIndicator);
    updateCameraPosition();
    initializeDefaultColors();
}

PointCloudWidget::~PointCloudWidget() {
    makeCurrent();
    doneCurrent();
}

// ============================================================================
// 🤖 로봇 관련 함수들
// ============================================================================

void PointCloudWidget::setRobot(const QString& robot) {
    robotName_ = robot;
    update();
}

// ============================================================================
// 🎨 색상 관리 함수들
// ============================================================================

void PointCloudWidget::initializeDefaultColors() {
    // 기본 포인트 색상들
    robotPointsColors_["TUGV"] = glm::vec3(1.0f, 0.0f, 0.0f);    // 빨간색
    robotPointsColors_["MUGV"] = glm::vec3(0.0f, 1.0f, 0.0f);    // 초록색
    robotPointsColors_["SUGV1"] = glm::vec3(0.0f, 0.0f, 1.0f);   // 파란색
    robotPointsColors_["SUGV2"] = glm::vec3(1.0f, 1.0f, 0.0f);   // 노란색
    robotPointsColors_["SUAV"] = glm::vec3(1.0f, 0.0f, 1.0f);    // 보라색
    robotPointsColors_["DEFAULT"] = glm::vec3(0.0f, 1.0f, 0.0f); // 기본 초록색
    
    // 기본 경로 색상들 (포인트보다 살짝 밝게)
    robotPathColors_["TUGV"] = glm::vec3(1.0f, 0.5f, 0.5f);     // 연한 빨간색
    robotPathColors_["MUGV"] = glm::vec3(0.5f, 1.0f, 0.5f);     // 연한 초록색
    robotPathColors_["SUGV1"] = glm::vec3(0.5f, 0.5f, 1.0f);    // 연한 파란색
    robotPathColors_["SUGV2"] = glm::vec3(1.0f, 1.0f, 0.5f);    // 연한 노란색
    robotPathColors_["SUAV"] = glm::vec3(1.0f, 0.5f, 1.0f);     // 연한 보라색
    robotPathColors_["DEFAULT"] = glm::vec3(0.5f, 1.0f, 0.5f);  // 기본 연한 초록색
}

void PointCloudWidget::setRobotPointsColor(const QString& robot, const glm::vec3& color) {
    robotPointsColors_[robot] = color;
    update();
}

void PointCloudWidget::setRobotPathColor(const QString& robot, const glm::vec3& color) {
    robotPathColors_[robot] = color;
    update();
}

glm::vec3 PointCloudWidget::getRobotPointsColor(const QString& robot) const {
    if (robotPointsColors_.contains(robot)) {
        return robotPointsColors_[robot];
    }
    return robotPointsColors_["DEFAULT"];
}

glm::vec3 PointCloudWidget::getRobotPathColor(const QString& robot) const {
    if (robotPathColors_.contains(robot)) {
        return robotPathColors_[robot];
    }
    return robotPathColors_["DEFAULT"];
}

void PointCloudWidget::resetAllColorsToDefault() {
    initializeDefaultColors();
    update();
}

// ============================================================================
// 📡 데이터 수신 슬롯들
// ============================================================================

void PointCloudWidget::onCloudShared(const QString& robot, CloudConstPtr cloud) {
    std::lock_guard<std::mutex> lock(cloudMutex_);
    clouds_[robot] = cloud;
    update();
}

void PointCloudWidget::onPathShared(const QString& robot, PathConstPtr path) {
    std::lock_guard<std::mutex> lock(pathMutex_);
    paths_[robot] = path;
    
    // 카메라 인디케이터가 이 로봇에 고정되어 있으면 위치 업데이트
    if (lockIndicatorToCurrentPosition_ && indicatorTargetRobot_ == robot) {
        QMetaObject::invokeMethod(this, [this]() {
            updateIndicatorPosition();
            update();
        }, Qt::QueuedConnection);
    } else {
        update();
    }
}

// ============================================================================
// 📷 카메라 제어 함수들
// ============================================================================

void PointCloudWidget::setFocusPoint(const glm::vec3& focus) {
    focusPoint_ = focus;
    updateCameraPosition();
    update();
}

void PointCloudWidget::setRotationSensitivity(float sensitivity) {
    rotationSensitivity_ = sensitivity;
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

void PointCloudWidget::setTopView(bool enable) {
    if (isTopView_ == enable) return;
    
    if (enable) {
        backupCameraState();
        isTopView_ = true;
        pitch_ = 90.0f;
        yaw_ = 0.0f;
        distance_ = topViewHeight_;
        updateTopViewCamera();
    } else {
        isTopView_ = false;
        restoreCameraState();
        updateCameraPosition();
    }
    
    update();
}

void PointCloudWidget::backupCameraState() {
    backupDistance_ = distance_;
    backupYaw_ = yaw_;
    backupPitch_ = pitch_;
    backupFocusPoint_ = focusPoint_;
}

void PointCloudWidget::restoreCameraState() {
    distance_ = backupDistance_;
    yaw_ = backupYaw_;
    pitch_ = backupPitch_;
    focusPoint_ = backupFocusPoint_;
}

void PointCloudWidget::updateTopViewCamera() {
    // 탑뷰용 카메라 위치 계산 (yaw 회전 적용)
    float yawRad = glm::radians(yaw_);
    
    // yaw에 따른 up 벡터 계산 (Z축 중심 회전)
    glm::vec3 up = glm::vec3(-sin(yawRad), 0.0f, -cos(yawRad));
    
    // 카메라는 항상 포커스 지점 바로 위에 위치
    cameraPos_ = focusPoint_ + glm::vec3(0.0f, topViewHeight_, 0.0f);
    
    // 뷰 매트릭스 계산 (yaw 회전이 적용된 up 벡터 사용)
    viewMatrix_ = glm::lookAt(cameraPos_, focusPoint_, up);
}

void PointCloudWidget::resetCamera() {
    if (isTopView_) {
        // 탑뷰 모드에서는 탑뷰 기본값으로 리셋
        focusPoint_ = glm::vec3(0.0f, 0.0f, 0.0f);
        topViewHeight_ = 20.0f;
        topViewZoom_ = 1.0f;
        updateTopViewCamera();
    } else {
        // 일반 모드에서는 일반 기본값으로 리셋
        focusPoint_ = glm::vec3(0.0f, 0.0f, 0.0f);
        distance_ = 10.0f;
        yaw_ = 0.0f;
        pitch_ = 0.0f;
        updateCameraPosition();
    }
    update();
}

void PointCloudWidget::jumpToPosition(const glm::vec3& position) {
    focusPoint_ = position;
    
    if (isTopView_) {
        updateTopViewCamera();
    } else {
        updateCameraPosition();
    }
    
    qDebug() << "📷 Camera jumped to position: (" 
             << position.x << "," << position.y << "," << position.z << ")";
    
    update();
}

void PointCloudWidget::jumpToRobotPosition(const QString& robotName) {
    if (!hasValidCurrentPosition(robotName)) {
        qDebug() << "❌ No valid position found for robot:" << robotName;
        return;
    }
    
    glm::vec3 robotPos = getCurrentRobotPosition(robotName);
    jumpToPosition(robotPos);
    
    qDebug() << "📷 Camera jumped to" << robotName 
             << "at position (" << robotPos.x << "," << robotPos.y << "," << robotPos.z << ")";
}

// ============================================================================
// 🎯 인디케이터 시스템
// ============================================================================

void PointCloudWidget::setLockIndicatorToCurrentPosition(bool lock) {
    lockIndicatorToCurrentPosition_ = lock;
    
    if (lock) {
        if (robotName_ != "COMBINED") {
            indicatorTargetRobot_ = robotName_;
        } else {
            if (!paths_.isEmpty()) {
                indicatorTargetRobot_ = paths_.begin().key();
            }
        }
        updateIndicatorPosition();
        showIndicator_ = true;
    } else {
        indicatorTargetRobot_ = "";
        showIndicator_ = false;
    }
    
    update();
}

void PointCloudWidget::setIndicatorTargetRobot(const QString& robot) {
    indicatorTargetRobot_ = robot;
    
    if (lockIndicatorToCurrentPosition_ && !robot.isEmpty()) {
        updateIndicatorPosition();
        showIndicator_ = true;
        update();
    }
}

void PointCloudWidget::updateIndicatorPosition() {
    if (!lockIndicatorToCurrentPosition_ || indicatorTargetRobot_.isEmpty()) {
        return;
    }
    
    glm::vec3 currentPos = getCurrentRobotPosition(indicatorTargetRobot_);
    
    if (hasValidCurrentPosition(indicatorTargetRobot_)) {
        lastKnownPosition_ = currentPos;
        
        if (!isTopView_) {
            focusPoint_ = currentPos;
            updateCameraPosition();
        } else {
            focusPoint_ = currentPos;
            updateTopViewCamera();
        }
    }
}

glm::vec3 PointCloudWidget::getCurrentRobotPosition(const QString& robot) const {
    std::lock_guard<std::mutex> lock(pathMutex_);
    
    auto it = paths_.find(robot);
    if (it != paths_.end() && !it.value().empty()) {
        const auto& currentPose = it.value().back();
        
        return glm::vec3(
            -currentPose.pose.position.y,
            currentPose.pose.position.z,
            -currentPose.pose.position.x
        );
    }
    
    return lastKnownPosition_;
}

bool PointCloudWidget::hasValidCurrentPosition(const QString& robot) const {
    std::lock_guard<std::mutex> lock(pathMutex_);
    
    auto it = paths_.find(robot);
    return (it != paths_.end() && !it.value().empty());
}

glm::vec3 PointCloudWidget::getRobotCurrentPosition(const QString& robotName) {
    return getCurrentRobotPosition(robotName);
}

void PointCloudWidget::hideIndicator() {
    showIndicator_ = false;
    update();
}

// ============================================================================
// 🎭 표시 옵션 함수들
// ============================================================================

void PointCloudWidget::setShowAxes(bool show) {
    showAxes_ = show;
    update();
}

void PointCloudWidget::setShowGrid(bool show) {
    showGrid_ = show;
    update();
}

void PointCloudWidget::setShowRobotLabel(bool show) {
    showRobotLabel_ = show;
    update();
}

void PointCloudWidget::setShowPosition(bool show) {
    showPosition_ = show;
    update();
}

// ✅ 새로 추가: 포인트 클라우드 표시 설정
void PointCloudWidget::setShowPoints(bool show) {
    showPoints_ = show;
    qDebug() << "🎯 Points display:" << (show ? "ON" : "OFF");
    update();
}

// ✅ 새로 추가: 경로 표시 설정
void PointCloudWidget::setShowPath(bool show) {
    showPath_ = show;
    qDebug() << "📍 Path display:" << (show ? "ON" : "OFF");
    update();
}

void PointCloudWidget::setPositionRadius(float radius) {
    if (radius >= 0.1f && radius <= 2.0f) {
        currentPositionRadius_ = radius;
        update();
    }
}

void PointCloudWidget::setPositionMarkerType(PositionMarkerType type) {
    positionMarkerType_ = type;
    update();
}

// ============================================================================
// 🖥️ Qt 이벤트 오버라이드
// ============================================================================

void PointCloudWidget::initializeGL() {
    initializeOpenGLFunctions();
    glEnable(GL_DEPTH_TEST);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glClearColor(0.15f, 0.15f, 0.15f, 1.0f);
    setLockIndicatorToCurrentPosition(false);
}

void PointCloudWidget::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
    float aspect = float(w) / h;
    projectionMatrix_ = glm::perspective(glm::radians(45.0f), aspect, 0.1f, 1000.0f);
}

void PointCloudWidget::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (isTopView_) {
        // 탑뷰에서는 updateTopViewCamera에서 계산된 viewMatrix_ 사용
    } else {
        glm::vec3 openglUpVector = glm::vec3(0, 1, 0);
        viewMatrix_ = glm::lookAt(cameraPos_, focusPoint_, openglUpVector);
    }

    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(glm::value_ptr(projectionMatrix_));
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(glm::value_ptr(viewMatrix_));

    // ✅ 조건부 렌더링: showPoints_와 showPath_ 확인
    if (showPoints_) drawPoints();      // 포인트 클라우드 표시
    if (showPath_) drawPath();          // 경로 표시
    
    if (showPosition_) drawPositions();
    if (showAxes_) drawAxes();
    if (showGrid_) drawGrid();
    if (showIndicator_) drawCameraIndicator();
}

void PointCloudWidget::paintEvent(QPaintEvent* event) {
    QOpenGLWidget::paintEvent(event);
    
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    
    if (showRobotLabel_) drawRobotLabel(painter);
}

void PointCloudWidget::mousePressEvent(QMouseEvent *event) {
    lastMousePos_ = event->pos();
    showIndicator_ = true;
    hideTimer_.stop();
    update();
}

void PointCloudWidget::mouseMoveEvent(QMouseEvent *event) {
    if (event->buttons() & Qt::LeftButton) {
        QPoint delta = event->pos() - lastMousePos_;
        
        if (isTopView_) {
            float sensitivity = rotationSensitivity_;
            yaw_ -= delta.x() * sensitivity;
            pitch_ = 90.0f;
            
            if (yaw_ > 360.0f) yaw_ -= 360.0f;
            if (yaw_ < -360.0f) yaw_ += 360.0f;
            
            updateTopViewCamera();
        } else {
            float sensitivity = rotationSensitivity_;
            yaw_ -= delta.x() * sensitivity;
            pitch_ += delta.y() * sensitivity;
            pitch_ = glm::clamp(pitch_, -89.0f, 89.0f);
            updateCameraPosition();
        }
        
        update();
    } else if (event->buttons() & Qt::MiddleButton && !lockIndicatorToCurrentPosition_) {
        float deltaX = (event->x() - lastMousePos_.x()) * 0.02f;
        float deltaY = (event->y() - lastMousePos_.y()) * 0.02f;

        if (isTopView_) {
            float panSensitivity = 0.03f * topViewHeight_;
            float cosYaw = cos(glm::radians(yaw_));
            float sinYaw = sin(glm::radians(yaw_));
            
            float worldDeltaX = -(deltaX * cosYaw - deltaY * sinYaw);
            float worldDeltaZ = -(deltaX * sinYaw + deltaY * cosYaw);
            
            focusPoint_.x += worldDeltaX * panSensitivity;
            focusPoint_.z += worldDeltaZ * panSensitivity;
            
            updateTopViewCamera();
        } else {
            glm::vec3 cameraDir = glm::normalize(cameraPos_ - focusPoint_);
            glm::vec3 openglUp = glm::vec3(0, 1, 0);
            glm::vec3 openglRight = glm::normalize(glm::cross(openglUp, cameraDir));
            glm::vec3 openglActualUp = glm::cross(cameraDir, openglRight);
            
            focusPoint_ += -(openglRight * deltaX) + (openglActualUp * deltaY);
            updateCameraPosition();
        }

        showIndicator_ = true;
        update();
    }
    
    lastMousePos_ = event->pos();
}

void PointCloudWidget::mouseReleaseEvent(QMouseEvent *event) {
    Q_UNUSED(event);
    hideTimer_.start(timerInterval_);
}

void PointCloudWidget::wheelEvent(QWheelEvent *event) {
    float delta = event->angleDelta().y() / 120.0f;
    
    if (isTopView_) {
        float zoomFactor = 1.0f + delta * 0.1f;
        topViewHeight_ *= zoomFactor;
        topViewHeight_ = glm::clamp(topViewHeight_, 2.0f, 100.0f);
        updateTopViewCamera();
    } else {
        distance_ -= event->angleDelta().y() * 0.01f;
        distance_ = std::max(1.0f, distance_);
        updateCameraPosition();
    }
    
    update();
}

// ============================================================================
// 🎨 렌더링 시스템
// ============================================================================

void PointCloudWidget::setPointSize(float size) {
    pointSize_ = glm::clamp(size, 0.5f, 10.0f);
    qDebug() << "🔍 Point size set to:" << pointSize_;
    update();
}

void PointCloudWidget::setPathWidth(float width) {
    pathWidth_ = glm::clamp(width, 0.5f, 10.0f);
    qDebug() << "📏 Path width set to:" << pathWidth_;
    update();
}

void PointCloudWidget::drawPoints() {
    // ✅ 추가 체크: showPoints_가 false면 렌더링하지 않음
    if (!showPoints_) return;
    
    std::lock_guard<std::mutex> lock(cloudMutex_);

    glPointSize(pointSize_);  // ✅ 동적 포인트 크기
    glBegin(GL_POINTS);

    for (auto it = clouds_.cbegin(); it != clouds_.cend(); ++it) {
        const QString& robotName = it.key();
        
        if (robotName_ != "COMBINED" && robotName != robotName_) {
            continue;
        }
        
        const auto& cloud = it.value();

        if (cloud && !cloud->empty()) {
            glm::vec3 color;
            
            if (robotName_ == "COMBINED") {
                color = getRobotPointsColor(robotName);
            } else {
                color = getRobotPointsColor(robotName_);
            }
            
            glColor3f(color.x, color.y, color.z);
            
            for (const auto& point : cloud->points) {
                glVertex3f(-point.y, point.z, -point.x);
            }
        }
    }
    glEnd();
}

void PointCloudWidget::drawPath() {
    // ✅ 추가 체크: showPath_가 false면 렌더링하지 않음
    if (!showPath_) return;
    
    std::lock_guard<std::mutex> lock(pathMutex_);
    
    glLineWidth(pathWidth_);  // ✅ 동적 경로 두께
    glBegin(GL_LINES);
    
    for (auto it = paths_.cbegin(); it != paths_.cend(); ++it) {
        const QString& robotName = it.key();
        
        if (robotName_ != "COMBINED" && robotName != robotName_) {
            continue;
        }
        
        const auto& path = it.value();
        
        if (!path.empty()) {
            glm::vec3 color;
            
            if (robotName_ == "COMBINED") {
                color = getRobotPathColor(robotName);
            } else {
                color = getRobotPathColor(robotName_);
            }
            
            glColor3f(color.x, color.y, color.z);
            
            for (size_t i = 1; i < path.size(); ++i) {
                const auto& prev_pose = path[i-1];
                const auto& curr_pose = path[i];
                
                glVertex3f(-prev_pose.pose.position.y, prev_pose.pose.position.z, -prev_pose.pose.position.x);
                glVertex3f(-curr_pose.pose.position.y, curr_pose.pose.position.z, -curr_pose.pose.position.x);
            }
        }
    }
    
    glEnd();
}

void PointCloudWidget::drawAxes() {
    glm::vec3 origin = glm::vec3(0.0f, 0.01f, 0.0f);
    glm::mat4 identityMatrix = glm::mat4(1.0f);
    
    ShapeHelper::SimpleShape::drawRosAxes(
        origin,
        identityMatrix,
        axesLength_,
        axesRadius_,
        true
    );
}

void PointCloudWidget::drawGrid() {
    float totalSize = planeCellCount_ * cellSize_;
    float halfSize = totalSize / 2.0f;
    
    glColor3f(0.3f, 0.3f, 0.3f);
    glLineWidth(gridLineWidth_);
    glBegin(GL_LINES);
    
    for (int i = 0; i <= planeCellCount_; ++i) {
        float pos = -halfSize + (i * cellSize_);
        
        glVertex3f(-halfSize, 0.0f, -pos);
        glVertex3f(halfSize, 0.0f, -pos);
    }
    
    for (int i = 0; i <= planeCellCount_; ++i) {
        float pos = -halfSize + (i * cellSize_);
        
        glVertex3f(-pos, 0.0f, -halfSize);
        glVertex3f(-pos, 0.0f, halfSize);
    }
    glEnd();

    glLineWidth(1.0f);
}

void PointCloudWidget::drawCameraIndicator() {
    glDisable(GL_DEPTH_TEST);
    
    ShapeHelper::SimpleShape::drawCylinder(
        focusPoint_,
        glm::vec3(0, 1, 0),
        0.1f,
        0.3f,
        glm::vec4(1.0f, 1.0f, 0.0f, 0.7f)
    );
    
    glEnable(GL_DEPTH_TEST);
}

void PointCloudWidget::drawPositions() {
    if (!showPosition_) return;
    
    std::lock_guard<std::mutex> lock(pathMutex_);
    
    for (auto it = paths_.cbegin(); it != paths_.cend(); ++it) {
        const QString& robotName = it.key();
        
        if (robotName_ != "COMBINED" && robotName != robotName_) {
            continue;
        }
        
        const auto& path = it.value();
        
        if (!path.empty()) {
            const auto& currentPose = path.back();
            
            glm::vec3 position(
                -currentPose.pose.position.y,
                currentPose.pose.position.z,
                -currentPose.pose.position.x
            );
            
            auto quat = currentPose.pose.orientation;
            glm::quat orientation(quat.w, quat.x, quat.y, quat.z);
            
            glm::vec3 robotColor;
            if (robotName_ == "COMBINED") {
                robotColor = getRobotPathColor(robotName);
            } else {
                robotColor = getRobotPathColor(robotName_);
            }
            
            switch (positionMarkerType_) {
                case PositionMarkerType::CYLINDER:
                    drawCylinderMarker(position, robotColor, robotName);
                    break;
                    
                case PositionMarkerType::AXES:
                    drawPositionAxes(position, orientation, robotName);
                    break;
            }
        }
    }
}

void PointCloudWidget::drawCylinderMarker(const glm::vec3& position, const glm::vec3& robotColor, const QString& robotName) {
    ShapeHelper::SimpleShape::drawCylinder(
        position + glm::vec3(0, currentPositionHeight_/2, 0),
        glm::vec3(0, 1, 0),
        currentPositionHeight_,
        currentPositionRadius_,
        glm::vec4(robotColor.x, robotColor.y, robotColor.z, 0.8f)
    );
    
    ShapeHelper::SimpleShape::drawCylinder(
        position + glm::vec3(0, currentPositionHeight_ + 0.05f, 0),
        glm::vec3(0, 1, 0),
        0.05f,
        currentPositionRadius_ * 0.7f,
        glm::vec4(1.0f, 1.0f, 1.0f, 1.0f)
    );
}

void PointCloudWidget::drawPositionAxes(const glm::vec3& position, const glm::quat& orientation, const QString& robotName) {
    float axesLength = positionAxesLength_;
    float axesRadius = positionAxesRadius_;
    
    if (robotName == "SUAV") {
        axesLength *= 1.2f;
    } else if (robotName == "SUGV1" || robotName == "SUGV2") {
        axesLength *= 0.8f;
    }
    
    glm::vec3 axesStart = position + glm::vec3(0, 0.05f, 0);
    
    ShapeHelper::SimpleShape::drawRosAxes(axesStart, orientation, axesLength, axesRadius, true);
}

void PointCloudWidget::drawCustomAxes(const glm::vec3& position, const glm::quat& orientation) {
    // 사용자 정의 축 그리기 (필요시 구현)
}

// ============================================================================
// 🏷️ 로봇 라벨 렌더링
// ============================================================================

void PointCloudWidget::drawRobotLabel(QPainter& painter) {
    if (robotName_ == "TUGV") {
        QString robotText = "TUGV";
        glm::vec3 color = getRobotPointsColor("TUGV");
        QColor robotColor(color.x * 255, color.y * 255, color.z * 255);
        drawSingleLabel(painter, robotText, robotColor, QPoint(10, 10));
        
    } else if (robotName_ == "MUGV") {
        QString robotText = "MUGV";
        glm::vec3 color = getRobotPointsColor("MUGV");
        QColor robotColor(color.x * 255, color.y * 255, color.z * 255);
        drawSingleLabel(painter, robotText, robotColor, QPoint(10, 10));
        
    } else if (robotName_ == "SUGV1") {
        QString robotText = "SUGV1";
        glm::vec3 color = getRobotPointsColor("SUGV1");
        QColor robotColor(color.x * 255, color.y * 255, color.z * 255);
        drawSingleLabel(painter, robotText, robotColor, QPoint(10, 10));
        
    } else if (robotName_ == "SUGV2") {
        QString robotText = "SUGV2";
        glm::vec3 color = getRobotPointsColor("SUGV2");
        QColor robotColor(color.x * 255, color.y * 255, color.z * 255);
        drawSingleLabel(painter, robotText, robotColor, QPoint(10, 10));
        
    } else if (robotName_ == "SUAV") {
        QString robotText = "SUAV";
        glm::vec3 color = getRobotPointsColor("SUAV");
        QColor robotColor(color.x * 255, color.y * 255, color.z * 255);
        drawSingleLabel(painter, robotText, robotColor, QPoint(10, 10));
        
    } else if (robotName_ == "COMBINED") {
        QStringList robots = {"TUGV", "MUGV", "SUGV1", "SUGV2", "SUAV"};
        
        for (int i = 0; i < robots.size(); ++i) {
            QString robotName = robots[i];
            glm::vec3 color = getRobotPointsColor(robotName);
            QColor robotColor(color.x * 255, color.y * 255, color.z * 255);
            
            int yOffset = 10 + i * 40;
            drawSingleLabel(painter, robotName, robotColor, QPoint(10, yOffset));
        }
    }
}

void PointCloudWidget::drawSingleLabel(QPainter& painter, const QString& text, const QColor& robotColor, const QPoint& position) {
    painter.setFont(QFont("Arial", fontSize_, QFont::Bold));
    QFontMetrics fm(painter.font());
    
    QRect textBounds = fm.boundingRect(text);
    int textWidth = textBounds.width();
    int textHeight = fm.height();
    
    int boxWidth = horizontalMargin_ + circleSize_ + circleMargin_ + textWidth + horizontalMargin_;
    int boxHeight = std::max(circleSize_ + verticalMargin_ * 2, textHeight + verticalMargin_ * 2);
    
    QRect labelRect(position.x(), position.y(), boxWidth, boxHeight);
    
    painter.setBrush(QBrush(QColor(0, 0, 0, 100)));
    painter.setPen(Qt::NoPen);
    painter.fillRect(labelRect, QColor(0, 0, 0, 100));
    
    painter.setBrush(Qt::NoBrush);
    painter.setPen(QPen(Qt::white, 1));
    painter.drawRect(labelRect);
    
    int boxCenterY = labelRect.top() + labelRect.height() / 2;
    
    painter.setBrush(QBrush(robotColor));
    painter.setPen(Qt::NoPen);
    int circleX = labelRect.left() + horizontalMargin_;
    painter.drawEllipse(circleX, boxCenterY - circleSize_/2, circleSize_, circleSize_);
    
    painter.setBrush(Qt::NoBrush);
    painter.setPen(Qt::white);
    int textX = circleX + circleSize_ + circleMargin_;
    int textY = boxCenterY + fm.ascent()/2 - fm.descent()/2;
    painter.drawText(textX, textY, text);
}

} // namespace Widget