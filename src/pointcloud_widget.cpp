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
    updateCameraPosition();
    
    // 기본 색상들 초기화
    initializeDefaultColors();
}

PointCloudWidget::~PointCloudWidget() {
    makeCurrent();
    doneCurrent();
}

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

void PointCloudWidget::setRobot(const QString& robot) {
    robotName_ = robot;
    update();  // setDefaultColor 호출 제거
}

// 기존 setDefaultColor 함수 삭제하고 새로운 함수들로 대체

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
    return robotPointsColors_["DEFAULT"];  // 기본값 반환
}

glm::vec3 PointCloudWidget::getRobotPathColor(const QString& robot) const {
    if (robotPathColors_.contains(robot)) {
        return robotPathColors_[robot];
    }
    return robotPathColors_["DEFAULT"];  // 기본값 반환
}

void PointCloudWidget::resetAllColorsToDefault() {
    initializeDefaultColors();
    update();
}

void PointCloudWidget::onCloudShared(const QString& robot, CloudConstPtr cloud) {
    std::lock_guard<std::mutex> lock(cloudMutex_);
    clouds_[robot] = cloud;
    update();
}

void PointCloudWidget::onPathShared(const QString& robot, PathConstPtr path) {
    std::lock_guard<std::mutex> lock(pathMutex_);
    paths_[robot] = path;
    
    // ✅ 카메라 인디케이터가 이 로봇에 고정되어 있으면 위치 업데이트
    if (lockIndicatorToCurrentPosition_ && indicatorTargetRobot_ == robot) {
        // 메인 스레드에서 업데이트 실행 (스레드 안전)
        QMetaObject::invokeMethod(this, [this]() {
            updateIndicatorPosition();
            update();
        }, Qt::QueuedConnection);
    } else {
        update();
    }
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
    setLockIndicatorToCurrentPosition(true);  // 기본적으로 현재 위치에 고정
}

void PointCloudWidget::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
    float aspect = float(w) / h;
    projectionMatrix_ = glm::perspective(glm::radians(45.0f), aspect, 0.1f, 1000.0f);
}

void PointCloudWidget::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // 탑뷰 모드에서는 별도 처리
    if (isTopView_) {
        // 탑뷰에서는 updateTopViewCamera에서 계산된 viewMatrix_ 사용
    } else {
        // 일반 모드에서만 뷰 매트릭스 재계산
        glm::vec3 openglUpVector = glm::vec3(0, 1, 0);
        viewMatrix_ = glm::lookAt(cameraPos_, focusPoint_, openglUpVector);
    }

    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(glm::value_ptr(projectionMatrix_));
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(glm::value_ptr(viewMatrix_));

    // 기존 그리기 함수들
    drawPoints();
    drawPath();
    
    // 현재 위치 표시 추가 (경로 다음에 그려서 위에 보이도록)
    if (showPosition_) drawPositions();
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

void PointCloudWidget::setTopView(bool enable) {
    if (isTopView_ == enable) return;
    
    if (enable) {
        // 일반 뷰 → 탑뷰 전환
        backupCameraState();
        isTopView_ = true;
        
        // 탑뷰 카메라 설정
        pitch_ = 90.0f;  // 정확히 아래를 보도록
        yaw_ = 0.0f;      // 북쪽 방향
        distance_ = topViewHeight_;
        
        updateTopViewCamera();
    } else {
        // 탑뷰 → 일반 뷰 전환
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
            // 탑뷰 모드: yaw만 조절 가능, pitch는 90도로 고정
            float sensitivity = rotationSensitivity_;
            yaw_ -= delta.x() * sensitivity;
            pitch_ = 90.0f;  // 탑뷰에서는 피치 고정
            
            // 360도 순환
            if (yaw_ > 360.0f) yaw_ -= 360.0f;
            if (yaw_ < -360.0f) yaw_ += 360.0f;
            
            updateTopViewCamera();
        } else {
            // 일반 모드: 기존 회전 제어
            float sensitivity = rotationSensitivity_;
            yaw_ -= delta.x() * sensitivity;
            pitch_ += delta.y() * sensitivity;
            
            // 피치 제한 (완전히 뒤집히지 않도록)
            pitch_ = glm::clamp(pitch_, -89.0f, 89.0f);
            
            updateCameraPosition();
        }
        
        update();
    } else if (event->buttons() & Qt::MiddleButton && !lockIndicatorToCurrentPosition_) {
        // ✅ 탑뷰 모드 체크 추가!
        float deltaX = (event->x() - lastMousePos_.x()) * 0.02f;
        float deltaY = (event->y() - lastMousePos_.y()) * 0.02f;

        if (isTopView_) {
            // 탑뷰에서는 더 직관적인 팬 이동
            float panSensitivity = 0.03f * topViewHeight_;
            
            // 화면 이동을 월드 좌표 이동으로 변환 (yaw 고려)
            float cosYaw = cos(glm::radians(yaw_));
            float sinYaw = sin(glm::radians(yaw_));
            
            // 카메라 방향에 따른 이동 보정
            float worldDeltaX = -(deltaX * cosYaw - deltaY * sinYaw);
            float worldDeltaZ = -(deltaX * sinYaw + deltaY * cosYaw);
            
            focusPoint_.x += worldDeltaX * panSensitivity;
            focusPoint_.z += worldDeltaZ * panSensitivity;
            
            // ✅ 탑뷰 전용 카메라 업데이트 호출
            updateTopViewCamera();
        } else {
            // 일반 모드에서는 기존 팬 이동
            glm::vec3 cameraDir = glm::normalize(cameraPos_ - focusPoint_);
            glm::vec3 openglUp = glm::vec3(0, 1, 0);
            glm::vec3 openglRight = glm::normalize(glm::cross(openglUp, cameraDir));
            glm::vec3 openglActualUp = glm::cross(cameraDir, openglRight);
            
            focusPoint_ += -(openglRight * deltaX) + (openglActualUp * deltaY);
            
            // ✅ 일반 모드 카메라 업데이트 호출
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
        // 탑뷰 모드: 높이 조절 (줌)
        float zoomFactor = 1.0f + delta * 0.1f;
        topViewHeight_ *= zoomFactor;
        topViewHeight_ = glm::clamp(topViewHeight_, 2.0f, 100.0f);
        
        updateTopViewCamera();  // ✅ 탑뷰 전용 업데이트
    } else {
        // 일반 모드: 기존 거리 조절
        distance_ -= event->angleDelta().y() * 0.01f;
        distance_ = std::max(1.0f, distance_);
        
        updateCameraPosition();  // ✅ 일반 모드 업데이트
    }
    
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
            // 동적 색상 가져오기
            glm::vec3 color;
            
            if (robotName_ == "COMBINED") {
                // COMBINED 모드: 각 로봇의 설정된 색상 사용
                color = getRobotPointsColor(robotName);
            } else {
                // 단일 로봇 모드: 현재 로봇의 설정된 색상 사용
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
            // 동적 색상 가져오기
            glm::vec3 color;
            
            if (robotName_ == "COMBINED") {
                // COMBINED 모드: 각 로봇의 설정된 경로 색상 사용
                color = getRobotPathColor(robotName);
            } else {
                // 단일 로봇 모드: 현재 로봇의 설정된 경로 색상 사용
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
    // 원점에 ROS 표준 축 그리기
    glm::vec3 origin = glm::vec3(0.0f, 0.01f, 0.0f);
    glm::mat4 identityMatrix = glm::mat4(1.0f);
    
    // ✅ ROS 표준 축 그리기 (빨간색=앞, 초록색=왼쪽, 파란색=위)
    ShapeHelper::SimpleShape::drawRosAxes(
        origin,           // 원점 위치
        identityMatrix,   // 회전 없음
        axesLength_,      // 축 길이
        axesRadius_,      // 축 두께
        true              // 원뿔 화살표 포함
    );
}

void PointCloudWidget::drawGrid() {
    // planeCellCount_: 한 축의 셀 개수 (예: 20 = 20x20 그리드)
    // cellSize_: 각 셀의 크기 (예: 1.0f = 1미터)
    
    // 전체 그리드 크기 계산
    float totalSize = planeCellCount_ * cellSize_;
    float halfSize = totalSize / 2.0f;
    
    // 마이너 그리드 (각 셀 경계)
    glColor3f(0.3f, 0.3f, 0.3f);  // 어두운 회색
    glLineWidth(gridLineWidth_);
    glBegin(GL_LINES);
    
    // X축 방향 선들 (cellSize_ 간격)
    for (int i = 0; i <= planeCellCount_; ++i) {
        float pos = -halfSize + (i * cellSize_);
        
        // ROS 좌표계에서 OpenGL로 변환
        glVertex3f(-halfSize, 0.0f, -pos);  // 시작점
        glVertex3f(halfSize, 0.0f, -pos);   // 끝점
    }
    
    // Y축 방향 선들 (cellSize_ 간격)
    for (int i = 0; i <= planeCellCount_; ++i) {
        float pos = -halfSize + (i * cellSize_);
        
        glVertex3f(-pos, 0.0f, -halfSize);  // 시작점
        glVertex3f(-pos, 0.0f, halfSize);   // 끝점
    }
    glEnd();

    // 라인 두께 원복
    glLineWidth(1.0f);
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
    if (showRobotLabel_) drawRobotLabel(painter);
}

void PointCloudWidget::drawRobotLabel(QPainter& painter) {
    QString robotText;
    QColor robotColor;
    
    if (robotName_ == "TUGV") {
        robotText = "TUGV";
        glm::vec3 color = getRobotPointsColor("TUGV");
        robotColor = QColor(color.x * 255, color.y * 255, color.z * 255);
    } else if (robotName_ == "MUGV") {
        robotText = "MUGV";
        glm::vec3 color = getRobotPointsColor("MUGV");
        robotColor = QColor(color.x * 255, color.y * 255, color.z * 255);
    } else if (robotName_ == "SUGV1") {
        robotText = "SUGV1";
        glm::vec3 color = getRobotPointsColor("SUGV1");
        robotColor = QColor(color.x * 255, color.y * 255, color.z * 255);
    } else if (robotName_ == "SUGV2") {
        robotText = "SUGV2";
        glm::vec3 color = getRobotPointsColor("SUGV2");
        robotColor = QColor(color.x * 255, color.y * 255, color.z * 255);
    } else if (robotName_ == "SUAV") {
        robotText = "SUAV";
        glm::vec3 color = getRobotPointsColor("SUAV");
        robotColor = QColor(color.x * 255, color.y * 255, color.z * 255);
    } else if (robotName_ == "COMBINED") {
        robotText = "ALL ROBOTS";
        robotColor = QColor(255, 255, 255);
    }
    
    // 나머지 라벨 그리기 코드는 동일...
    painter.setFont(QFont("Arial", fontSize_, QFont::Bold));
    QFontMetrics fm(painter.font());
    
    QRect textBounds = fm.boundingRect(robotText);
    int textWidth = textBounds.width();
    int textHeight = fm.height();
    
    int boxWidth = horizontalMargin_ + circleSize_ + circleMargin_ + textWidth + horizontalMargin_;
    int boxHeight = std::max(circleSize_ + verticalMargin_ * 2, textHeight + verticalMargin_ * 2);
    
    QRect labelRect(10, 10, boxWidth, boxHeight);
    painter.fillRect(labelRect, QColor(0, 0, 0, 150));
    
    painter.setPen(QPen(Qt::white, 2));
    painter.drawRect(labelRect);
    
    int boxCenterY = labelRect.top() + labelRect.height() / 2;
    
    painter.setBrush(robotColor);
    painter.setPen(Qt::NoPen);
    int circleX = labelRect.left() + horizontalMargin_;
    painter.drawEllipse(circleX, boxCenterY - circleSize_/2, circleSize_, circleSize_);
    
    painter.setPen(Qt::white);
    int textX = circleX + circleSize_ + circleMargin_;
    int textY = boxCenterY + fm.ascent()/2 - fm.descent()/2;
    
    painter.drawText(textX, textY, robotText);
}

void PointCloudWidget::setShowPosition(bool show) {
    showPosition_ = show;
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

void PointCloudWidget::drawCylinderMarker(const glm::vec3& position, const glm::vec3& robotColor, const QString& robotName) {
    // 기본 실린더 마커 그리기
    ShapeHelper::SimpleShape::drawCylinder(
        position + glm::vec3(0, currentPositionHeight_/2, 0),  // 위치 (바닥에서 약간 위)
        glm::vec3(0, 1, 0),                                    // 위쪽 방향
        currentPositionHeight_,                                // 높이
        currentPositionRadius_,                                // 반지름
        glm::vec4(robotColor.x, robotColor.y, robotColor.z, 0.8f)  // 약간 투명
    );
    
    // 상단에 더 밝은 색상의 작은 원 추가 (더 눈에 띄게)
    ShapeHelper::SimpleShape::drawCylinder(
        position + glm::vec3(0, currentPositionHeight_ + 0.05f, 0),
        glm::vec3(0, 1, 0),
        0.05f,                                                 // 얇은 원판
        currentPositionRadius_ * 0.7f,                         // 더 작은 반지름
        glm::vec4(1.0f, 1.0f, 1.0f, 1.0f)                    // 흰색 (눈에 띄게)
    );
    
}

// pointcloud_widget.cpp에 ROS 축 사용
void PointCloudWidget::drawPositionAxes(const glm::vec3& position, const glm::quat& orientation, const QString& robotName) {
    float axesLength = positionAxesLength_;
    float axesRadius = positionAxesRadius_;
    
    if (robotName == "SUAV") {
        axesLength *= 1.2f;
    } else if (robotName == "SUGV1" || robotName == "SUGV2") {
        axesLength *= 0.8f;
    }
    
    glm::vec3 axesStart = position + glm::vec3(0, 0.05f, 0);
    
    // ✅ ROS 축 그리기 (로봇의 방향 표시)
    ShapeHelper::SimpleShape::drawRosAxes(axesStart, orientation, axesLength, axesRadius, true);
}
void PointCloudWidget::drawPositions() {
    if (!showPosition_) return;
    
    std::lock_guard<std::mutex> lock(pathMutex_);
    
    for (auto it = paths_.cbegin(); it != paths_.cend(); ++it) {
        const QString& robotName = it.key();
        
        // 현재 선택된 로봇만 표시하거나 COMBINED 모드에서 모든 로봇 표시
        if (robotName_ != "COMBINED" && robotName != robotName_) {
            continue;
        }
        
        const auto& path = it.value();
        
        // 경로가 있고 비어있지 않은 경우
        if (!path.empty()) {
            // 가장 최근 위치 (경로의 마지막 점)
            const auto& currentPose = path.back();
            
            // ROS 좌표를 OpenGL 좌표로 변환
            glm::vec3 position(
                -currentPose.pose.position.y,  // ROS Y → OpenGL -X
                currentPose.pose.position.z,   // ROS Z → OpenGL Y
                -currentPose.pose.position.x   // ROS X → OpenGL -Z
            );
            
            // 로봇의 방향 정보 추출 (쿼터니언)
            auto quat = currentPose.pose.orientation;
            glm::quat orientation(quat.w, quat.x, quat.y, quat.z);
            
            // 로봇별 색상 가져오기
            glm::vec3 robotColor;
            if (robotName_ == "COMBINED") {
                robotColor = getRobotPathColor(robotName);
            } else {
                robotColor = getRobotPathColor(robotName_);
            }
            
            // 마커 유형에 따라 다른 그리기 방식 적용
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

// pointcloud_widget.cpp에 함수들 추가

void PointCloudWidget::setLockIndicatorToCurrentPosition(bool lock) {
    lockIndicatorToCurrentPosition_ = lock;
    
    if (lock) {
        // 잠금 활성화 시 현재 선택된 로봇을 타겟으로 설정
        if (robotName_ != "COMBINED") {
            indicatorTargetRobot_ = robotName_;
        } else {
            // COMBINED 모드에서는 첫 번째 로봇을 타겟으로
            if (!paths_.isEmpty()) {
                indicatorTargetRobot_ = paths_.begin().key();
            }
        }
        
        // 즉시 위치 업데이트
        updateIndicatorPosition();
        showIndicator_ = true;
    } else {
        // 잠금 해제 시 타겟 초기화
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
    
    // 유효한 위치가 있으면 포커스 포인트를 해당 위치로 설정
    if (hasValidCurrentPosition(indicatorTargetRobot_)) {
        lastKnownPosition_ = currentPos;
        
        // 탑뷰 모드가 아닐 때만 포커스 포인트 변경
        if (!isTopView_) {
            focusPoint_ = currentPos;
            updateCameraPosition();
        } else {
            // 탑뷰에서는 카메라 위치만 변경
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
        
        // ✅ drawPositions()와 동일한 좌표 변환 로직
        return glm::vec3(
            -currentPose.pose.position.y,  // ROS Y → OpenGL -X
            currentPose.pose.position.z,   // ROS Z → OpenGL Y
            -currentPose.pose.position.x   // ROS X → OpenGL -Z
        );
    }
    
    return lastKnownPosition_;  // 데이터가 없으면 마지막 알려진 위치 반환
}

bool PointCloudWidget::hasValidCurrentPosition(const QString& robot) const {
    std::lock_guard<std::mutex> lock(pathMutex_);
    
    auto it = paths_.find(robot);
    return (it != paths_.end() && !it.value().empty());
}
} // namespace Widget