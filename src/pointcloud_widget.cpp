#include "pointcloud_widget.hpp"
#include "shape_helper.hpp"
#include "grid_map_processor.hpp"
#include "render_helper.hpp"
#include <QDebug>
#include <QEvent>
#include <QPainter>
#include <QPaintEvent>
#include <QKeyEvent>
#include <QMouseEvent>  // 추가
#include <QWheelEvent>  // 추가
#include <QPen>
#include <QBrush>
#include <QFont>
#include <QColor>
#include <QRect>
#include <QTimer>       // 추가
#include <algorithm>    // 추가
#include <mutex>        // 추가

namespace Widget {

// ============================================================================
// Constructor and Destructor
// ============================================================================

PointCloudWidget::PointCloudWidget(QWidget *parent) : QOpenGLWidget(parent) {
    lastMousePos_ = QPoint(0, 0);
    showIndicator_ = false;
    
    // Enable keyboard focus for this widget
    setFocusPolicy(Qt::StrongFocus);
    
    // 기본값 설정 (RenderHelper 타입 사용)
    showPoints_ = true;
    showPath_ = true;
    showPosition_ = true;
    showAxes_ = true;
    showGrid_ = true;
    showRobotLabel_ = true;
    showGridMap_ = true;
    pointSize_ = 2.0f;
    pathWidth_ = 3.0f;
    positionMarkerType_ = RenderHelper::PositionMarkerType::AXES;  // 명시적으로 RenderHelper 타입 사용
    
    connect(&hideTimer_, &QTimer::timeout, this, &PointCloudWidget::hideIndicator);
    updateCameraPosition();
    initializeDefaultColors();
    
    qDebug() << "PointCloudWidget created with RenderHelper integration";
}

PointCloudWidget::~PointCloudWidget() {
    makeCurrent();
    doneCurrent();
}

// ============================================================================
// Robot Related Functions
// ============================================================================

void PointCloudWidget::setRobot(const QString& robot) {
    robotName_ = robot;
    update();
}

// ============================================================================
// Color Management Functions
// ============================================================================

void PointCloudWidget::initializeDefaultColors() {
    // Default point colors
    robotPointsColors_["TUGV"] = glm::vec3(1.0f, 0.0f, 0.0f);    // Red
    robotPointsColors_["MUGV"] = glm::vec3(0.0f, 1.0f, 0.0f);    // Green
    robotPointsColors_["SUGV1"] = glm::vec3(0.0f, 0.0f, 1.0f);   // Blue
    robotPointsColors_["SUGV2"] = glm::vec3(1.0f, 1.0f, 0.0f);   // Yellow
    robotPointsColors_["SUAV"] = glm::vec3(1.0f, 0.0f, 1.0f);    // Magenta
    robotPointsColors_["DEFAULT"] = glm::vec3(0.0f, 1.0f, 0.0f); // Default Green
    
    // Default path colors (slightly brighter than points)
    robotPathColors_["TUGV"] = glm::vec3(1.0f, 0.5f, 0.5f);     // Light Red
    robotPathColors_["MUGV"] = glm::vec3(0.5f, 1.0f, 0.5f);     // Light Green
    robotPathColors_["SUGV1"] = glm::vec3(0.5f, 0.5f, 1.0f);    // Light Blue
    robotPathColors_["SUGV2"] = glm::vec3(1.0f, 1.0f, 0.5f);    // Light Yellow
    robotPathColors_["SUAV"] = glm::vec3(1.0f, 0.5f, 1.0f);     // Light Magenta
    robotPathColors_["DEFAULT"] = glm::vec3(0.5f, 1.0f, 0.5f);  // Default Light Green
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
// Data Reception Slots
// ============================================================================

void PointCloudWidget::onCloudShared(const QString& robot, CloudConstPtr cloud) {
    {
        std::lock_guard<std::mutex> lock(cloudMutex_);
        clouds_[robot] = cloud;
    }
    
    // 그리드 맵 업데이트 (백그라운드에서 처리)
    if (showGridMap_) {
        updateGridMapForRobot(robot, cloud);
    }
    
    update();
}

void PointCloudWidget::onPathShared(const QString& robot, PathConstPtr path) {
    std::lock_guard<std::mutex> lock(pathMutex_);
    paths_[robot] = path;
    
    // Update position if camera indicator is locked to this robot
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
// Camera Control Functions
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
    // Calculate camera position based on ROS coordinate system
    // ROS: X=forward, Y=left, Z=up
    float rosX = distance_ * cos(glm::radians(pitch_)) * cos(glm::radians(yaw_));  // forward
    float rosY = distance_ * cos(glm::radians(pitch_)) * sin(glm::radians(yaw_));  // left
    float rosZ = distance_ * sin(glm::radians(pitch_));                            // up
    
    // ROS → OpenGL coordinate conversion
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
    // Calculate top view camera position (with yaw rotation applied)
    float yawRad = glm::radians(yaw_);
    
    // Calculate up vector based on yaw (rotation around Z-axis)
    glm::vec3 up = glm::vec3(-sin(yawRad), 0.0f, -cos(yawRad));
    
    // Camera is always positioned directly above the focus point
    cameraPos_ = focusPoint_ + glm::vec3(0.0f, topViewHeight_, 0.0f);
    
    // Calculate view matrix (using up vector with yaw rotation applied)
    viewMatrix_ = glm::lookAt(cameraPos_, focusPoint_, up);
}

void PointCloudWidget::resetCamera() {
    if (isTopView_) {
        // Reset to top view defaults in top view mode
        focusPoint_ = glm::vec3(0.0f, 0.0f, 0.0f);
        topViewHeight_ = 20.0f;
        topViewZoom_ = 1.0f;
        updateTopViewCamera();
    } else {
        // Reset to normal defaults in normal mode
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
    
    qDebug() << "Camera jumped to position: (" 
             << position.x << "," << position.y << "," << position.z << ")";
    
    update();
}

void PointCloudWidget::jumpToRobotPosition(const QString& robotName) {
    if (!hasValidCurrentPosition(robotName)) {
        qDebug() << "No valid position found for robot:" << robotName;
        return;
    }
    
    glm::vec3 robotPos = getCurrentRobotPosition(robotName);
    jumpToPosition(robotPos);
    
    qDebug() << "Camera jumped to" << robotName 
             << "at position (" << robotPos.x << "," << robotPos.y << "," << robotPos.z << ")";
}

// ============================================================================
// Indicator System
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
// Display Option Functions
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

// Newly added: Point cloud display setting
void PointCloudWidget::setShowPoints(bool show) {
    showPoints_ = show;
    qDebug() << "Points display:" << (show ? "ON" : "OFF");
    update();
}

// Newly added: Path display setting
void PointCloudWidget::setShowPath(bool show) {
    showPath_ = show;
    qDebug() << "Path display:" << (show ? "ON" : "OFF");
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

void PointCloudWidget::setGridCellCount(int count) {
    planeCellCount_ = glm::clamp(count, 4, 50);  // 4x4 ~ 50x50 범위로 제한
    update();
    qDebug() << "Grid cell count set to:" << planeCellCount_ << "x" << planeCellCount_;
}

void PointCloudWidget::setGridSize(float size) {
    cellSize_ = glm::clamp(size, 0.1f, 10.0f);  // 0.1m ~ 10.0m 범위로 제한
    update();
    qDebug() << "Grid cell size set to:" << cellSize_ << "meters";
}

void PointCloudWidget::setAxesSize(float size) {
    axesLength_ = glm::clamp(size, 0.1f, 5.0f);  // 0.1m ~ 5.0m 범위로 제한
    axesRadius_ = axesLength_ * 0.05f;           // 축 길이에 비례하여 반지름 자동 조정
    update();
    qDebug() << "Axes size set to:" << axesLength_ << "meters (radius:" << axesRadius_ << ")";
}

// ============================================================================
// Point and Path Style Settings
// ============================================================================
void PointCloudWidget::setPointSize(float size) {
    pointSize_ = size;
    update();
    qDebug() << "Point size set to:" << size << "for robot:" << robotName_;
}

void PointCloudWidget::setPathWidth(float width) {
    pathWidth_ = width;
    update();
    qDebug() << "Path width set to:" << width << "for robot:" << robotName_;
}

// ============================================================================
// Map Style Control Functions
// ============================================================================
void PointCloudWidget::setMapStyle(const QString& style) {
    if (style.toLower() == "pointcloud") {
        showPoints_ = true;
        showGridMap_ = false;
        
        qDebug() << "Map style set to: PointCloud for robot:" << robotName_;
    } else if (style.toLower() == "gridmap") {
        showPoints_ = false;
        showGridMap_ = true;
        qDebug() << "Map style set to: Grid Map for robot:" << robotName_;
    } else {
        qWarning() << "Invalid map style:" << style << "- using PointCloud as default";
        showPoints_ = true;
        showGridMap_ = false;
    }
    mapStyle_ = style.toLower();
    update();
}

// Grid Map 함수들이 누락되어 있다면 추가
void PointCloudWidget::setShowGridMap(bool show) {
    if (showGridMap_ != show) {
        showGridMap_ = show;
        update();  // 화면 갱신
        qDebug() << "GridMap display set to:" << (show ? "ON" : "OFF");
    }
}

void PointCloudWidget::setGridMapParameters(const GridMap::GridMapParameters& params) {
    gridParams_ = params;
    
    // 기존 그리드맵 데이터 클리어 (새 파라미터 적용)
    {
        std::lock_guard<std::mutex> lock(gridMapMutex_);
        gridMaps_.clear();
    }
    
    qDebug() << "Grid map parameters updated for robot:" << robotName_;
    update();
}

void PointCloudWidget::updateGridMapForRobot(const QString& robotName, CloudConstPtr cloud) {
    if (!cloud || cloud->empty()) {
        qDebug() << "PointCloudWidget::updateGridMapForRobot: Empty cloud for robot:" << robotName;
        return;
    }
    
    // 그리드맵 프로세싱
    std::lock_guard<std::mutex> lock(gridMapMutex_);
    
    auto gridData = std::make_shared<GridMap::GridMapData>();
    
    // GridMapProcessor::processPointCloud 사용
    if (GridMap::GridMapProcessor::processPointCloud(cloud, gridParams_, *gridData)) {
        gridMaps_[robotName] = gridData;
        qDebug() << "PointCloudWidget: Grid map updated for robot:" << robotName 
                 << "size:" << gridData->width << "x" << gridData->height
                 << "points:" << cloud->size();
    } else {
        qDebug() << "PointCloudWidget: Failed to process grid map for robot:" << robotName;
    }
}

void PointCloudWidget::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // 카메라 설정
    if (isTopView_) {
        updateTopViewCamera();
    } else {
        updateCameraPosition();
        viewMatrix_ = glm::lookAt(cameraPos_, focusPoint_, glm::vec3(0, 1, 0));
    }

    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(glm::value_ptr(projectionMatrix_));
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(glm::value_ptr(viewMatrix_));

    // Map Style에 따른 렌더링 분기
    QString currentMapStyle = getMapStyle();
    
    if (currentMapStyle == "gridmap" && showGridMap_) {  // showGridMap_ 체크 추가
        // GridMap 모드: 기본 OpenGL로 그리드맵 + 2D 평면화된 경로
        RenderHelper::GridMapRenderer::drawBasicGridMaps(
            gridMaps_, robotName_, robotPointsColors_, gridMapMutex_
        );
    } else if (currentMapStyle == "pointcloud") {
        // PointCloud 모드: 기존 3D 렌더링
        if (showPoints_) {
            RenderHelper::PointCloudRenderer::drawPoints(
                clouds_, robotName_, robotPointsColors_, pointSize_, cloudMutex_
            );
        }
    }
    
    // Path는 showPath_ 플래그에 따라 모든 모드에서 렌더링 가능
    if (showPath_) {
        if (currentMapStyle == "gridmap") {
            RenderHelper::GridMapRenderer::draw2DProjectedPaths(
                paths_, robotName_, robotPathColors_, pathMutex_, pathWidth_
            );
        } else {
            // PointCloud 모드에서는 3D Path 렌더링
            RenderHelper::PointCloudRenderer::drawPaths(
                paths_, robotName_, robotPathColors_, pathWidth_, pathMutex_
            );
        }
    }
    
    // 공통 요소들 (Map Style과 무관하게 항상 렌더링)
    if (showPosition_) {
        RenderHelper::PointCloudRenderer::drawPositions(
            paths_, robotName_, robotPathColors_, positionMarkerType_,
            currentPositionRadius_,        // Cylinder과 Axes 모두에서 사용될 기본 크기
            currentPositionHeight_,        // Cylinder 높이 (Axes에서는 무시)
            currentPositionRadius_ * 3.0f, // Axes 길이 = radius * 3 (적절한 비율)
            currentPositionRadius_ * 0.2f, // Axes 두께 = radius * 0.2 (얇게)
            pathMutex_
        );
    }
    
    if (showAxes_) {
        RenderHelper::PointCloudRenderer::drawAxes(
            glm::vec3(0.0f, 0.01f, 0.0f), axesLength_, axesRadius_
        );
    }
    
    if (showGrid_) {
        RenderHelper::PointCloudRenderer::drawGrid(
            planeCellCount_, cellSize_, gridLineWidth_
        );
    }
    
    if (showIndicator_) {
        RenderHelper::PointCloudRenderer::drawCameraIndicator(focusPoint_);
    }
}

void PointCloudWidget::paintEvent(QPaintEvent* event) {
    QOpenGLWidget::paintEvent(event);
    
    if (showRobotLabel_) {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);
        RenderHelper::PointCloudRenderer::drawRobotLabels(
            painter, robotName_, robotPointsColors_
        );
    }
}

void PointCloudWidget::keyPressEvent(QKeyEvent *event) {
    switch (event->key()) {
        case Qt::Key_Z:
            // Reset camera to default position
            resetCamera();
            qDebug() << "Camera reset with Z key for robot:" << robotName_;
            event->accept();
            break;
            
        default:
            QOpenGLWidget::keyPressEvent(event);
            break;
    }
}

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

// ============================================================================
// Mouse Event Handlers
// ============================================================================
void PointCloudWidget::mousePressEvent(QMouseEvent *event) {
    // Set focus to this widget so it can receive keyboard events
    setFocus();
    
    lastMousePos_ = event->pos();
    showIndicator_ = true;
    hideTimer_.stop();
    update();
}

void PointCloudWidget::mouseReleaseEvent(QMouseEvent *event) {
    Q_UNUSED(event);
    hideTimer_.start(timerInterval_);
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

// GridMap Resolution 제어 함수 구현

void PointCloudWidget::setGridMapResolution(float resolution) {
    if (gridMapResolution_ != resolution) {
        gridMapResolution_ = std::clamp(resolution, 0.01f, 1.0f);  // 범위 제한
        
        // 모든 GridMap에 새로운 해상도 적용
        std::lock_guard<std::mutex> lock(gridMapMutex_);
        for (auto it = gridMaps_.begin(); it != gridMaps_.end(); ++it) {
            if (it.value() && it.value()->isValid()) {
                // GridMap 데이터를 새로운 해상도로 리샘플링
                resampleGridMapResolution(it.value(), gridMapResolution_);
            }
        }
        
        update();  // 화면 갱신
        qDebug() << "GridMap resolution set to:" << gridMapResolution_ << "meters";
    }
}

float PointCloudWidget::getGridMapResolution() const {
    return gridMapResolution_;
}

// GridMap 해상도 리샘플링 함수 (새로 추가)
void PointCloudWidget::resampleGridMapResolution(
    std::shared_ptr<GridMap::GridMapData> gridData, 
    float newResolution) {
    
    if (!gridData || !gridData->isValid()) return;
    
    qDebug() << "Resampling GridMap from" << gridData->resolution 
             << "to" << newResolution << "meters";
    
    // 원본 데이터 백업
    cv::Mat originalMap = gridData->occupancyMap.clone();
    float originalResolution = gridData->resolution;
    
    // 새로운 크기 계산
    float scaleRatio = originalResolution / newResolution;
    int newWidth = static_cast<int>(gridData->width * scaleRatio);
    int newHeight = static_cast<int>(gridData->height * scaleRatio);
    
    qDebug() << "Resizing from" << gridData->width << "x" << gridData->height 
             << "to" << newWidth << "x" << newHeight;
    
    // OpenCV 리사이즈 (INTER_NEAREST: 장애물 정보 보존)
    cv::Mat resizedMap;
    cv::resize(originalMap, resizedMap, cv::Size(newWidth, newHeight), 0, 0, cv::INTER_NEAREST);
    
    // GridMap 데이터 업데이트
    gridData->occupancyMap = resizedMap;
    gridData->width = newWidth;
    gridData->height = newHeight;
    gridData->resolution = newResolution;
    
    // Origin은 동일하게 유지 (같은 실제 위치)
    
    qDebug() << "GridMap resampling completed - New size:" 
             << gridData->width << "x" << gridData->height 
             << "at" << gridData->resolution << "m/cell";
}

}