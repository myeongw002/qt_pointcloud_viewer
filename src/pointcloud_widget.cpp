#include "pointcloud_widget.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <QDebug>

namespace Widget {

PointCloudWidget::PointCloudWidget(QWidget *parent) : QOpenGLWidget(parent) {
    lastMousePos_ = QPoint(0, 0);
    showIndicator_ = false;
    cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    connect(&hideTimer_, &QTimer::timeout, this, &PointCloudWidget::hideIndicator);
    updateCameraPosition();
}

PointCloudWidget::~PointCloudWidget() {
    makeCurrent();
    doneCurrent();
}

void PointCloudWidget::setNode(rclcpp::Node::SharedPtr ros_node) {
    node_ = ros_node;
    yaw_ = 0.0f;
    pitch_ = 0.0f;
    distance_ = 10.0f;
    focusPoint_ = glm::vec3(0.0f, 0.0f, 0.0f);
    showIndicator_ = false;

    hideTimer_.setSingleShot(true);
    connect(&hideTimer_, &QTimer::timeout, this, &PointCloudWidget::hideIndicator);

    updateCameraPosition();
}

void PointCloudWidget::setRobot(const QString& robot) {
    robotName_ = robot;
}


void PointCloudWidget::setTopicName(int index) {
    switch (index) {
        case 1:
            pcdTopic_ = "/tugv/viz_global_cloud";
            pathTopic_ = "/tugv/viz_path";
            break;
        case 2:
            pcdTopic_ = "/mugv/viz_global_cloud";
            pathTopic_ = "/mugv/viz_path";
            break;
        case 3:
            pcdTopic_ = "/sugv1/viz_global_cloud";
            pathTopic_ = "/sugv1/viz_path";
            break;
        case 4:
            pcdTopic_ = "/sugv2/viz_global_cloud";
            pathTopic_ = "/sugv2/viz_path";
            break;        
        case 5:
            pcdTopic_ = "/suav/viz_global_cloud";
            pathTopic_ = "/suav/viz_path";
            break;    
        default:
            pcdTopic_ = "/"; 
            pathTopic_ = "/";
            return;
    }

    rclcpp::QoS qos_settings(rclcpp::KeepLast(10));
    qos_settings.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_settings.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    
    pcdSubscribtion_ = this->node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        pcdTopic_, qos_settings,
        std::bind(&PointCloudWidget::pointCloudCallback, this, std::placeholders::_1));
    pathSubscribtion_ = this->node_->create_subscription<nav_msgs::msg::Path>(
        pathTopic_, qos_settings,
        std::bind(&PointCloudWidget::pathCallback, this, std::placeholders::_1));


    std::cout << "Subscribed to pcd topic : " << this->pcdTopic_ << std::endl;
    std::cout << "Subscribed to path topic : " << this->pathTopic_ << std::endl;
}

void PointCloudWidget::onCloudShared(const QString& robot, CloudConstPtr cloud) {
    // qDebug() << "Received cloud for robot:" << robot;
    std::lock_guard<std::mutex> lock(cloudMutex_);
    clouds_[robot] = cloud; // 여러 로봇의 클라우드 저장
    update();
}

std::string PointCloudWidget::getPcdTopic() {
    return pcdTopic_;
}

std::string PointCloudWidget::getPathTopic() {
    return pathTopic_;
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

    viewMatrix_ = glm::lookAt(
        cameraPos_,
        focusPoint_,
        glm::vec3(0, 1, 0)
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
    float x = distance_ * cos(glm::radians(pitch_)) * cos(glm::radians(yaw_));
    float y = distance_ * sin(glm::radians(pitch_));
    float z = distance_ * cos(glm::radians(pitch_)) * sin(glm::radians(yaw_));

    cameraPos_ = focusPoint_ + glm::vec3(x, y, z);
}

void PointCloudWidget::mousePressEvent(QMouseEvent *event) {
    lastMousePos_ = event->pos();
    showIndicator_ = true;
    hideTimer_.stop(); // 기존 타이머 중지
    update();
}

void PointCloudWidget::mouseMoveEvent(QMouseEvent *event) {
    if (event->buttons() & Qt::LeftButton) {
        float deltaX = event->x() - lastMousePos_.x();
        float deltaY = event->y() - lastMousePos_.y();

        yaw_ += deltaX * rotationSensitivity_;
        pitch_ += deltaY * rotationSensitivity_;

        pitch_ = std::max(-180.0f, std::min(180.0f, pitch_));

        lastMousePos_ = event->pos();
        showIndicator_ = true; // 드래그 중에는 인디케이터 유지
        updateCameraPosition();
        update();
    } else if (event->buttons() & Qt::MiddleButton) {
        float deltaX = (event->x() - lastMousePos_.x()) * 0.01f;
        float deltaY = (event->y() - lastMousePos_.y()) * 0.01f;

        glm::vec3 right = glm::cross(glm::vec3(0, 1, 0), glm::normalize(cameraPos_ - focusPoint_));
        glm::vec3 up = glm::cross(glm::normalize(cameraPos_ - focusPoint_), right);
        focusPoint_ += -(right * deltaX) + (up * deltaY);

        lastMousePos_ = event->pos();
        showIndicator_ = true; // 드래그 중에는 인디케이터 유지
        updateCameraPosition();
        update();
    }
}

void PointCloudWidget::mouseReleaseEvent(QMouseEvent *event) {
    Q_UNUSED(event);
    hideTimer_.start(timerInterval_); // 마우스 뗄 때 타이머 시작
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

void PointCloudWidget::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(cloudMutex_);
    pcl::fromROSMsg(*msg, *cloud_);
    update();
}

void PointCloudWidget::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(pathMutex_);
    path_ = msg->poses; // poses 배열 복사
    update(); // GUI 갱신
}

void PointCloudWidget::drawPoints() {
    std::lock_guard<std::mutex> lock(cloudMutex_);

    glPointSize(2.0f);
    glBegin(GL_POINTS);

    for (auto it = clouds_.cbegin(); it != clouds_.cend(); ++it) {
        const QString& robotName = it.key();
        
        // COMBINED인 경우 모든 로봇의 점을 그리고, 아니면 해당 로봇만 그리기
        if (robotName_ != "COMBINED" && robotName != robotName_) {
            continue; // 현재 로봇과 일치하지 않으면 건너뛰기
        }
        
        const auto& cloud = it.value();

        if (cloud && !cloud->empty()) {
            // qDebug() << "Drawing cloud for robot:" << robotName;
            
            // 로봇별로 다른 색상 설정 (선택사항)
            if (robotName == "TUGV") {
                glColor3f(1.0f, 0.0f, 0.0f); // 빨강
            } else if (robotName == "MUGV") {
                glColor3f(0.0f, 1.0f, 0.0f); // 초록
            } else if (robotName == "SUGV1") {
                glColor3f(0.0f, 0.0f, 1.0f); // 파랑
            } else if (robotName == "SUGV2") {
                glColor3f(1.0f, 1.0f, 0.0f); // 노랑
            } else if (robotName == "SUAV") {
                glColor3f(1.0f, 0.0f, 1.0f); // 자홍
            } else {
                glColor3f(0.0f, 1.0f, 0.0f); // 기본 초록
            }
            
            for (const auto& point : cloud->points) {
                glVertex3f(point.x, point.y, point.z);
            }
        }
    }
    glEnd();
}

void PointCloudWidget::drawPath() {
    std::lock_guard<std::mutex> lock(pathMutex_);
    if (path_.empty()) return;

    glLineWidth(2.0f); // 선 두께 설정
    glBegin(GL_LINE_STRIP); // 포인트를 연결하는 선
    glColor3f(0.0f, 0.0f, 1.0f); // 파란색 경로

    for (const auto& pose : path_) {
        const auto& position = pose.pose.position;
        glVertex3f(position.x, position.y, position.z);
    }
    glEnd();
}

void PointCloudWidget::drawAxes() {
    glBegin(GL_LINES);
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(1.0f, 0.0f, 0.0f);
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 1.0f, 0.0f);
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 1.0f);
    glEnd();
}

void PointCloudWidget::drawGrid() {
    glColor3f(0.3f, 0.3f, 0.3f);
    glBegin(GL_LINES);
    for (float i = -10.0f; i <= 10.0f; i += 1.0f) {
        glVertex3f(i, 0.0f, -10.0f);
        glVertex3f(i, 0.0f, 10.0f);
        glVertex3f(-10.0f, 0.0f, i);
        glVertex3f(10.0f, 0.0f, i);
    }
    glEnd();
}

void PointCloudWidget::drawCameraIndicator() {
    glDisable(GL_DEPTH_TEST);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();

    glTranslatef(focusPoint_.x, focusPoint_.y, focusPoint_.z);

    float cylinderRadius = 0.3f;  // Fixed size regardless of zoom
    float cylinderHeight = 0.1f;  // Increase this for more thickness
    float alpha = 0.7f;  // Transparency (0.0 = fully transparent, 1.0 = fully opaque)
    int segments = 30;  // Smoother cylinder

    glColor4f(1.0f, 1.0f, 0.0f, alpha); 
    
    // ✅ Draw the Top Disk
    glBegin(GL_TRIANGLE_FAN);
    glVertex3f(0.0f, 0.0f, cylinderHeight / 2.0f);
    for (int i = 0; i <= segments; ++i) {
        float angle = 2.0f * M_PI * float(i) / float(segments);
        float x = cylinderRadius * cos(angle);
        float y = cylinderRadius * sin(angle);
        glVertex3f(x, y, cylinderHeight / 2.0f);
    }
    glEnd();

    // ✅ Draw the Bottom Disk
    glBegin(GL_TRIANGLE_FAN);
    glVertex3f(0.0f, 0.0f, -cylinderHeight / 2.0f);
    for (int i = 0; i <= segments; ++i) {
        float angle = 2.0f * M_PI * float(i) / float(segments);
        float x = cylinderRadius * cos(angle);
        float y = cylinderRadius * sin(angle);
        glVertex3f(x, y, -cylinderHeight / 2.0f);
    }
    glEnd();

    // ✅ Draw the Side Surface (Cylinder Walls)
    glBegin(GL_QUAD_STRIP);
    for (int i = 0; i <= segments; ++i) {
        float angle = 2.0f * M_PI * float(i) / float(segments);
        float x = cylinderRadius * cos(angle);
        float y = cylinderRadius * sin(angle);

        glVertex3f(x, y, -cylinderHeight / 2.0f);
        glVertex3f(x, y, cylinderHeight / 2.0f);
    }
    glEnd();
}

} // namespace Widget