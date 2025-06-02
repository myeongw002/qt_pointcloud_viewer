#include "pointcloud_widget.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>

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

void PointCloudWidget::setTopicName(int index) {
    switch (index) {
        case 1:
            topicName_ = "/tugv/viz_global_cloud";
            break;
        case 2:
            topicName_ = "/mugv/viz_global_cloud";
            break;
        case 3:
            topicName_ = "/sugv1/viz_global_cloud";
            break;
        case 4:
            topicName_ = "/sugv2/viz_global_cloud";
            break;        
        case 5:
            topicName_ = "/suav/viz_global_cloud";
            break;    
        default:
            topicName_ = "/"; 
            return;
    }

    std::cout << "Topic name set to: " << topicName_ << std::endl;

    rclcpp::QoS qos_settings(rclcpp::KeepLast(10));
    qos_settings.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_settings.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    
    subscription_ = this->node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        topicName_, qos_settings,
        std::bind(&PointCloudWidget::pointCloudCallback, this, std::placeholders::_1));
    
    std::cout << "Subscribed to " << this->topicName_ << std::endl;
}

std::string PointCloudWidget::getTopicName() {
    return topicName_;
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

    // if (cloud_ && !cloud_->empty()) {
    //     float centerX = 0.0f, centerY = 0.0f, centerZ = 0.0f;
    //     for (const auto& point : *cloud_) {
    //         centerX += point.x;
    //         centerY += point.y;
    //         centerZ += point.z;
    //     }
    //     centerX /= cloud_->size();
    //     centerY /= cloud_->size();
    //     centerZ /= cloud_->size();
    //     focusPoint_ = glm::vec3(centerX, centerY, centerZ);

    //     std::cout << "Focus Point Updated: (" << centerX << ", " << centerY << ", " << centerZ << ")\n";
    // }

    // updateCameraPosition();
    update();
}

void PointCloudWidget::drawPoints() {
    std::lock_guard<std::mutex> lock(cloudMutex_);
    if (!cloud_ || cloud_->empty()) return;

    glPointSize(2.0f);
    glBegin(GL_POINTS);
    for (const auto& point : *cloud_) {
        glColor3f(0.0f, 1.0f, 0.0f);
        glVertex3f(point.x, point.y, point.z);
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