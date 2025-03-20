#include "pointcloud_widget.h"
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <qdebug.h>
PointCloudWidget::PointCloudWidget(QWidget *parent)
    : QOpenGLWidget(parent), cloud(new pcl::PointCloud<pcl::PointXYZ>) {

    rotationX = rotationY = 0.0f;
    panX = panY = 0.0f;
    zoom = -5.0f;
    showIndicator = false;  // ✅ Initially hidden
    
    hideTimer.setSingleShot(true);
    connect(&hideTimer, &QTimer::timeout, this, &PointCloudWidget::hideIndicator);
}

void PointCloudWidget::setNode(rclcpp::Node::SharedPtr ros_node) {
    this->node = ros_node;
    // std::cout << "🔹 setSubscription() called on object: " << this << std::endl;

    rotationX = rotationY = 0.0f;
    panX = panY = 0.0f;
    zoom = -5.0f;
    showIndicator = false;  // ✅ Initially hidden
    
    hideTimer.setSingleShot(true);
    connect(&hideTimer, &QTimer::timeout, this, &PointCloudWidget::hideIndicator);
}


void PointCloudWidget::setStartFlag(bool flag) {
    // std::cout << "setStartFlag called on object at: " << this << std::endl;
    // std::cout << "setStartFlag called with flag: " << flag << std::endl;
    this->start_flag = flag;
    // std::cout << "start_flag is now: " << this->start_flag << std::endl;
}

void PointCloudWidget::setTopicName(int index) {
    switch (index) {
        case 1:
            this->topic_name = "/mugv/os128_pts";
            break;
        case 2:
            this->topic_name = "/sugv2/os64_pts";
            break;
        default:
            this->topic_name = "/"; 
            this->start_flag = false;  // ✅ Disable subscription if no valid topic is selected
            return;
    }

    std::cout << "Topic name set to: " << this->topic_name << std::endl;

    rclcpp::QoS qos_settings(rclcpp::KeepLast(10));
    qos_settings.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_settings.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    
    subscription = this->node->create_subscription<sensor_msgs::msg::PointCloud2>(
        this->topic_name, qos_settings,
        std::bind(&PointCloudWidget::pointCloudCallback, this, std::placeholders::_1));
    
    std::cout << "Subscribed to " << this->topic_name <<std::endl;
}

std::string PointCloudWidget::getTopicName() {
    return this->topic_name;
}


void PointCloudWidget::initializeGL() {
    qDebug() << "initializeGL() called";  // ✅ 로그 출력 추가
    glEnable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float aspect = width() / static_cast<float>(height());
    glFrustum(-aspect, aspect, -1.0, 1.0, 1.0, 100.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void PointCloudWidget::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // ✅ Apply camera transformations (panning, zooming, rotation) to the scene
    glLoadIdentity();
    glTranslatef(panX, panY, zoom);
    glRotatef(rotationX, 1.0f, 0.0f, 0.0f);
    glRotatef(rotationY, 0.0f, 1.0f, 0.0f);

    // ✅ Draw the Point Cloud
    glBegin(GL_POINTS);
    glColor3f(0.0f, 1.0f, 0.0f);  // Green color
    for (const auto &point : cloud->points) {
        glVertex3f(point.x, point.y, point.z);
    }
    glEnd();

    // ✅ Only draw the indicator when rotating or panning
    if (showIndicator) {
        // ✅ Switch to 2D screen space for the indicator
        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();
        float aspect = float(width()) / float(height());
        glOrtho(-aspect, aspect, -1, 1, -1, 1);  // ✅ Corrected for aspect ratio
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        // ✅ Apply rotation to the indicator (matches camera rotation)
        glRotatef(rotationX, 1.0f, 0.0f, 0.0f);
        glRotatef(rotationY, 0.0f, 1.0f, 0.0f);

        // ✅ Enable transparency
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        // ✅ Define indicator parameters (adjust transparency with alpha)
        float cylinderRadius = 0.03f;  // Fixed size regardless of zoom
        float cylinderHeight = 0.01f;  // Increase this for more thickness
        float alpha = 0.5f;  // Transparency (0.0 = fully transparent, 1.0 = fully opaque)
        int segments = 30;  // Smoother cylinder

        glColor4f(1.0f, 1.0f, 0.0f, alpha);  // ✅ Semi-transparent Yellow color

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

        // ✅ Disable blending after drawing the indicator
        glDisable(GL_BLEND);

        // ✅ Restore previous projection
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();
        glMatrixMode(GL_MODELVIEW);
    }
}

void PointCloudWidget::resizeGL(int w, int h) {
    if (h == 0) h = 1;
    float aspect = static_cast<float>(w) / static_cast<float>(h);
    
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glFrustum(-aspect, aspect, -1.0, 1.0, 1.0, 100.0);
    glMatrixMode(GL_MODELVIEW);
}

// ✅ Handle mouse press (store initial position)
void PointCloudWidget::mousePressEvent(QMouseEvent *event) {
    lastMousePos = event->pos();
    showIndicator = true;  // ✅ Show indicator
}

void PointCloudWidget::mouseReleaseEvent(QMouseEvent *event) {
    // ✅ Stop showing the indicator when the mouse is released
    showIndicator = false;
    hideTimer.stop();  // ✅ Stop the timer
    update();  // ✅ Trigger repaint
}

void PointCloudWidget::mouseMoveEvent(QMouseEvent *event) {
    int dx = event->x() - lastMousePos.x();
    int dy = event->y() - lastMousePos.y();

    if (event->buttons() & Qt::LeftButton) {
        // ✅ Rotate X-axis (Up/Down) with limits
        rotationX += dy * 0.5f;
        rotationX = std::clamp(rotationX, -180.0f, 180.0f);  // ✅ Limit to -80 to 80 degrees

        // ✅ Rotate Y-axis (Left/Right) with limits
        rotationY += dx * 0.5f;
    } 
    else if (event->buttons() & Qt::MiddleButton) {
        // ✅ Pan the camera
        panX += dx * 0.01f;
        panY -= dy * 0.01f;
    }

    showIndicator = true;
    lastMousePos = event->pos();
    update();
}


void PointCloudWidget::wheelEvent(QWheelEvent *event) {
    float delta = event->angleDelta().y();
    zoom += delta * 0.01f;
    update();  // ✅ Trigger repaint
}

void PointCloudWidget::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (this->start_flag == false) {
        // std::cout << "pointCloudCallback on object at: " << this << " | start_flag: " << this->start_flag << std::endl;
        // std::cout << "start_flag is false, ignoring message." << this->start_flag <<std::endl;
        return;  // ✅ Ignore messages if start_flag is false
    }

    pcl::fromROSMsg(*msg, *cloud);
    
    if (cloud->points.empty()) {
        std::cout << "Received an empty point cloud!" << std::endl;
    } else {
        std::cout << "Received point cloud with " << cloud->points.size() << " points." << std::endl;
    }

    update();     
}

void PointCloudWidget::hideIndicator() {
    showIndicator = false;
    update();  // ✅ Trigger repaint to remove indicator
}
