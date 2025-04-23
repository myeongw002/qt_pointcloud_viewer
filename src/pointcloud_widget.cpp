#include "pointcloud_widget.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <qdebug.h>

namespace Widget {
    PointCloudWidget::PointCloudWidget(QWidget *parent)
    : QOpenGLWidget(parent), cloud_(new pcl::PointCloud<pcl::PointXYZI>) {

    rotationX_ = rotationY_ = 0.0f;
    panX_ = panY_ = 0.0f;
    zoom_ = -5.0f;
    showIndicator_ = false;  // ✅ Initially hidden
    
    hideTimer_.setSingleShot(true);
    connect(&hideTimer_, &QTimer::timeout, this, &PointCloudWidget::hideIndicator);
}

    void PointCloudWidget::setNode(rclcpp::Node::SharedPtr ros_node) {
        node_ = ros_node;
        // std::cout << "🔹 setSubscription() called on object: " << this << std::endl;

        rotationX_ = rotationY_ = 0.0f;
        panX_ = panY_ = 0.0f;
        zoom_ = -5.0f;
        showIndicator_ = false;  // ✅ Initially hidden
        
        hideTimer_.setSingleShot(true);
        connect(&hideTimer_, &QTimer::timeout, this, &PointCloudWidget::hideIndicator);
    }

    void PointCloudWidget::setTopicName(int index) {
        switch (index) {
            case 1:
                topicName_ = "/mugv/os128_pts";
                break;
            case 2:
                topicName_ = "/tugv/os64_pts";
                break;
            case 3:
                topicName_ = "/sugv1/os64_pts";
                break;
            case 4:
                topicName_ = "/sugv2/os64_pts";
                break;        
            case 5:
                topicName_ = "/suav/livox/lidar";
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
        
        std::cout << "Subscribed to " << this->topicName_ <<std::endl;
    }

    std::string PointCloudWidget::getTopicName() {
        return topicName_;
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
        glTranslatef(panX_, panY_, zoom_);
        glRotatef(rotationX_, 1.0f, 0.0f, 0.0f);
        glRotatef(rotationY_, 0.0f, 1.0f, 0.0f);
        
        if (showGrid_) {
            drawGrid();
        }
        if (showAxes_) {
            drawAxes();
        }

        drawPoints();


        // ✅ Only draw the indicator when rotating or panning
        if (showIndicator_) {
            // ✅ Switch to 2D screen space for the indicator
            glMatrixMode(GL_PROJECTION);
            glPushMatrix();
            glLoadIdentity();
            float aspect = float(width()) / float(height());
            glOrtho(-aspect, aspect, -1, 1, -1, 1);  // ✅ Corrected for aspect ratio
            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();

            // ✅ Apply rotation to the indicator (matches camera rotation)
            glRotatef(rotationX_, 1.0f, 0.0f, 0.0f);
            glRotatef(rotationY_, 0.0f, 1.0f, 0.0f);

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
        lastMousePos_ = event->pos();
        showIndicator_ = true;  // ✅ Show indicator
    }

    void PointCloudWidget::mouseReleaseEvent(QMouseEvent *event) {
        // ✅ Stop showing the indicator when the mouse is released
        showIndicator_ = false;
        hideTimer_.stop();  // ✅ Stop the timer
        update();  // ✅ Trigger repaint
    }

    void PointCloudWidget::mouseMoveEvent(QMouseEvent *event) {
        int dx = event->x() - lastMousePos_.x();
        int dy = event->y() - lastMousePos_.y();

        if (event->buttons() & Qt::LeftButton) {
            // ✅ Rotate X-axis (Up/Down) with limits
            rotationX_ += dy * 0.5f;
            rotationX_ = std::clamp(rotationX_, -180.0f, 180.0f);  // ✅ Limit to -80 to 80 degrees

            // ✅ Rotate Y-axis (Left/Right) with limits
            rotationY_ += dx * 0.5f;
        } 
        else if (event->buttons() & Qt::MiddleButton) {
            // ✅ Pan the camera
            panX_ += dx * 0.01f;
            panY_ -= dy * 0.01f;
        }

        showIndicator_ = true;
        lastMousePos_ = event->pos();
        update();
    }


    void PointCloudWidget::wheelEvent(QWheelEvent *event) {
        float delta = event->angleDelta().y();
        zoom_ += delta * 0.01f;
        update();  // ✅ Trigger repaint
    }

    void PointCloudWidget::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::fromROSMsg(*msg, *cloud_);
        
        if (cloud_->points.empty()) {
            std::cout << "Received an empty point cloud!" << std::endl;
        } else {
            std::cout << "Received point cloud with " << cloud_->points.size() << " points." << std::endl;
        }

        update();     
    }

    void PointCloudWidget::hideIndicator() {
        showIndicator_ = false;
        update();  // ✅ Trigger repaint to remove indicator
    }


    void PointCloudWidget::drawPoints() {
        /*
        glBegin(GL_POINTS);
        for (const auto &point : cloud_->points) {
            // intensity 값을 [0, 1] 범위로 정규화
            float normalized_intensity = point.intensity / 255.0f;  // 보통 0~255 범위

            float r = normalized_intensity;
            float g = 1.0f - std::abs(normalized_intensity - 0.5f) * 2.0f;
            float b = 1.0f - normalized_intensity;
            glColor3f(r, g, b);
            glVertex3f(point.x, point.y, point.z);
        }
        glEnd();
        */
        glBegin(GL_POINTS);
        glColor3f(0.0f, 1.0f, 0.0f);  // Green color
        for (const auto &point : cloud_->points) {
            glVertex3f(point.x, point.y, point.z);
        }
        glEnd();
    }    

    void PointCloudWidget::drawAxes()
    {
        glLineWidth(3.0f);
        glBegin(GL_LINES);

        // X-axis (Red)
        glColor3f(1.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(1.0f, 0.0f, 0.0f);

        // Y-axis (Green)
        glColor3f(0.0f, 1.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, 1.0f, 0.0f);

        // Z-axis (Blue)
        glColor3f(0.0f, 0.0f, 1.0f);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, 1.0f);

        glEnd();
        glLineWidth(1.0f);
    }

    void PointCloudWidget::drawGrid() {
        glBegin(GL_LINES);
        glColor3f(0.5f, 0.5f, 0.5f);  // Gray color
        for (float i = -10.0f; i <= 10.0f; i += 1.0f) {
            glVertex3f(i, -10.0f, 0.0f);
            glVertex3f(i, 10.0f, 0.0f);
            glVertex3f(-10.0f, i, 0.0f);
            glVertex3f(10.0f, i, 0.0f);
        }
        glEnd();
    }

    void PointCloudWidget::setShowAxes(bool show) {
        showAxes_ = show;
        update();  // ✅ Trigger repaint
    }
    void PointCloudWidget::setShowGrid(bool show) {
        showGrid_ = show;
        update();  // ✅ Trigger repaint
    }
}
