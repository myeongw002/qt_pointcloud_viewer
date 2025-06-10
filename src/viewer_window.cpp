/* viewer_window.cpp */
#include "viewer_window.hpp"
#include <pcl_conversions/pcl_conversions.h>


namespace Widget {

ViewerWindow::ViewerWindow(const QString &robot,
                           rclcpp::Node::SharedPtr node,
                           QOpenGLContext *share)
    : node_(std::move(node))
{
    if (share) {
        auto *ctx = new QOpenGLContext(this);
        ctx->setShareContext(share);
        ctx->setFormat(share->format());
        if (!ctx->create()) {
            qWarning("Failed to create shared OpenGL context");
        }
    }
    setTitle(QString("%1 – PointCloud").arg(robot));

    sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        toTopic(robot).toStdString(), 10,
        std::bind(&ViewerWindow::pointCloudCallback, this, std::placeholders::_1));
}

QString ViewerWindow::toTopic(const QString &r) const
{
    static const QMap<QString,QString> map{
        {"TUGV","/tugv/viz_global_cloud"},
        {"TUGV","/mugv/viz_global_cloud"},
        {"SUGV","/sugv1/viz_global_cloud"},
        {"SUGV","/sugv2/viz_global_cloud"},
        {"SUAV"  ,"/suav/viz_global_cloud"} };
    return map.value(r,"/");
}

void ViewerWindow::initializeGL()
{
    initializeOpenGLFunctions();
    glEnable(GL_DEPTH_TEST);
    glClearColor(0,0,0,1);
}

void ViewerWindow::resizeGL(int w,int h)
{
    glViewport(0,0,w,h);
    proj_ = glm::perspective(glm::radians(60.f), float(w)/h, 0.1f, 10'000.f);
}

void ViewerWindow::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);   glLoadMatrixf(&proj_[0][0]);
    glMatrixMode(GL_MODELVIEW);    glLoadIdentity();

    std::lock_guard<std::mutex> lk(mtx_);
    glPointSize(2.f);
    glBegin(GL_POINTS);
    for (auto &p : *cloud_) {
        glColor3f(0,1,0);
        glVertex3f(p.x,p.y,p.z);
    }
    glEnd();
}

void ViewerWindow::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    std::lock_guard<std::mutex> lk(mtx_);
    pcl::fromROSMsg(*msg,*cloud_);
    update();               // GUI 스레드 호출 안전(싱글스레드 ROS 스핀 가정)
}


} // namespace Widget