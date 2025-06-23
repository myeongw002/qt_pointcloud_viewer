#ifndef INTEREST_OBJECT_SERVER_HPP
#define INTEREST_OBJECT_SERVER_HPP

#include <rclcpp/rclcpp.hpp>
#include "u2_icd_pkg/srv/interest_objs.hpp"
#include "interest_object_manager.hpp"
#include "common_types.hpp"
#include <QObject>

namespace Service {

class InterestObjectServer : public QObject {
    Q_OBJECT

public:
    explicit InterestObjectServer(rclcpp::Node::SharedPtr node, QObject* parent = nullptr);
    ~InterestObjectServer() = default;

signals:
    void serviceRequestReceived(const QString& robotId, int objectCount);
    void objectsRegistered(int count);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Service<u2_icd_pkg::srv::InterestObjs>::SharedPtr service_;
    
    // 서비스 콜백
    void handleInterestObjectsRequest(
        const std::shared_ptr<u2_icd_pkg::srv::InterestObjs::Request> request,
        std::shared_ptr<u2_icd_pkg::srv::InterestObjs::Response> response);
    
    // 헬퍼 함수들 (ObjectType 관련 함수 제거)
    Types::Vec3 poseToVec3(const geometry_msgs::msg::Pose& pose);
    Types::Vec3 rosToOpenGLCoordinates(const Types::Vec3& rosPos);
};

} // namespace Service

#endif // INTEREST_OBJECT_SERVER_HPP