#include "interest_object_server.hpp"
#include <QDebug>

namespace Service {

InterestObjectServer::InterestObjectServer(
    rclcpp::Node::SharedPtr node, QObject* parent)
    : QObject(parent), node_(node) {
    
    // 서비스 서버 생성
    service_ = node_->create_service<u2_icd_pkg::srv::InterestObjs>(
        "interest_objects_service",
        std::bind(&InterestObjectServer::handleInterestObjectsRequest,
                  this, std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(node_->get_logger(), "Interest Objects Service Server started");
    qDebug() << "Interest Objects Service Server initialized and ready";
}

void InterestObjectServer::handleInterestObjectsRequest(
    const std::shared_ptr<u2_icd_pkg::srv::InterestObjs::Request> request,
    std::shared_ptr<u2_icd_pkg::srv::InterestObjs::Response> response) {
    
    RCLCPP_INFO(node_->get_logger(), 
        "Received interest objects request from robot: %s with %zu objects", 
        request->robot_id.c_str(), request->obj_class.size());
    
    QString robotId = QString::fromStdString(request->robot_id);
    int registeredCount = 0;
    
    try {
        auto& manager = ObjectManager::InterestObjectManager::instance();
        
        // 각 객체를 처리
        for (size_t i = 0; i < request->obj_class.size(); ++i) {
            // 배열 크기 확인
            if (i >= request->obj_id.size()) {
                RCLCPP_WARN(node_->get_logger(), "Object ID array size mismatch at index %zu", i);
                continue;
            }
            
            // 객체 정보 추출
            std::string objClassStr = request->obj_class[i];
            Types::ObjectType objType = stringToObjectType(objClassStr);
            
            // 로봇 위치를 객체 위치로 사용 (실제로는 bbox 정보를 사용해 3D 위치 계산 가능)
            Types::Vec3 rosPosition = poseToVec3(request->position);
            Types::Vec3 openglPosition = rosToOpenGLCoordinates(rosPosition);
            
            // 바운딩 박스 정보가 있다면 활용 (선택적)
            if (i < request->bbox.size()) {
                const auto& bbox = request->bbox[i];
                // bbox 정보를 이용해 위치 오프셋 적용 가능
                // 현재는 단순히 로봇 위치 사용
                RCLCPP_DEBUG(node_->get_logger(), 
                    "BBox center: (%.3f, %.3f), size: %.3fx%.3f",
                    bbox.center.x, bbox.center.y, bbox.size_x, bbox.size_y);
            }
            
            // Interest Object 등록
            QString objectId = manager.registerInterestObject(objType, robotId, openglPosition);
            
            if (!objectId.isEmpty()) {
                registeredCount++;
                RCLCPP_DEBUG(node_->get_logger(), 
                    "Registered object: %s (type: %s, ID: %s)",
                    objClassStr.c_str(), 
                    Types::objectTypeToString(objType).toStdString().c_str(),
                    objectId.toStdString().c_str());
            }
        }
        
        // 응답 설정
        response->result = true;
        
        RCLCPP_INFO(node_->get_logger(), 
            "Successfully registered %d/%zu objects from robot %s",
            registeredCount, request->obj_class.size(), request->robot_id.c_str());
        
        // 시그널 발생
        emit serviceRequestReceived(robotId, static_cast<int>(request->obj_class.size()));
        emit objectsRegistered(registeredCount);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), 
            "Error processing interest objects request: %s", e.what());
        response->result = false;
    }
}

Types::ObjectType InterestObjectServer::stringToObjectType(const std::string& objClass) {
    // 서비스에서 오는 문자열을 ObjectType으로 변환
    if (objClass == "obstacle" || objClass == "Obstacle") {
        return Types::ObjectType::OBSTACLE;
    } else if (objClass == "custom" || objClass == "Custom") {
        return Types::ObjectType::CUSTOM;
    } else if (objClass == "person" || objClass == "car" || objClass == "object") {
        // 일반적인 검출 객체들은 OBSTACLE로 분류
        return Types::ObjectType::OBSTACLE;
    }
    
    return Types::ObjectType::UNKNOWN;
}

Types::Vec3 InterestObjectServer::poseToVec3(const geometry_msgs::msg::Pose& pose) {
    return Types::Vec3(pose.position.x, pose.position.y, pose.position.z);
}

Types::Vec3 InterestObjectServer::rosToOpenGLCoordinates(const Types::Vec3& rosPos) {
    // ROS 좌표계 (X=전진, Y=좌측, Z=상승) → OpenGL 좌표계 (X=우측, Y=상승, Z=후진)
    return Types::Vec3(-rosPos.y, rosPos.z, -rosPos.x);
}

} // namespace Service