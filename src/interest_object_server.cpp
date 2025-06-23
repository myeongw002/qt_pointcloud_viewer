#include "interest_object_server.hpp"
#include <QDebug>

namespace Service {

InterestObjectServer::InterestObjectServer(
    rclcpp::Node::SharedPtr node, QObject* parent)
    : QObject(parent), node_(node) {
    
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
        "🔔 RECEIVED SERVICE REQUEST from robot: %s", 
        request->robot_id.c_str());
    
    QString robotId = QString::fromStdString(request->robot_id);
    
    // 요청 내용 상세 로그
    RCLCPP_INFO(node_->get_logger(), "📋 Request details:");
    RCLCPP_INFO(node_->get_logger(), "  Robot ID: %s", request->robot_id.c_str());
    RCLCPP_INFO(node_->get_logger(), "  Object classes: %zu", request->obj_class.size());
    RCLCPP_INFO(node_->get_logger(), "  Object IDs: %zu", request->obj_id.size());
    RCLCPP_INFO(node_->get_logger(), "  Object position: (%.3f, %.3f, %.3f)", 
                request->position.position.x, request->position.position.y, request->position.position.z);
    RCLCPP_INFO(node_->get_logger(), "  Timestamp: %d.%d", 
                request->stamp.sec, request->stamp.nanosec);
    
    try {
        auto& manager = ObjectManager::InterestObjectManager::instance();
        
        // CLEAR_ALL 처리 (특수 케이스)
        if (robotId == "CLEAR_ALL") {
            RCLCPP_INFO(node_->get_logger(), "🗑️  Clearing all objects...");
            manager.clearAllInterestObjects();
            response->result = true;
            emit serviceRequestReceived(robotId, 0);
            return;
        }
        
        // 배열 크기 검증 (길이 1 가정)
        if (request->obj_class.empty() || request->obj_id.empty()) {
            RCLCPP_ERROR(node_->get_logger(), 
                "❌ Empty arrays: obj_class(%zu) or obj_id(%zu)", 
                request->obj_class.size(), request->obj_id.size());
            response->result = false;
            return;
        }
        
        // 첫 번째 (유일한) 객체 처리 - 길이 1 배열 가정
        
        // ============================================================================
        // 1. 서비스 요청에서 객체 정보 추출
        // ============================================================================
        
        // Object Class (obj_class[0]) - 문자열 그대로 사용
        std::string objClassStr = request->obj_class[0];
        QString objectType = QString::fromStdString(objClassStr);  // 문자열 그대로 사용
        
        // Object ID (obj_id[0]) - uint16을 QString으로 변환
        uint16_t serviceObjectId = request->obj_id[0];
        QString objectId = QString::number(serviceObjectId);
        
        QString discoveredBy = robotId;
        
        RCLCPP_INFO(node_->get_logger(), 
            "🔍 Processing single object: service_id=%u, class=%s, robot=%s", 
            serviceObjectId, objClassStr.c_str(), robotId.toStdString().c_str());
        
        // ============================================================================
        // 2. 객체 위치 사용 (position을 물체의 위치로 직접 사용)
        // ============================================================================
        
        // 서비스 요청의 position을 물체 위치로 직접 사용
        Types::Vec3 rosObjectPos = poseToVec3(request->position);
        Types::Vec3 objectPosition = rosToOpenGLCoordinates(rosObjectPos);
        
        RCLCPP_INFO(node_->get_logger(), 
            "📍 Using service position as object location: ROS(%.3f,%.3f,%.3f) -> OpenGL(%.3f,%.3f,%.3f)",
            rosObjectPos.x, rosObjectPos.y, rosObjectPos.z,
            objectPosition.x, objectPosition.y, objectPosition.z);
        
        // ============================================================================
        // 3. Interest Object 등록 (서비스 값 직접 사용)
        // ============================================================================
        
        // Interest Object 등록 (문자열 타입으로 직접 전달)
        bool success = manager.registerInterestObjectFromService(
            objectId,        // obj_id[0]을 그대로 사용
            objectType,      // obj_class[0]을 문자열 그대로 사용
            objectPosition,  // position을 물체 위치로 직접 사용
            discoveredBy     // robot_id
        );
        
        if (success) {
            RCLCPP_INFO(node_->get_logger(), 
                "✅ Registered NEW object: id=%s, type=%s, robot=%s, pos=(%.3f,%.3f,%.3f)",
                objectId.toStdString().c_str(), objClassStr.c_str(), 
                discoveredBy.toStdString().c_str(),
                objectPosition.x, objectPosition.y, objectPosition.z);
        } else {
            RCLCPP_INFO(node_->get_logger(), 
                "🔄 Updated EXISTING object: id=%s (position/data refreshed)",
                objectId.toStdString().c_str());
        }
        
        // 응답 설정
        response->result = true;
        
        RCLCPP_INFO(node_->get_logger(), 
            "✅ Successfully processed object from robot %s at timestamp %d.%d",
            request->robot_id.c_str(), request->stamp.sec, request->stamp.nanosec);
        
        // 시그널 발생
        emit serviceRequestReceived(robotId, 1);  // 항상 1개 객체
        emit objectsRegistered(1);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), 
            "❌ Error processing interest objects request: %s", e.what());
        response->result = false;
    }
}

Types::Vec3 InterestObjectServer::poseToVec3(const geometry_msgs::msg::Pose& pose) {
    return Types::Vec3(pose.position.x, pose.position.y, pose.position.z);
}

Types::Vec3 InterestObjectServer::rosToOpenGLCoordinates(const Types::Vec3& rosPos) {
    // ROS 좌표계 (X=전진, Y=좌측, Z=상승) → OpenGL 좌표계 (X=우측, Y=상승, Z=후진)
    return Types::Vec3(-rosPos.y, rosPos.z, -rosPos.x);
}

} // namespace Service