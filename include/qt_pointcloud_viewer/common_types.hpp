#ifndef QT_POINTCLOUD_VIEWER_COMMON_TYPES_HPP
#define QT_POINTCLOUD_VIEWER_COMMON_TYPES_HPP

// STL
#include <vector>
#include <memory>
#include <chrono>

// Qt (Qt 메타타입 등록을 위해)
#include <QMetaType>
#include <QString>

// Third-party math
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// ROS 2 msgs
#include <geometry_msgs/msg/pose_stamped.hpp>

// OpenCV (GridMapData에서 사용)
#include <opencv2/opencv.hpp>

namespace Types   // 짧은 프로젝트 전용 네임스페이스
{
// ── 수학 ───────────────────────────────────────────────
using Vec3 = glm::vec3;
using Vec4 = glm::vec4;
using Mat4 = glm::mat4;
using Quat = glm::quat;

// ── 포인트클라우드 ────────────────────────────────────
using PointType = pcl::PointXYZI;
using Cloud = pcl::PointCloud<PointType>;
using CloudPtr = Cloud::Ptr;
using CloudConstPtr = Cloud::ConstPtr;

// ── 경로 / Pose ───────────────────────────────────────
using PoseStamped = geometry_msgs::msg::PoseStamped;
using Path = std::vector<PoseStamped>;
using PathPtr = std::shared_ptr<Path>;
using PathConstPtr = std::shared_ptr<const Path>;



// ── 그리드맵 구조체 정의 ───────────────────────────────
struct GridMapData {
    int width = 0;
    int height = 0;
    float resolution = 0.1f;
    float originX = 0.0f;
    float originY = 0.0f;
    
    // OpenCV 형태의 점유도 맵
    cv::Mat occupancyMap;
    
    // 렌더링용 데이터
    std::vector<float> data;
    
    // 메타 정보
    size_t sourceCloudHash = 0;
    std::chrono::steady_clock::time_point timestamp;
    
    // 유효성 검사
    bool isValid() const {
        return width > 0 && height > 0 && resolution > 0.0f && 
               !occupancyMap.empty() && 
               occupancyMap.rows == height && occupancyMap.cols == width;
    }
};

using GridMapPtr = std::shared_ptr<GridMapData>;
using GridMapConstPtr = std::shared_ptr<const GridMapData>;

// ── 시간 ──────────────────────────────────────────────
using TimePoint = std::chrono::steady_clock::time_point;
using Duration = std::chrono::steady_clock::duration;

// ── 색상 ──────────────────────────────────────────────
using ColorRGB = Vec3;
constexpr ColorRGB kDefaultColor{1.0f, 1.0f, 1.0f};

using ColorRGBA = glm::vec4;  // RGBA 형식 (투명도 포함)
constexpr ColorRGBA kDefaultColorRGBA{1.0f, 1.0f, 1.0f, 1.0f};  // 기본 흰색 불투명


// ── 관심물체 구조체 정의 ───────────────────────────────
enum class ObjectType {
    UNKNOWN = 0,
    OBSTACLE,
    CUSTOM
};

struct InterestObject {
    QString id;                    // 고유 ID
    ObjectType type;               // 물체 타입
    Vec3 position;                 // 3D 위치 (OpenGL 좌표계)
    float size;                     // 크기 (width, height, depth)
    ColorRGB color;                // 색상
    QString discoveredBy;          // 발견한 로봇 이름
    std::chrono::steady_clock::time_point timestamp;  // 등록 시간
    bool isActive;                 // 활성 상태
    QString description;           // 설명
    
    // 생성자
    InterestObject() 
        : type(ObjectType::UNKNOWN)
        , position(0.0f, 0.0f, 0.0f)
        , size(1.0f)
        , color(1.0f, 1.0f, 0.0f)  // 기본 노란색
        , timestamp(std::chrono::steady_clock::now())
        , isActive(true) {}
    
    InterestObject(const QString& objectId, ObjectType objectType, 
                  const Vec3& pos, const QString& robot)
        : id(objectId), type(objectType), position(pos)
        , size(1.0f), color(1.0f, 1.0f, 0.0f)
        , discoveredBy(robot), timestamp(std::chrono::steady_clock::now())
        , isActive(true) {}
};

// ── 유틸리티 함수들 ────────────────────────────────────
inline QString objectTypeToString(ObjectType type) {
    switch (type) {
        case ObjectType::OBSTACLE: return "Obstacle";
        case ObjectType::CUSTOM: return "Custom";
        default: return "Unknown";
    }
}

inline ObjectType stringToObjectType(const QString& str) {
    if (str == "Obstacle") return ObjectType::OBSTACLE;
    if (str == "Custom") return ObjectType::CUSTOM;
    return ObjectType::UNKNOWN;
}

using InterestObjectPtr = std::shared_ptr<InterestObject>;
using InterestObjectConstPtr = std::shared_ptr<const InterestObject>;

} // namespace Types

// ── Qt 메타타입 등록 ──────────────────────────────────
Q_DECLARE_METATYPE(Types::CloudConstPtr)
Q_DECLARE_METATYPE(Types::PathConstPtr)
Q_DECLARE_METATYPE(Types::GridMapPtr)
Q_DECLARE_METATYPE(Types::InterestObjectPtr)
Q_DECLARE_METATYPE(Types::ObjectType)

#endif // QT_POINTCLOUD_VIEWER_COMMON_TYPES_HPP