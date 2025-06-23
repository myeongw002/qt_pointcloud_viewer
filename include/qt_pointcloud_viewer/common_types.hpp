#ifndef QT_POINTCLOUD_VIEWER_COMMON_TYPES_HPP
#define QT_POINTCLOUD_VIEWER_COMMON_TYPES_HPP

// STL
#include <vector>
#include <memory>
#include <chrono>

// Qt (Qt 메타타입 등록을 위해)
#include <QMetaType>

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

} // namespace Types

// ── Qt 메타타입 등록 ──────────────────────────────────
Q_DECLARE_METATYPE(Types::CloudConstPtr)
Q_DECLARE_METATYPE(Types::PathConstPtr)
Q_DECLARE_METATYPE(Types::GridMapPtr)

#endif // QT_POINTCLOUD_VIEWER_COMMON_TYPES_HPP