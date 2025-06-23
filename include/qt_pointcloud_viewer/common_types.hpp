#ifndef QT_POINTCLOUD_VIEWER_COMMON_TYPES_HPP
#define QT_POINTCLOUD_VIEWER_COMMON_TYPES_HPP

// STL
#include <vector>
#include <memory>
#include <chrono>
#include <random>

// Qt (Qt 메타타입 등록을 위해)
#include <QMetaType>
#include <QString>
#include <QDateTime>

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

// ── 관심물체 색상 팔레트 ──────────────────────────────
namespace InterestObjectColors {
    // 미리 정의된 색상 팔레트 (RGB 0.0~1.0 범위)
    const std::vector<ColorRGB> COLOR_PALETTE = {
        ColorRGB(1.0f, 0.2f, 0.2f),   // 빨간색
        ColorRGB(0.2f, 1.0f, 0.2f),   // 초록색  
        ColorRGB(0.2f, 0.2f, 1.0f),   // 파란색
        ColorRGB(1.0f, 1.0f, 0.2f),   // 노란색
        ColorRGB(1.0f, 0.2f, 1.0f),   // 마젠타
        ColorRGB(0.2f, 1.0f, 1.0f),   // 시안
        ColorRGB(1.0f, 0.6f, 0.2f),   // 주황색
        ColorRGB(0.8f, 0.2f, 1.0f),   // 보라색
        ColorRGB(0.2f, 0.8f, 0.4f),   // 연두색
        ColorRGB(1.0f, 0.4f, 0.6f),   // 분홍색
        ColorRGB(0.4f, 0.6f, 1.0f),   // 하늘색
        ColorRGB(0.8f, 0.8f, 0.2f),   // 올리브색
        ColorRGB(1.0f, 0.8f, 0.4f),   // 살구색
        ColorRGB(0.6f, 0.4f, 0.8f),   // 라벤더색
        ColorRGB(0.4f, 0.8f, 0.6f),   // 민트색
    };
    
    // 랜덤 색상 선택 함수
    inline ColorRGB getRandomColor() {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::uniform_int_distribution<> dis(0, COLOR_PALETTE.size() - 1);
        
        return COLOR_PALETTE[dis(gen)];
    }
    
    // 타입별 기본 색상 (백업용)
    inline ColorRGB getColorByType(const QString& type) {
        QString lowerType = type.toLower();
        
        if (lowerType.contains("obstacle") || lowerType.contains("person")) {
            return ColorRGB(1.0f, 0.2f, 0.2f);  // 빨간색
        } else if (lowerType.contains("car") || lowerType.contains("vehicle")) {
            return ColorRGB(0.2f, 0.2f, 1.0f);  // 파란색
        } else if (lowerType.contains("custom") || lowerType.contains("special")) {
            return ColorRGB(0.2f, 1.0f, 0.2f);  // 초록색
        } else {
            return ColorRGB(1.0f, 1.0f, 0.2f);  // 노란색 (기본)
        }
    }
}

// ── 관심물체 구조체 정의 (문자열 타입 사용) ─────────────
struct InterestObject {
    QString id;                    // 고유 ID
    QString type;                  // 물체 타입 (문자열로 변경)
    Vec3 position;                 // 3D 위치 (OpenGL 좌표계)
    float size;                    // 크기 (width, height, depth)
    ColorRGB color;                // 색상
    QString discoveredBy;          // 발견한 로봇 이름
    std::chrono::steady_clock::time_point timestamp;  // 등록 시간
    bool isActive;                 // 활성 상태
    QString description;           // 설명
    QDateTime lastUpdateTime;      // 마지막 업데이트 시간
    
    // 생성자
    InterestObject() 
        : type("unknown")
        , position(0.0f, 0.0f, 0.0f)
        , size(1.0f)
        , color(InterestObjectColors::getRandomColor())  // 랜덤 색상
        , timestamp(std::chrono::steady_clock::now())
        , isActive(true)
        , lastUpdateTime(QDateTime::currentDateTime()) {}
    
    InterestObject(const QString& objectId, const QString& objectType, 
                  const Vec3& pos, const QString& robot)
        : id(objectId), type(objectType), position(pos)
        , size(1.0f), discoveredBy(robot)
        , timestamp(std::chrono::steady_clock::now())
        , isActive(true)
        , lastUpdateTime(QDateTime::currentDateTime()) {
        
        // 랜덤 색상 설정
        color = InterestObjectColors::getRandomColor();
        setDescription();
    }

private:
    void setDescription() {
        description = QString("Object #%1 (%2) discovered by %3")
            .arg(id)
            .arg(type)
            .arg(discoveredBy);
    }
};

using InterestObjectPtr = std::shared_ptr<InterestObject>;
using InterestObjectConstPtr = std::shared_ptr<const InterestObject>;

} // namespace Types

// ── Qt 메타타입 등록 ──────────────────────────────────
Q_DECLARE_METATYPE(Types::CloudConstPtr)
Q_DECLARE_METATYPE(Types::PathConstPtr)
Q_DECLARE_METATYPE(Types::GridMapPtr)
Q_DECLARE_METATYPE(Types::InterestObjectPtr)

#endif // QT_POINTCLOUD_VIEWER_COMMON_TYPES_HPP