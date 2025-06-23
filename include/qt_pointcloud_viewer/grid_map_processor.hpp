// grid_map_processor.hpp
#ifndef GRID_MAP_PROCESSOR_HPP
#define GRID_MAP_PROCESSOR_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <vector>
#include <QString>
#include "common_types.hpp"  // Types::GridMapData 사용

namespace GridMap {

struct GridMapParameters {
    float resolution = 0.1f;         // m/cell
    float mapSizeX = 100.0f;         // meters
    float mapSizeY = 100.0f;         // meters
    float heightFilterMin = 0.1f;    // 바닥 필터 (m)
    float heightFilterMax = 1.0f;    // 천장 필터 (m)
    float occupancyThreshold = 0.6f; // 점유 확률 임계값
    bool enableMorphology = true;    // 모폴로지 연산 사용
    int morphKernelSize = 3;         // 커널 크기
};

// Types::GridMapData 사용 (이제 같은 타입)
using GridMapData = Types::GridMapData;

// Path 데이터 구조체
struct ProjectedPath {
    std::vector<cv::Point2f> points;  // 그리드 좌표계의 경로 점들
    std::vector<float> orientations;  // 각 점에서의 방향각 (라디안)
    QString robotName;
    glm::vec3 color;
    bool isValid = false;
};

class GridMapProcessor {
public:
    static std::shared_ptr<GridMapData> convertPointCloudToGridMap(
        const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud,
        const GridMapParameters& params = GridMapParameters{}
    );
    
    static std::shared_ptr<GridMapData> mergeGridMaps(
        const std::vector<std::shared_ptr<GridMapData>>& maps,
        const GridMapParameters& params = GridMapParameters{}
    );
    
    static cv::Mat visualizeGridMap(
        const std::shared_ptr<GridMapData>& gridData,
        bool showProbability = false
    );

    static bool processPointCloud(
        const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud,
        const GridMapParameters& params,
        GridMapData& gridData
    );
    
    // Path 프로젝션 함수들
    static ProjectedPath projectPathToGridMap(
        const std::vector<geometry_msgs::msg::PoseStamped>& path,
        const GridMapData& gridData,
        const QString& robotName,
        const glm::vec3& color
    );
    
    static cv::Mat drawPathOnGridMap(
        const cv::Mat& gridMap,
        const ProjectedPath& projectedPath,
        float pathWidth = 3.0f
    );
    
    static cv::Mat combineGridMapWithPaths(
        const cv::Mat& gridMap,
        const std::vector<ProjectedPath>& paths
    );

private:
    static std::vector<pcl::PointXYZI> filterPointsByHeight(
        const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud,
        float minHeight, float maxHeight
    );
    
    static cv::Rect2f calculateBoundingBox(
        const std::vector<pcl::PointXYZI>& points
    );
    
    static cv::Mat applyMorphologyOperations(
        const cv::Mat& inputMap, int kernelSize
    );
    
    static size_t calculateCloudHash(
        const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud
    );
    
    // Mat을 Vector로 변환
    static void convertMatToVector(GridMapData& gridData);
    
    // Path 프로젝션 헬퍼 함수들
    static cv::Point2f worldToGrid(
        float worldX, float worldY,
        const GridMapData& gridData
    );
    
    static float calculateOrientation(
        const geometry_msgs::msg::PoseStamped& pose
    );
    
    static void drawArrow(
        cv::Mat& image,
        const cv::Point2f& point,
        float orientation,
        const cv::Scalar& color,
        float size = 5.0f
    );
};

} // namespace GridMap

#endif // GRID_MAP_PROCESSOR_HPP