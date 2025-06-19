// grid_map_processor.hpp
#ifndef GRID_MAP_PROCESSOR_HPP
#define GRID_MAP_PROCESSOR_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <memory>
#include <vector>

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

struct GridMapData {
    cv::Mat occupancyMap;               // 단순 이진 맵: 0=Unknown, 255=Occupied
    float resolution;                   // m/cell
    float originX, originY;             // 맵 원점 (world coordinates)
    int width, height;                  // 맵 크기 (cells)
    size_t sourceCloudHash;             // 소스 클라우드 해시
    std::chrono::steady_clock::time_point timestamp;
    
    bool isValid() const {
        return !occupancyMap.empty() && width > 0 && height > 0;
    }
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
};

} // namespace GridMap

#endif // GRID_MAP_PROCESSOR_HPP