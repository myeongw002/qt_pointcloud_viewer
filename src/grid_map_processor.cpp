// grid_map_processor.cpp
#include "grid_map_processor.hpp"
#include <algorithm>
#include <unordered_set>
#include <functional>

namespace GridMap {

std::shared_ptr<GridMapData> GridMapProcessor::convertPointCloudToGridMap(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud,
    const GridMapParameters& params) {
    
    if (!cloud || cloud->empty()) {
        return nullptr;
    }
    
    auto gridData = std::make_shared<GridMapData>();
    gridData->resolution = params.resolution;
    gridData->sourceCloudHash = calculateCloudHash(cloud);
    gridData->timestamp = std::chrono::steady_clock::now();
    
    // 1. 높이 필터링 (바닥/천장 제거)
    auto filteredPoints = filterPointsByHeight(cloud, 
                                              params.heightFilterMin, 
                                              params.heightFilterMax);
    
    if (filteredPoints.empty()) {
        return nullptr;
    }
    
    // 2. 바운딩 박스 계산
    cv::Rect2f bbox = calculateBoundingBox(filteredPoints);
    
    // 3. 그리드 맵 크기 설정
    gridData->width = static_cast<int>(std::ceil(bbox.width / params.resolution));
    gridData->height = static_cast<int>(std::ceil(bbox.height / params.resolution));
    gridData->originX = bbox.x;
    gridData->originY = bbox.y;
    
    // 최대 크기 제한
    int maxCells = static_cast<int>(std::max(params.mapSizeX, params.mapSizeY) / params.resolution);
    gridData->width = std::min(gridData->width, maxCells);
    gridData->height = std::min(gridData->height, maxCells);
    
    // 4. 그리드 맵 초기화 (0=Unknown, 255=Occupied)
    gridData->occupancyMap = cv::Mat::zeros(gridData->height, gridData->width, CV_8UC1);
    
    // 5. 포인트들을 그리드에 투영 - Python처럼 단순하게
    for (const auto& point : filteredPoints) {
        // ROS → OpenGL 좌표 변환
        float worldX = -point.y;  // ROS Y → OpenGL X
        float worldZ = -point.x;  // ROS X → OpenGL Z
        
        // 그리드 좌표 계산
        int gridX = static_cast<int>((worldX - gridData->originX) / params.resolution);
        int gridY = static_cast<int>((worldZ - gridData->originY) / params.resolution);
        
        // 경계 검사
        if (gridX >= 0 && gridX < gridData->width && 
            gridY >= 0 && gridY < gridData->height) {
            
            // 단순 점유 표시 (높이 필터링 통과한 모든 포인트)
            gridData->occupancyMap.at<uchar>(gridY, gridX) = 255;  // Occupied
        }
    }
    
    // 6. 모폴로지 연산 (선택적 - 노이즈 제거)
    if (params.enableMorphology) {
        gridData->occupancyMap = applyMorphologyOperations(
            gridData->occupancyMap, params.morphKernelSize);
    }
    
    return gridData;
}

std::vector<pcl::PointXYZI> GridMapProcessor::filterPointsByHeight(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud,
    float minHeight, float maxHeight) {
    
    std::vector<pcl::PointXYZI> filtered;
    filtered.reserve(cloud->size() / 2);  // 대략적인 예상 크기
    
    for (const auto& point : cloud->points) {
        // ROS Z (높이) 체크
        if (point.z >= minHeight && point.z <= maxHeight) {
            filtered.push_back(point);
        }
    }
    
    return filtered;
}

cv::Rect2f GridMapProcessor::calculateBoundingBox(
    const std::vector<pcl::PointXYZI>& points) {
    
    if (points.empty()) return cv::Rect2f();
    
    float minX = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::lowest();
    float minY = std::numeric_limits<float>::max();
    float maxY = std::numeric_limits<float>::lowest();
    
    for (const auto& point : points) {
        // ROS → OpenGL 좌표 변환 적용
        float worldX = -point.y;
        float worldY = -point.x;
        
        minX = std::min(minX, worldX);
        maxX = std::max(maxX, worldX);
        minY = std::min(minY, worldY);
        maxY = std::max(maxY, worldY);
    }
    
    return cv::Rect2f(minX, minY, maxX - minX, maxY - minY);
}

cv::Mat GridMapProcessor::applyMorphologyOperations(
    const cv::Mat& inputMap, int kernelSize) {
    
    cv::Mat result;
    cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(kernelSize, kernelSize));
    
    // 단순한 Closing 연산만 (작은 구멍 채우기)
    cv::morphologyEx(inputMap, result, cv::MORPH_CLOSE, kernel);
    
    return result;
}

size_t GridMapProcessor::calculateCloudHash(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud) {
    
    if (!cloud || cloud->empty()) return 0;
    
    // 간단한 해시 계산 (크기 + 몇 개 포인트의 좌표)
    size_t hash = cloud->size();
    
    // 몇 개 샘플 포인트로 해시 생성
    int step = std::max(1, static_cast<int>(cloud->size() / 100));
    for (size_t i = 0; i < cloud->size(); i += step) {
        const auto& pt = cloud->points[i];
        hash ^= std::hash<float>{}(pt.x) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        hash ^= std::hash<float>{}(pt.y) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        hash ^= std::hash<float>{}(pt.z) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    }
    
    return hash;
}

std::shared_ptr<GridMapData> GridMapProcessor::mergeGridMaps(
    const std::vector<std::shared_ptr<GridMapData>>& maps,
    const GridMapParameters& params) {
    
    if (maps.empty()) return nullptr;
    
    // TODO: 여러 그리드 맵 병합 구현
    // 현재는 첫 번째 맵 반환
    return maps[0];
}

cv::Mat GridMapProcessor::visualizeGridMap(
    const std::shared_ptr<GridMapData>& gridData,
    bool showProbability) {
    
    if (!gridData || !gridData->isValid()) {
        return cv::Mat();
    }
    
    const cv::Mat& occupancyMap = gridData->occupancyMap;
    cv::Mat colorMap = cv::Mat::zeros(occupancyMap.size(), CV_8UC3);
    
    // 단순한 2색 표시
    for (int y = 0; y < occupancyMap.rows; ++y) {
        for (int x = 0; x < occupancyMap.cols; ++x) {
            uchar value = occupancyMap.at<uchar>(y, x);
            cv::Vec3b& pixel = colorMap.at<cv::Vec3b>(y, x);
            
            if (value == 255) {
                // 점유: 빨간색
                pixel = cv::Vec3b(0, 0, 255);  // BGR format
            } else {
                // 미지: 검은색 (자유공간 구분 없음)
                pixel = cv::Vec3b(0, 0, 0);
            }
        }
    }
    
    return colorMap;
}

} // namespace GridMap