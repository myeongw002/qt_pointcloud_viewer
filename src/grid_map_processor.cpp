// grid_map_processor.cpp
#include "grid_map_processor.hpp"
#include <algorithm>
#include <unordered_set>
#include <functional>
#include <QDebug>

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

// 새로 추가: pointcloud_widget에서 사용할 함수
bool GridMapProcessor::processPointCloud(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud,
    const GridMapParameters& params,
    GridMapData& gridData) {
    
    if (!cloud || cloud->empty()) {
        qDebug() << "GridMapProcessor::processPointCloud: Empty cloud provided";
        return false;
    }
    
    // 기존 convertPointCloudToGridMap 함수 활용
    auto processedGridData = convertPointCloudToGridMap(cloud, params);
    
    if (!processedGridData || !processedGridData->isValid()) {
        qDebug() << "GridMapProcessor::processPointCloud: Failed to convert point cloud";
        return false;
    }
    
    // 결과를 참조로 전달된 gridData에 복사
    gridData.width = processedGridData->width;
    gridData.height = processedGridData->height;
    gridData.resolution = processedGridData->resolution;
    gridData.originX = processedGridData->originX;
    gridData.originY = processedGridData->originY;
    gridData.occupancyMap = processedGridData->occupancyMap.clone();  // OpenCV Mat 복사
    gridData.sourceCloudHash = processedGridData->sourceCloudHash;
    gridData.timestamp = processedGridData->timestamp;
    
    // data 벡터 형태로도 변환 (렌더링용)
    convertMatToVector(gridData);
    
    // qDebug() << "GridMapProcessor::processPointCloud: Successfully processed" 
    //          << cloud->size() << "points into" 
    //          << gridData.width << "x" << gridData.height << "grid";
    
    return true;
}

// OpenCV Mat을 std::vector<float>로 변환하는 헬퍼 함수
void GridMapProcessor::convertMatToVector(GridMapData& gridData) {
    if (gridData.occupancyMap.empty()) {
        gridData.data.clear();
        return;
    }
    
    const cv::Mat& occupancyMap = gridData.occupancyMap;
    int totalCells = gridData.width * gridData.height;
    gridData.data.resize(totalCells);
    
    // OpenCV Mat (uchar) → std::vector<float> 변환
    for (int y = 0; y < gridData.height; ++y) {
        for (int x = 0; x < gridData.width; ++x) {
            int vectorIndex = y * gridData.width + x;
            uchar occupancyValue = occupancyMap.at<uchar>(y, x);
            
            // uchar (0-255) → float (0.0-1.0) 변환
            if (occupancyValue == 255) {
                gridData.data[vectorIndex] = 1.0f;  // Occupied
            } else if (occupancyValue == 0) {
                gridData.data[vectorIndex] = -1.0f; // Unknown
            } else {
                gridData.data[vectorIndex] = occupancyValue / 255.0f;  // Probability
            }
        }
    }
}

} // namespace GridMap

namespace GridMap {

// Path를 GridMap 좌표계로 프로젝션
ProjectedPath GridMapProcessor::projectPathToGridMap(
    const std::vector<geometry_msgs::msg::PoseStamped>& path,
    const GridMapData& gridData,
    const QString& robotName,
    const glm::vec3& color) {
    
    ProjectedPath projectedPath;
    projectedPath.robotName = robotName;
    projectedPath.color = color;
    
    if (path.empty() || !gridData.isValid()) {
        qDebug() << "GridMapProcessor: Invalid path or grid data for projection";
        return projectedPath;
    }
    
    projectedPath.points.reserve(path.size());
    projectedPath.orientations.reserve(path.size());
    
    for (const auto& poseStamped : path) {
        const auto& pose = poseStamped.pose;
        
        // 3D 위치를 2D 그리드 좌표로 변환
        cv::Point2f gridPoint = worldToGrid(
            pose.position.x, 
            pose.position.y, 
            gridData
        );
        
        // 그리드 범위 체크
        if (gridPoint.x >= 0 && gridPoint.x < gridData.width &&
            gridPoint.y >= 0 && gridPoint.y < gridData.height) {
            
            projectedPath.points.push_back(gridPoint);
            
            // 방향각 계산 (쿼터니언 → 오일러)
            float orientation = calculateOrientation(poseStamped);
            projectedPath.orientations.push_back(orientation);
        }
    }
    
    projectedPath.isValid = !projectedPath.points.empty();
    
    qDebug() << "GridMapProcessor: Projected" << projectedPath.points.size() 
             << "/" << path.size() << "path points for robot:" << robotName;
    
    return projectedPath;
}

// 그리드맵에 경로 그리기
cv::Mat GridMapProcessor::drawPathOnGridMap(
    const cv::Mat& gridMap,
    const ProjectedPath& projectedPath,
    float pathWidth) {
    
    if (!projectedPath.isValid || gridMap.empty()) {
        return gridMap.clone();
    }
    
    cv::Mat result = gridMap.clone();
    
    // 색상 변환 (glm::vec3 → cv::Scalar)
    cv::Scalar pathColor(
        projectedPath.color.b * 255,  // OpenCV는 BGR 순서
        projectedPath.color.g * 255,
        projectedPath.color.r * 255
    );
    
    // 경로 선 그리기
    for (size_t i = 1; i < projectedPath.points.size(); ++i) {
        cv::line(result, 
                projectedPath.points[i-1], 
                projectedPath.points[i], 
                pathColor, 
                static_cast<int>(pathWidth));
    }
    
    // 방향 화살표 그리기 (일정 간격마다)
    int arrowInterval = std::max(1, static_cast<int>(projectedPath.points.size() / 10));
    for (size_t i = 0; i < projectedPath.points.size(); i += arrowInterval) {
        drawArrow(result, 
                 projectedPath.points[i], 
                 projectedPath.orientations[i], 
                 pathColor, 
                 pathWidth * 2.0f);
    }
    
    return result;
}

// 여러 경로를 그리드맵에 합성
cv::Mat GridMapProcessor::combineGridMapWithPaths(
    const cv::Mat& gridMap,
    const std::vector<ProjectedPath>& paths) {
    
    cv::Mat result = gridMap.clone();
    
    // 각 경로를 순차적으로 그리기
    for (const auto& path : paths) {
        if (path.isValid) {
            result = drawPathOnGridMap(result, path, 3.0f);
        }
    }
    
    return result;
}

// 헬퍼 함수들
cv::Point2f GridMapProcessor::worldToGrid(
    float worldX, float worldY,
    const GridMapData& gridData) {
    
    float gridX = (worldX - gridData.originX) / gridData.resolution;
    float gridY = (worldY - gridData.originY) / gridData.resolution;
    
    return cv::Point2f(gridX, gridY);
}

float GridMapProcessor::calculateOrientation(
    const geometry_msgs::msg::PoseStamped& poseStamped) {
    
    const auto& q = poseStamped.pose.orientation;
    
    // 쿼터니언을 오일러 각(yaw)으로 변환
    float yaw = std::atan2(
        2.0f * (q.w * q.z + q.x * q.y),
        1.0f - 2.0f * (q.y * q.y + q.z * q.z)
    );
    
    return yaw;
}

void GridMapProcessor::drawArrow(
    cv::Mat& image,
    const cv::Point2f& point,
    float orientation,
    const cv::Scalar& color,
    float size) {
    
    // 화살표 끝점 계산
    cv::Point2f arrowEnd(
        point.x + size * std::cos(orientation),
        point.y + size * std::sin(orientation)
    );
    
    // 화살표 날개 계산
    float arrowAngle = M_PI / 6;  // 30도
    cv::Point2f wing1(
        point.x + (size * 0.7f) * std::cos(orientation + arrowAngle),
        point.y + (size * 0.7f) * std::sin(orientation + arrowAngle)
    );
    cv::Point2f wing2(
        point.x + (size * 0.7f) * std::cos(orientation - arrowAngle),
        point.y + (size * 0.7f) * std::sin(orientation - arrowAngle)
    );
    
    // 화살표 그리기
    cv::line(image, point, arrowEnd, color, 2);
    cv::line(image, arrowEnd, wing1, color, 2);
    cv::line(image, arrowEnd, wing2, color, 2);
}

} // namespace GridMap