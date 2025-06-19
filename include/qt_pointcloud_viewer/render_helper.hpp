#ifndef RENDER_HELPER_HPP
#define RENDER_HELPER_HPP

#include <QHash>
#include <QString>
#include <QPainter>
#include <QPoint>
#include <glm/glm.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mutex>
#include <memory>
#include "grid_map_processor.hpp"


namespace GridMap {
    struct GridMapData;
}

namespace RenderHelper {

// 타입 정의 통일
using CloudConstPtr = pcl::PointCloud<pcl::PointXYZI>::ConstPtr;
using PathVector = std::vector<geometry_msgs::msg::PoseStamped>;  // 직접 벡터 타입

enum class PositionMarkerType {
    CYLINDER,
    AXES
};

class PointCloudRenderer {
public:
    // Points 렌더링
    static void drawPoints(
        const QHash<QString, CloudConstPtr>& clouds,
        const QString& currentRobot,
        const QHash<QString, glm::vec3>& robotColors,
        float pointSize,
        std::mutex& cloudMutex
    );
    
    // Paths 렌더링 (타입 수정)
    static void drawPaths(
        const QHash<QString, PathVector>& paths,  // PathConstPtr → PathVector
        const QString& currentRobot,
        const QHash<QString, glm::vec3>& robotColors,
        float pathWidth,
        std::mutex& pathMutex
    );
    
    // Position 마커 렌더링 (타입 수정)
    static void drawPositions(
        const QHash<QString, PathVector>& paths,  // PathConstPtr → PathVector
        const QString& currentRobot,
        const QHash<QString, glm::vec3>& robotColors,
        PositionMarkerType markerType,
        float radius,
        float height,
        float axesLength,
        float axesRadius,
        std::mutex& pathMutex
    );
    
    // 기타 렌더링 함수들
    static void drawAxes(const glm::vec3& origin, float axesLength, float axesRadius);
    static void drawGrid(int cellCount, float cellSize, float lineWidth);
    static void drawCameraIndicator(const glm::vec3& focusPoint);
    static void drawRobotLabels(QPainter& painter, const QString& currentRobot, const QHash<QString, glm::vec3>& robotColors);

private:
    // Private helper functions
    static void drawCylinderMarker(const glm::vec3& position, const glm::vec3& robotColor, float radius, float height);
    static void drawPositionAxes(const glm::vec3& position, const glm::quat& orientation, const QString& robotName, float axesLength, float axesRadius);
    static void drawSingleLabel(QPainter& painter, const QString& text, const QColor& robotColor, const QPoint& position);
};

class GridMapRenderer {
public:
    static void drawBasicGridMaps(
        const QHash<QString, std::shared_ptr<GridMap::GridMapData>>& gridMaps,
        const QString& currentRobot,
        const QHash<QString, glm::vec3>& robotColors,
        std::mutex& gridMapMutex
    );

    static void draw2DProjectedPaths(
        const QHash<QString, PathVector>& paths,  // PathConstPtr → PathVector
        const QString& currentRobot,
        const QHash<QString, glm::vec3>& robotColors,
        std::mutex& pathMutex,
        float pathWidth
    );

private:
    // 헬퍼 함수들 (타입 수정)
    
    static void drawCombinedGridMap(
        const QHash<QString, std::shared_ptr<GridMap::GridMapData>>& gridMaps,
        const QHash<QString, glm::vec3>& robotColors
    );

    static void drawSingleGridMap(
        const std::shared_ptr<GridMap::GridMapData>& gridData,
        const glm::vec3& robotColor
    );
    
    static void drawObstaclesOnly(
        const std::shared_ptr<GridMap::GridMapData>& gridData,
        const glm::vec3& robotColor
    );

};

} // namespace RenderHelper

#endif // RENDER_HELPER_HPP