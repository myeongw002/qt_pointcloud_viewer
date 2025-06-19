#ifndef RENDER_HELPER_HPP
#define RENDER_HELPER_HPP

#include <GL/gl.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <QHash>
#include <QString>
#include <QPainter>
#include <QFont>
#include <QColor>
#include <memory>
#include <mutex>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "grid_map_processor.hpp"

namespace RenderHelper {

using Cloud = pcl::PointCloud<pcl::PointXYZI>;
using CloudConstPtr = Cloud::ConstPtr;
using PathConstPtr = std::vector<geometry_msgs::msg::PoseStamped>;

enum class PositionMarkerType {
    CYLINDER = 0,
    AXES = 1
};

class PointCloudRenderer {
public:
    // 포인트클라우드 렌더링
    static void drawPoints(
        const QHash<QString, CloudConstPtr>& clouds,
        const QString& currentRobot,
        const QHash<QString, glm::vec3>& robotColors,
        float pointSize,
        std::mutex& cloudMutex
    );
    
    // 경로 렌더링
    static void drawPaths(
        const QHash<QString, PathConstPtr>& paths,
        const QString& currentRobot,
        const QHash<QString, glm::vec3>& robotColors,
        float pathWidth,
        std::mutex& pathMutex
    );
    
    // 현재 위치 마커 렌더링
    static void drawPositions(
        const QHash<QString, PathConstPtr>& paths,
        const QString& currentRobot,
        const QHash<QString, glm::vec3>& robotColors,
        PositionMarkerType markerType,
        float radius,
        float height,
        float axesLength,
        float axesRadius,
        std::mutex& pathMutex
    );
    
    // 기본 도형 렌더링
    static void drawAxes(
        const glm::vec3& origin,
        float axesLength,
        float axesRadius
    );
    
    static void drawGrid(
        int cellCount,
        float cellSize,
        float lineWidth
    );
    
    static void drawCameraIndicator(const glm::vec3& focusPoint);
    
    // UI 라벨 렌더링
    static void drawRobotLabels(
        QPainter& painter,
        const QString& currentRobot,
        const QHash<QString, glm::vec3>& robotColors
    );

private:
    // 헬퍼 함수들
    static void drawCylinderMarker(
        const glm::vec3& position,
        const glm::vec3& robotColor,
        float radius,
        float height
    );
    
    static void drawPositionAxes(
        const glm::vec3& position,
        const glm::quat& orientation,
        const QString& robotName,
        float axesLength,
        float axesRadius
    );
    
    static void drawSingleLabel(
        QPainter& painter,
        const QString& text,
        const QColor& color,
        const QPoint& position
    );
};

class GridMapRenderer {
public:
    // 그리드맵 렌더링
    static void drawGridMaps(
        const QHash<QString, std::shared_ptr<GridMap::GridMapData>>& gridMaps,
        const QString& currentRobot,
        const QHash<QString, glm::vec3>& robotColors,
        std::mutex& gridMapMutex
    );
    
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