#ifndef RENDER_HELPER_HPP
#define RENDER_HELPER_HPP

#include <QHash>
#include <QString>
#include <QPainter>
#include <QPoint>
#include <mutex>
#include <memory>
#include "common_types.hpp"

namespace RenderHelper {

// Types 네임스페이스 사용
using CloudConstPtr = Types::CloudConstPtr;
using PathConstPtr = Types::PathConstPtr;

enum class PositionMarkerType {
    CYLINDER,
    AXES
};

class PointCloudRenderer {
public:
    // Points 렌더링 (Types::ColorRGB 사용)
    static void drawPoints(
        const QHash<QString, CloudConstPtr>& clouds,
        const QString& currentRobot,
        const QHash<QString, Types::ColorRGB>& robotColors,
        float pointSize,
        std::mutex& cloudMutex
    );
    
    // Paths 렌더링 (PathConstPtr 사용)
    static void drawPaths(
        const QHash<QString, PathConstPtr>& paths,
        const QString& currentRobot,
        const QHash<QString, Types::ColorRGB>& robotColors,
        float pathWidth,
        std::mutex& pathMutex
    );
    
    // Position 마커 렌더링
    static void drawPositions(
        const QHash<QString, PathConstPtr>& paths,
        const QString& currentRobot,
        const QHash<QString, Types::ColorRGB>& robotColors,
        PositionMarkerType markerType,
        float radius,
        float height,
        float axesLength,
        float axesRadius,
        std::mutex& pathMutex
    );
    
    // 기타 렌더링 함수들 (Types::Vec3 사용)
    static void drawAxes(const Types::Vec3& origin, float axesLength, float axesRadius);
    static void drawGrid(int cellCount, float cellSize, float lineWidth);
    static void drawCameraIndicator(const Types::Vec3& focusPoint);
    static void drawRobotLabels(QPainter& painter, const QString& currentRobot, 
                               const QHash<QString, Types::ColorRGB>& robotColors);
    
    // 로봇 위치에 이름 표시
    static void drawRobotNamesAtPositions(
        QPainter& painter,
        const QHash<QString, PathConstPtr>& paths,
        const QString& currentRobot,
        const QHash<QString, Types::ColorRGB>& robotColors,
        const Types::Mat4& viewMatrix,
        const Types::Mat4& projectionMatrix,
        int screenWidth,
        int screenHeight,
        float textSize,
        std::mutex& pathMutex
    );

private:
    // Private helper functions
    static void drawCylinderMarker(const Types::Vec3& position, 
                                  const Types::ColorRGB& robotColor, 
                                  float radius, float height);
    static void drawPositionAxes(const Types::Vec3& position, 
                                const Types::Quat& orientation, 
                                const QString& robotName, 
                                float axesLength, float axesRadius);
    static void drawSingleLabel(QPainter& painter, const QString& text, 
                               const QColor& robotColor, const QPoint& position);
    
    // 3D 위치를 화면 좌표로 변환
    static QPoint worldToScreen(
        const Types::Vec3& worldPos,
        const Types::Mat4& viewMatrix,
        const Types::Mat4& projectionMatrix,
        int screenWidth,
        int screenHeight
    );
    
    // 단일 로봇 이름 텍스트 그리기
    static void drawRobotNameText(
        QPainter& painter,
        const QString& robotName,
        const QColor& robotColor,
        const QPoint& screenPos,
        float textSize
    );
};

class GridMapRenderer {
public:
    static void drawBasicGridMaps(
        const QHash<QString, Types::GridMapPtr>& gridMaps,
        const QString& currentRobot,
        const QHash<QString, Types::ColorRGB>& robotColors,
        std::mutex& gridMapMutex
    );

    static void draw2DProjectedPaths(
        const QHash<QString, PathConstPtr>& paths,
        const QString& currentRobot,
        const QHash<QString, Types::ColorRGB>& robotColors,
        std::mutex& pathMutex,
        float pathWidth
    );

private:
    static void drawCombinedGridMap(
        const QHash<QString, Types::GridMapPtr>& gridMaps,
        const QHash<QString, Types::ColorRGB>& robotColors
    );

    static void drawSingleGridMap(
        const Types::GridMapPtr& gridData,
        const Types::ColorRGB& robotColor
    );
    
    static void drawObstaclesOnly(
        const Types::GridMapPtr& gridData,
        const Types::ColorRGB& robotColor
    );
};

} // namespace RenderHelper

#endif // RENDER_HELPER_HPP