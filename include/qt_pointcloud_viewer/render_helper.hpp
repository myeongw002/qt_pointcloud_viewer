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
        std::mutex& cloudMutex);
    
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

// ============================================================================
// InterestObjectRenderer Class (새로 추가)
// ============================================================================
class InterestObjectRenderer {
public:
    // Interest Object 렌더링 (로봇 필터링 포함)
    static void drawInterestObjects(
        const QHash<QString, Types::InterestObjectPtr>& allObjects,
        const QString& currentRobot);
        
    // 개별 Interest Object 렌더링
    static void drawSingleInterestObject(
        const Types::InterestObjectPtr& obj,
        bool drawWireframe = true);
        
    // Interest Object 라벨 렌더링
    static void drawInterestObjectLabels(
        QPainter& painter,
        const QHash<QString, Types::InterestObjectPtr>& allObjects,
        const QString& currentRobot,
        const Types::Mat4& viewMatrix,
        const Types::Mat4& projectionMatrix,
        int screenWidth,
        int screenHeight,
        float textSize = 10.0f);

private:
    // 정육면체 렌더링 (ShapeHelper 사용)
    static void drawCubeShape(
        const Types::Vec3& position,
        float size,
        const Types::ColorRGB& color,
        bool drawWireframe);
        
    // 화면 좌표 변환
    static QPoint worldToScreen(
        const Types::Vec3& worldPos,
        const Types::Mat4& viewMatrix,
        const Types::Mat4& projectionMatrix,
        int screenWidth,
        int screenHeight);
        
    // 라벨 텍스트 그리기
    static void drawObjectLabel(
        QPainter& painter,
        const QString& text,
        const QColor& color,
        const QPoint& screenPos,
        float textSize);
    
    // 새로 추가: 서비스 ID 추출 헬퍼
    static QString extractServiceId(const QString& objectId);
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