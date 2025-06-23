#pragma once

#include "common_types.hpp"
#include <QHash>
#include <QPainter>
#include <QPoint>
#include <QColor>
#include <QString>
#include <QFont>
#include <mutex>
#include <GL/gl.h>

namespace RenderHelper {

// Types 네임스페이스 사용
using CloudConstPtr = Types::CloudConstPtr;
using PathConstPtr = Types::PathConstPtr;

enum class PositionMarkerType {
    CYLINDER,
    AXES
};

// ============================================================================
// 공통 렌더링 유틸리티 클래스 (중복 코드 제거)
// ============================================================================
class RenderUtils {
public:
    // 3D 월드 좌표를 2D 화면 좌표로 변환
    static QPoint worldToScreen(
        const Types::Vec3& worldPos,
        const Types::Mat4& viewMatrix,
        const Types::Mat4& projectionMatrix,
        int screenWidth,
        int screenHeight);
    
    // 직선 연결선 그리기 (객체/로봇 위치 → 라벨)
    static void drawStraightConnectorLine(
        QPainter& painter,
        const QPoint& objectPos,
        const QPoint& labelPos,
        const QColor& color);
    
    // 화살표 머리 그리기
    static void drawArrowHead(
        QPainter& painter,
        const QPoint& from,
        const QPoint& to,
        const QColor& color,
        int size = 8);
    
    // 텍스트 라벨 배경 박스 그리기
    static void drawLabelBackground(
        QPainter& painter,
        const QRect& textRect,
        const QColor& borderColor,
        const QColor& backgroundColor = QColor(0, 0, 0, 180),
        int padding = 5,
        int borderRadius = 6);
    
    // 멀티라인 텍스트 크기 계산
    static QRect calculateMultilineTextRect(
        const QString& text,
        const QFont& font,
        const QPoint& centerPos);
};

// ============================================================================
// PointCloudRenderer 클래스
// ============================================================================
class PointCloudRenderer {
public:

    // 포인트클라우드 렌더링
    static void drawPoints(
        const QHash<QString, Types::CloudConstPtr>& clouds,
        const QString& currentRobot,
        const QHash<QString, Types::ColorRGB>& robotColors,
        float pointSize,
        std::mutex& cloudMutex);

    // 경로 렌더링
    static void drawPaths(
        const QHash<QString, Types::PathConstPtr>& paths,
        const QString& currentRobot,
        const QHash<QString, Types::ColorRGB>& robotColors,
        float pathWidth,
        std::mutex& pathMutex);

    // 로봇 위치 마커 렌더링
    static void drawPositions(
        const QHash<QString, Types::PathConstPtr>& paths,
        const QString& currentRobot,
        const QHash<QString, Types::ColorRGB>& robotColors,
        PositionMarkerType markerType,
        float radius,
        float height,
        float axesLength,
        float axesRadius,
        std::mutex& pathMutex);

    // 로봇 이름 라벨 렌더링 (2D 오버레이)
    static void drawRobotNamesAtPositions(
        QPainter& painter,
        const QHash<QString, Types::PathConstPtr>& paths,
        const QString& currentRobot,
        const QHash<QString, Types::ColorRGB>& robotColors,
        const Types::Mat4& viewMatrix,
        const Types::Mat4& projectionMatrix,
        int screenWidth,
        int screenHeight,
        float textSize,
        std::mutex& pathMutex);

    // 기본 요소들
    static void drawAxes(const Types::Vec3& origin, float axesLength, float axesRadius);
    static void drawGrid(int cellCount, float cellSize, float lineWidth);
    static void drawCameraIndicator(const Types::Vec3& focusPoint);

    // 로봇 라벨 (화면 상단)
    static void drawRobotLabels(
        QPainter& painter,
        const QString& currentRobot,
        const QHash<QString, Types::ColorRGB>& robotColors);

private:
    // 헬퍼 함수들
    static void drawCylinderMarker(
        const Types::Vec3& position,
        const Types::ColorRGB& robotColor,
        float radius,
        float height);
    
    static void drawPositionAxes(
        const Types::Vec3& position,
        const Types::Quat& orientation,
        const QString& robotName,
        float axesLength,
        float axesRadius);
    
    static void drawSingleLabel(
        QPainter& painter,
        const QString& text,
        const QColor& robotColor,
        const QPoint& position);
    
    static void drawRobotNameText(
        QPainter& painter,
        const QString& robotName,
        const QColor& robotColor,
        const QPoint& screenPos,
        float textSize);
};

// ============================================================================
// GridMapRenderer 클래스
// ============================================================================
class GridMapRenderer {
public:
    // 기본 그리드맵 렌더링
    static void drawBasicGridMaps(
        const QHash<QString, Types::GridMapPtr>& gridMaps,
        const QString& currentRobot,
        const QHash<QString, Types::ColorRGB>& robotColors,
        std::mutex& gridMapMutex);

    // 2D 투영 경로 렌더링 (그리드맵 위)
    static void draw2DProjectedPaths(
        const QHash<QString, Types::PathConstPtr>& paths,
        const QString& currentRobot,
        const QHash<QString, Types::ColorRGB>& robotColors,
        std::mutex& pathMutex,
        float pathWidth = 2.0f);

    // 통합 그리드맵 렌더링
    static void drawCombinedGridMap(
        const QHash<QString, Types::GridMapPtr>& gridMaps,
        const QHash<QString, Types::ColorRGB>& robotColors);

    // 단일 그리드맵 렌더링
    static void drawSingleGridMap(
        const Types::GridMapPtr& gridData,
        const Types::ColorRGB& robotColor);

private:
    // 장애물만 렌더링 (배경 제외)
    static void drawObstaclesOnly(
        const Types::GridMapPtr& gridData,
        const Types::ColorRGB& robotColor);
};

// ============================================================================
// InterestObjectRenderer 클래스
// ============================================================================
class InterestObjectRenderer {
public:
    // 관심객체 3D 렌더링
    static void drawInterestObjects(
        const QHash<QString, Types::InterestObjectPtr>& allObjects,
        const QString& currentRobot);

    // 관심객체 라벨 렌더링 (2D 오버레이)
    static void drawInterestObjectLabels(
        QPainter& painter,
        const QHash<QString, Types::InterestObjectPtr>& allObjects,
        const QString& currentRobot,
        const Types::Mat4& viewMatrix,
        const Types::Mat4& projectionMatrix,
        int screenWidth,
        int screenHeight,
        float textSize);

private:
    // 단일 관심객체 렌더링
    static void drawSingleInterestObject(
        const Types::InterestObjectPtr& obj,
        bool drawWireframe = true);
    
    // 객체 라벨 그리기
    static void drawObjectLabel(
        QPainter& painter,
        const QString& text,
        const QColor& color,
        const QPoint& screenPos,
        float textSize);
};

} // namespace RenderHelper