#include "render_helper.hpp"
#include "shape_helper.hpp"
#include "grid_map_processor.hpp"
#include <QFontMetrics>
#include <QBrush>
#include <QPen>
#include <QRect>
#include <algorithm>
#include <limits>
#include <cmath>

namespace RenderHelper {

// ============================================================================
// RenderUtils Implementation (공통 유틸리티)
// ============================================================================

QPoint RenderUtils::worldToScreen(
    const Types::Vec3& worldPos,
    const Types::Mat4& viewMatrix,
    const Types::Mat4& projectionMatrix,
    int screenWidth,
    int screenHeight) {
    
    // 월드 좌표를 클립 공간으로 변환
    Types::Vec4 clipPos = projectionMatrix * viewMatrix * Types::Vec4(worldPos, 1.0f);
    
    // 동차 나눗셈
    if (clipPos.w != 0.0f) {
        clipPos.x /= clipPos.w;
        clipPos.y /= clipPos.w;
        clipPos.z /= clipPos.w;
    }
    
    // NDC를 화면 좌표로 변환
    int screenX = static_cast<int>((clipPos.x + 1.0f) * 0.5f * screenWidth);
    int screenY = static_cast<int>((1.0f - clipPos.y) * 0.5f * screenHeight);  // Y축 뒤집기
    
    return QPoint(screenX, screenY);
}

void RenderUtils::drawStraightConnectorLine(
    QPainter& painter,
    const QPoint& objectPos,
    const QPoint& labelPos,
    const QColor& color) {
    
    // 연결선 스타일 설정
    QPen connectorPen(color, 2, Qt::SolidLine);
    connectorPen.setCapStyle(Qt::RoundCap);
    painter.setPen(connectorPen);
    
    // 직선으로 연결
    painter.drawLine(objectPos, labelPos);
    
    // 시작점에 작은 원 (객체/로봇 위치 표시)
    painter.setBrush(QBrush(color));
    painter.setPen(QPen(Qt::white, 1));
    painter.drawEllipse(objectPos, 4, 4);
    
    // 끝점에 작은 화살표 (라벨 방향 표시)
    drawArrowHead(painter, objectPos, labelPos, color, 8);
}

void RenderUtils::drawArrowHead(
    QPainter& painter,
    const QPoint& from,
    const QPoint& to,
    const QColor& color,
    int size) {
    
    // 방향 벡터 계산
    QPoint direction = to - from;
    double length = sqrt(direction.x() * direction.x() + direction.y() * direction.y());
    
    if (length < 0.001) return;  // 너무 짧으면 그리지 않음
    
    // 정규화된 방향 벡터
    double dx = direction.x() / length;
    double dy = direction.y() / length;
    
    // 화살표 머리 점들 계산
    double arrowLength = size;
    double arrowAngle = M_PI / 6;  // 30도
    
    QPoint arrowP1(
        to.x() - arrowLength * (dx * cos(arrowAngle) + dy * sin(arrowAngle)),
        to.y() - arrowLength * (dy * cos(arrowAngle) - dx * sin(arrowAngle))
    );
    
    QPoint arrowP2(
        to.x() - arrowLength * (dx * cos(arrowAngle) - dy * sin(arrowAngle)),
        to.y() - arrowLength * (dy * cos(arrowAngle) + dx * sin(arrowAngle))
    );
    
    // 화살표 머리 그리기
    painter.setBrush(QBrush(color));
    painter.setPen(QPen(color, 1));
    
    QPolygon arrowHead;
    arrowHead << to << arrowP1 << arrowP2;
    painter.drawPolygon(arrowHead);
}

void RenderUtils::drawLabelBackground(
    QPainter& painter,
    const QRect& textRect,
    const QColor& borderColor,
    const QColor& backgroundColor,
    int padding,
    int borderRadius) {
    
    QRect bgRect = textRect.adjusted(-padding, -padding, padding, padding);
    
    // 배경 그리기
    painter.setBrush(QBrush(backgroundColor));
    painter.setPen(QPen(borderColor, 2));
    painter.drawRoundedRect(bgRect, borderRadius, borderRadius);
}

QRect RenderUtils::calculateMultilineTextRect(
    const QString& text,
    const QFont& font,
    const QPoint& centerPos) {
    
    QFontMetrics metrics(font);
    QStringList lines = text.split('\n');
    
    int maxWidth = 0;
    int totalHeight = 0;
    
    for (const QString& line : lines) {
        QRect lineRect = metrics.boundingRect(line);
        maxWidth = std::max(maxWidth, lineRect.width());
        totalHeight += metrics.height();
    }
    
    return QRect(centerPos.x() - maxWidth / 2,
                 centerPos.y() - totalHeight / 2,
                 maxWidth,
                 totalHeight);
}

// ============================================================================
// PointCloudRenderer Implementation (중복 제거된 버전)
// ============================================================================

void PointCloudRenderer::drawPoints(
    const QHash<QString, Types::CloudConstPtr>& clouds,
    const QString& currentRobot,
    const QHash<QString, Types::ColorRGB>& robotColors,
    float pointSize,
    std::mutex& cloudMutex) {
    
    std::lock_guard<std::mutex> lock(cloudMutex);

    glPointSize(pointSize);
    glBegin(GL_POINTS);

    for (auto it = clouds.cbegin(); it != clouds.cend(); ++it) {
        const QString& robotName = it.key();
        
        // 현재 로봇 필터링
        if (currentRobot != "COMBINED" && robotName != currentRobot) {
            continue;
        }
        
        const auto& cloud = it.value();
        if (!cloud || cloud->empty()) continue;
        
        // 로봇 색상 설정
        Types::ColorRGB color = robotColors.value(robotName, Types::ColorRGB(0.0f, 1.0f, 0.0f));
        glColor3f(color.x, color.y, color.z);
        
        // 포인트 렌더링 (ROS → OpenGL 좌표 변환)
        for (const auto& point : cloud->points) {
            glVertex3f(-point.y, point.z, -point.x);
        }
    }
    
    glEnd();
}
void PointCloudRenderer::drawPaths(
    const QHash<QString, PathConstPtr>& paths,
    const QString& currentRobot,
    const QHash<QString, Types::ColorRGB>& robotColors,
    float pathWidth,
    std::mutex& pathMutex) {
    
    std::lock_guard<std::mutex> lock(pathMutex);
    
    glLineWidth(pathWidth);
    glBegin(GL_LINES);
    
    for (auto it = paths.cbegin(); it != paths.cend(); ++it) {
        const QString& robotName = it.key();
        
        // 현재 로봇 필터링
        if (currentRobot != "COMBINED" && robotName != currentRobot) {
            continue;
        }
        
        const auto& pathPtr = it.value();
        if (!pathPtr || pathPtr->empty()) continue;
        
        const auto& path = *pathPtr;  // 포인터 역참조
        
        // 로봇 색상 설정
        Types::ColorRGB color = robotColors.value(robotName, Types::ColorRGB(0.0f, 1.0f, 0.0f));
        glColor3f(color.x, color.y, color.z);
        
        // 경로 라인 렌더링
        for (size_t i = 1; i < path.size(); ++i) {
            const auto& prev_pose = path[i-1];
            const auto& curr_pose = path[i];
            
            // ROS → OpenGL 좌표 변환
            glVertex3f(-prev_pose.pose.position.y, prev_pose.pose.position.z, -prev_pose.pose.position.x);
            glVertex3f(-curr_pose.pose.position.y, curr_pose.pose.position.z, -curr_pose.pose.position.x);
        }
    }
    
    glEnd();
}
void PointCloudRenderer::drawRobotNamesAtPositions(
    QPainter& painter,
    const QHash<QString, Types::PathConstPtr>& paths,
    const QString& currentRobot,
    const QHash<QString, Types::ColorRGB>& robotColors,
    const Types::Mat4& viewMatrix,
    const Types::Mat4& projectionMatrix,
    int screenWidth,
    int screenHeight,
    float textSize,
    std::mutex& pathMutex) {
    
    std::lock_guard<std::mutex> lock(pathMutex);
    
    for (auto it = paths.cbegin(); it != paths.cend(); ++it) {
        const QString& robotName = it.key();
        
        // 현재 로봇 필터링
        if (currentRobot != "COMBINED" && robotName != currentRobot) {
            continue;
        }
        
        const auto& pathPtr = it.value();
        if (!pathPtr || pathPtr->empty()) continue;
        
        const auto& path = *pathPtr;
        const auto& currentPose = path.back();
        
        // 로봇의 실제 위치 (ROS → OpenGL 좌표 변환)
        Types::Vec3 robotWorldPosition(
            -currentPose.pose.position.y,
            currentPose.pose.position.z,
            -currentPose.pose.position.x
        );
        
        // 라벨 위치 (로봇 위에)
        Types::Vec3 labelWorldPosition = robotWorldPosition + Types::Vec3(0.0f, 5.0f, 0.0f);
        
        // RenderUtils 사용 (중복 제거)
        QPoint robotScreenPos = RenderUtils::worldToScreen(robotWorldPosition, viewMatrix, 
                                                         projectionMatrix, screenWidth, screenHeight);
        QPoint labelScreenPos = RenderUtils::worldToScreen(labelWorldPosition, viewMatrix, 
                                                         projectionMatrix, screenWidth, screenHeight);
        
        // 화면 범위 내에 있는지 확인
        if (labelScreenPos.x() >= 0 && labelScreenPos.x() < screenWidth && 
            labelScreenPos.y() >= 0 && labelScreenPos.y() < screenHeight) {
            
            // 로봇 색상 설정
            Types::ColorRGB color = robotColors.value(robotName, Types::ColorRGB(0.0f, 1.0f, 0.0f));
            QColor robotColor(color.x * 255, color.y * 255, color.z * 255);
            
            // RenderUtils 사용 (중복 제거)
            RenderUtils::drawStraightConnectorLine(painter, robotScreenPos, labelScreenPos, robotColor);
            
            // 로봇 이름 텍스트 그리기
            drawRobotNameText(painter, robotName, robotColor, labelScreenPos, textSize);
        }
    }
}

void PointCloudRenderer::drawRobotNameText(
    QPainter& painter,
    const QString& robotName,
    const QColor& robotColor,
    const QPoint& screenPos,
    float textSize) {
    
    // 텍스트 설정
    QFont font("Arial", static_cast<int>(textSize), QFont::Bold);
    painter.setFont(font);
    
    // RenderUtils 사용해서 텍스트 영역 계산
    QRect textRect = RenderUtils::calculateMultilineTextRect(robotName, font, screenPos);
    
    // RenderUtils 사용해서 배경 그리기
    RenderUtils::drawLabelBackground(painter, textRect, robotColor);
    
    // 텍스트 그리기
    painter.setPen(QPen(Qt::white));
    painter.drawText(textRect, Qt::AlignCenter, robotName);
}

// ============================================================================
// Private Helper Functions
// ============================================================================

void PointCloudRenderer::drawCylinderMarker(
    const Types::Vec3& position,
    const Types::ColorRGB& robotColor,
    float radius,
    float height) {
    
    // 메인 실린더
    ShapeHelper::SimpleShape::drawCylinder(
        position + Types::Vec3(0, height/2, 0),
        Types::Vec3(0, 1, 0),
        height,
        radius,
        Types::Vec4(robotColor.x, robotColor.y, robotColor.z, 0.8f)
    );
    
    // 상단 표시등
    ShapeHelper::SimpleShape::drawCylinder(
        position + Types::Vec3(0, height + 0.05f, 0),
        Types::Vec3(0, 1, 0),
        0.05f,
        radius * 0.7f,
        Types::Vec4(1.0f, 1.0f, 1.0f, 1.0f)
    );
}
void PointCloudRenderer::drawAxes(const Types::Vec3& origin, float axesLength, float axesRadius) {
    ShapeHelper::SimpleShape::drawRosAxes(
        origin,
        Types::Quat(1.0f, 0.0f, 0.0f, 0.0f),  // 단위 쿼터니언
        axesLength,
        axesRadius,
        true  // drawArrowheads
    );
}

void PointCloudRenderer::drawGrid(int cellCount, float cellSize, float lineWidth) {
    glLineWidth(lineWidth);
    glColor3f(0.5f, 0.5f, 0.5f);  // 회색
    glBegin(GL_LINES);
    
    float halfSize = (cellCount * cellSize) / 2.0f;
    
    // X 방향 선들
    for (int i = -cellCount/2; i <= cellCount/2; ++i) {
        float x = i * cellSize;
        glVertex3f(x, 0.0f, -halfSize);
        glVertex3f(x, 0.0f, halfSize);
    }
    
    // Z 방향 선들
    for (int i = -cellCount/2; i <= cellCount/2; ++i) {
        float z = i * cellSize;
        glVertex3f(-halfSize, 0.0f, z);
        glVertex3f(halfSize, 0.0f, z);
    }
    
    glEnd();
}

void PointCloudRenderer::drawCameraIndicator(const glm::vec3& focusPoint) {
    glDisable(GL_DEPTH_TEST);
    
    ShapeHelper::SimpleShape::drawCylinder(
        focusPoint,
        glm::vec3(0, 1, 0),
        0.1f,
        0.3f,
        glm::vec4(1.0f, 1.0f, 0.0f, 0.7f)
    );
    
    glEnable(GL_DEPTH_TEST);
}
void PointCloudRenderer::drawPositionAxes(
    const Types::Vec3& position,
    const Types::Quat& orientation,
    const QString& robotName,
    float axesLength,
    float axesRadius) {
    
    // 바닥에서 살짝 위에 시작점 설정
    Types::Vec3 axesStart = position + Types::Vec3(0, axesRadius, 0);
    
    ShapeHelper::SimpleShape::drawRosAxes(
        axesStart,
        orientation,
        axesLength,
        axesRadius,
        true  // drawArrowheads
    );
}

void PointCloudRenderer::drawPositions(
    const QHash<QString, PathConstPtr>& paths,
    const QString& currentRobot,
    const QHash<QString, Types::ColorRGB>& robotColors,
    PositionMarkerType markerType,
    float radius,
    float height,
    float axesLength,
    float axesRadius,
    std::mutex& pathMutex) {
    
    std::lock_guard<std::mutex> lock(pathMutex);
    
    for (auto it = paths.cbegin(); it != paths.cend(); ++it) {
        const QString& robotName = it.key();
        
        // 현재 로봇 필터링
        if (currentRobot != "COMBINED" && robotName != currentRobot) {
            continue;
        }
        
        const auto& pathPtr = it.value();
        if (!pathPtr || pathPtr->empty()) continue;
        
        const auto& path = *pathPtr;
        const auto& currentPose = path.back();
        
        // 위치 및 방향 계산 (ROS → OpenGL 변환)
        Types::Vec3 position(
            -currentPose.pose.position.y,
            currentPose.pose.position.z,
            -currentPose.pose.position.x
        );
        
        auto quat = currentPose.pose.orientation;
        Types::Quat orientation(quat.w, quat.x, quat.y, quat.z);
        
        Types::ColorRGB robotColor = robotColors.value(robotName, Types::ColorRGB(0.0f, 1.0f, 0.0f));
        
        // 마커 타입에 따른 렌더링
        switch (markerType) {
            case PositionMarkerType::CYLINDER:
                drawCylinderMarker(position, robotColor, radius, height);
                break;
                
            case PositionMarkerType::AXES:
                {
                    float calculatedLength = radius * 3.0f;
                    float calculatedRadius = radius * 0.15f;
                    
                    drawPositionAxes(position, orientation, robotName, 
                                   calculatedLength, calculatedRadius);
                }
                break;
        }
    }
}
void PointCloudRenderer::drawRobotLabels(
    QPainter& painter,
    const QString& currentRobot,
    const QHash<QString, glm::vec3>& robotColors) {
    
    if (currentRobot == "COMBINED") {
        QStringList robots = {"TUGV", "MUGV", "SUGV1", "SUGV2", "SUAV"};
        
        for (int i = 0; i < robots.size(); ++i) {
            QString robotName = robots[i];
            glm::vec3 color = robotColors.value(robotName, glm::vec3(0.0f, 1.0f, 0.0f));
            QColor robotColor(color.x * 255, color.y * 255, color.z * 255);
            
            int yOffset = 10 + i * 30;
            drawSingleLabel(painter, robotName, robotColor, QPoint(10, yOffset));  // 이 부분 수정됨
        }
    } else {
        glm::vec3 color = robotColors.value(currentRobot, glm::vec3(0.0f, 1.0f, 0.0f));
        QColor robotColor(color.x * 255, color.y * 255, color.z * 255);
        drawSingleLabel(painter, currentRobot, robotColor, QPoint(10, 10));
    }
}
void PointCloudRenderer::drawSingleLabel(
    QPainter& painter,
    const QString& text,
    const QColor& robotColor,
    const QPoint& position) {
    
    const int fontSize = 9;
    const int circleSize = 12;
    const int circleMargin = 5;
    const int horizontalMargin = 6;
    const int verticalMargin = 4;
    
    painter.setFont(QFont("Arial", fontSize, QFont::Bold));
    QFontMetrics fm(painter.font());
    
    QRect textBounds = fm.boundingRect(text);
    int textWidth = textBounds.width();
    int textHeight = fm.height();
    
    int boxWidth = horizontalMargin + circleSize + circleMargin + textWidth + horizontalMargin;
    int boxHeight = std::max(circleSize + verticalMargin * 2, textHeight + verticalMargin * 2);
    
    QRect labelRect(position.x(), position.y(), boxWidth, boxHeight);
    
    // 배경 박스
    painter.setBrush(QBrush(QColor(0, 0, 0, 100)));
    painter.setPen(Qt::NoPen);
    painter.fillRect(labelRect, QColor(0, 0, 0, 100));
    
    // 테두리
    painter.setBrush(Qt::NoBrush);
    painter.setPen(QPen(Qt::white, 1));
    painter.drawRect(labelRect);
    
    // 로봇 색상 원
    int boxCenterY = labelRect.top() + labelRect.height() / 2;
    painter.setBrush(QBrush(robotColor));
    painter.setPen(Qt::NoPen);
    int circleX = labelRect.left() + horizontalMargin;
    painter.drawEllipse(circleX, boxCenterY - circleSize/2, circleSize, circleSize);
    
    // 텍스트
    painter.setBrush(Qt::NoBrush);
    painter.setPen(Qt::white);
    int textX = circleX + circleSize + circleMargin;
    int textY = boxCenterY + fm.ascent()/2 - fm.descent()/2;
    painter.drawText(textX, textY, text);
}
// ============================================================================
// GridMapRenderer Implementation
// ============================================================================

void GridMapRenderer::draw2DProjectedPaths(
    const QHash<QString, PathConstPtr>& paths,
    const QString& currentRobot,
    const QHash<QString, Types::ColorRGB>& robotColors,
    std::mutex& pathMutex,
    float pathWidth) {
    
    std::lock_guard<std::mutex> lock(pathMutex);
    
    glLineWidth(pathWidth);
    glBegin(GL_LINES);
    
    for (auto it = paths.cbegin(); it != paths.cend(); ++it) {
        const QString& robotName = it.key();
        
        // 현재 로봇 필터링
        if (currentRobot != "COMBINED" && robotName != currentRobot) {
            continue;
        }
        
        const auto& pathPtr = it.value();
        if (!pathPtr || pathPtr->empty()) continue;
        
        const auto& path = *pathPtr;  // 포인터 역참조
        
        // 로봇 색상 설정
        Types::ColorRGB color = robotColors.value(robotName, Types::ColorRGB(0.0f, 1.0f, 0.0f));
        glColor4f(color.x, color.y, color.z, 0.9f);
        
        // 경로 라인 렌더링 (Z좌표 고정으로 2D 평면화)
        for (size_t i = 1; i < path.size(); ++i) {
            const auto& prev_pose = path[i-1];
            const auto& curr_pose = path[i];
            
            // ROS → OpenGL 좌표 변환, Z좌표는 GridMap 위에 고정
            float z = 0.02f;  // GridMap보다 살짝 위
            glVertex3f(-prev_pose.pose.position.y, z, -prev_pose.pose.position.x);
            glVertex3f(-curr_pose.pose.position.y, z, -curr_pose.pose.position.x);
        }
    }
    
    glEnd();
}

void GridMapRenderer::drawCombinedGridMap(
    const QHash<QString, Types::GridMapPtr>& gridMaps,
    const QHash<QString, Types::ColorRGB>& robotColors) {
    
    // 1. 통합 바운딩 박스 계산
    float minX = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::lowest();
    float minY = std::numeric_limits<float>::max();
    float maxY = std::numeric_limits<float>::lowest();
    
    for (auto it = gridMaps.cbegin(); it != gridMaps.cend(); ++it) {
        const auto& gridData = it.value();
        if (!gridData || !gridData->isValid()) continue;  // 직접 접근 (캐스팅 불필요)
        
        float gridMinX = gridData->originX;
        float gridMaxX = gridData->originX + gridData->width * gridData->resolution;
        float gridMinY = gridData->originY;
        float gridMaxY = gridData->originY + gridData->height * gridData->resolution;
        
        minX = std::min(minX, gridMinX);
        maxX = std::max(maxX, gridMaxX);
        minY = std::min(minY, gridMinY);
        maxY = std::max(maxY, gridMaxY);
    }
    
    // 2. 배경 먼저 그리기 (한 번만)
    glColor4f(0.5f, 0.6f, 0.6f, 0.8f);
    glBegin(GL_QUADS);
    glVertex3f(minX, 0.01f, minY);
    glVertex3f(maxX, 0.01f, minY);
    glVertex3f(maxX, 0.01f, maxY);
    glVertex3f(minX, 0.01f, maxY);
    glEnd();
    
    // 3. 각 로봇의 장애물만 그리기
    for (auto it = gridMaps.cbegin(); it != gridMaps.cend(); ++it) {
        const QString& robotName = it.key();
        const auto& gridData = it.value();
        
        if (!gridData) continue;
        
        Types::ColorRGB robotColor = robotColors.value(robotName, Types::ColorRGB(0.0f, 1.0f, 0.0f));
        drawObstaclesOnly(gridData, robotColor);
    }
}

void GridMapRenderer::drawSingleGridMap(
    const Types::GridMapPtr& gridData,
    const Types::ColorRGB& robotColor) {
    
    if (!gridData || !gridData->isValid()) return;  // 직접 접근
    
    const cv::Mat& occupancyMap = gridData->occupancyMap;
    float resolution = gridData->resolution;
    float originX = gridData->originX;
    float originY = gridData->originY;
    
    glBegin(GL_QUADS);
    
    for (int row = 0; row < occupancyMap.rows; ++row) {
        for (int col = 0; col < occupancyMap.cols; ++col) {
            uchar value = occupancyMap.at<uchar>(row, col);
            
            // 색상 설정: 배경(회색) vs 장애물(로봇색상)
            if (value == 255) {
                // 장애물: 로봇 색상 (불투명)
                glColor4f(robotColor.r, robotColor.g, robotColor.b, 0.8f);
            } else {
                // 배경 (Unknown): 회색 (반투명)
                glColor4f(0.5f, 0.6f, 0.6f, 0.8f);
            }
            
            // 셀의 월드 좌표 계산
            float worldX = originX + col * resolution;
            float worldY = originY + row * resolution;
            float z = 0.01f;  // 바닥 살짝 위
            
            // 사각형 그리기
            glVertex3f(worldX, z, worldY);
            glVertex3f(worldX + resolution, z, worldY);
            glVertex3f(worldX + resolution, z, worldY + resolution);
            glVertex3f(worldX, z, worldY + resolution);
        }
    }
    
    glEnd();
}

void GridMapRenderer::drawObstaclesOnly(
    const Types::GridMapPtr& gridData,
    const Types::ColorRGB& robotColor) {
    
    if (!gridData || !gridData->isValid()) return;  // 직접 접근
    
    const cv::Mat& occupancyMap = gridData->occupancyMap;
    float resolution = gridData->resolution;
    float originX = gridData->originX;
    float originY = gridData->originY;
    
    // 장애물만 그리기 (배경 제외)
    glColor4f(robotColor.r, robotColor.g, robotColor.b, 0.7f);
    glBegin(GL_QUADS);
    
    for (int row = 0; row < occupancyMap.rows; ++row) {
        for (int col = 0; col < occupancyMap.cols; ++col) {
            uchar value = occupancyMap.at<uchar>(row, col);
            
            // 장애물인 셀만 렌더링
            if (value == 255) {
                float worldX = originX + col * resolution;
                float worldY = originY + row * resolution;
                float z = 0.02f;  // 배경보다 살짝 위
                
                glVertex3f(worldX, z, worldY);
                glVertex3f(worldX + resolution, z, worldY);
                glVertex3f(worldX + resolution, z, worldY + resolution);
                glVertex3f(worldX, z, worldY + resolution);
            }
        }
    }
    
    glEnd();
}
void GridMapRenderer::drawBasicGridMaps(
    const QHash<QString, Types::GridMapPtr>& gridMaps,
    const QString& currentRobot,
    const QHash<QString, Types::ColorRGB>& robotColors,
    std::mutex& gridMapMutex) {
    
    std::lock_guard<std::mutex> lock(gridMapMutex);
    
    if (gridMaps.isEmpty()) return;
    
    // 단일 로봇 모드
    if (currentRobot != "COMBINED") {
        auto it = gridMaps.find(currentRobot);
        if (it != gridMaps.end() && it.value()) {
            Types::ColorRGB robotColor = robotColors.value(currentRobot, Types::ColorRGB(0.0f, 1.0f, 0.0f));
            drawSingleGridMap(it.value(), robotColor);
        }
    } else {
        // 결합 모드
        drawCombinedGridMap(gridMaps, robotColors);
    }
}
// ============================================================================
// InterestObjectRenderer Implementation (중복 제거된 버전)
// ============================================================================

void InterestObjectRenderer::drawInterestObjectLabels(
    QPainter& painter,
    const QHash<QString, Types::InterestObjectPtr>& allObjects,
    const QString& currentRobot,
    const Types::Mat4& viewMatrix,
    const Types::Mat4& projectionMatrix,
    int screenWidth,
    int screenHeight,
    float textSize) {
    
    if (allObjects.isEmpty()) return;
    
    for (auto it = allObjects.cbegin(); it != allObjects.cend(); ++it) {
        const auto& obj = it.value();
        if (!obj || !obj->isActive) continue;
        
        // 로봇 필터링
        if (currentRobot != "COMBINED" && obj->discoveredBy != currentRobot) {
            continue;
        }
        
        // 객체의 실제 위치
        Types::Vec3 objectWorldPosition = obj->position;
        
        // 라벨 위치 (객체 위에)
        Types::Vec3 labelWorldPosition = obj->position + Types::Vec3(0.0f, 3.0f, 0.0f);
        
        // RenderUtils 사용 (중복 제거)
        QPoint objectScreenPos = RenderUtils::worldToScreen(objectWorldPosition, viewMatrix, 
                                                          projectionMatrix, screenWidth, screenHeight);
        QPoint labelScreenPos = RenderUtils::worldToScreen(labelWorldPosition, viewMatrix, 
                                                         projectionMatrix, screenWidth, screenHeight);
        
        // 화면 범위 내에 있는지 확인
        if (labelScreenPos.x() >= 0 && labelScreenPos.x() < screenWidth && 
            labelScreenPos.y() >= 0 && labelScreenPos.y() < screenHeight) {
            
            // 새로운 라벨 형식: #obj_id[obj_class]
            QString labelText;
            
            if (currentRobot == "COMBINED") {
                // COMBINED 모드일 때: #obj_id[obj_class]\n{robot}
                labelText = QString("#%1[%2]{%3}")
                    .arg(obj->id)
                    .arg(obj->type)  // 문자열 직접 사용
                    .arg(obj->discoveredBy);
            } else {
                // 단일 로봇 모드일 때: #obj_id[obj_class]
                labelText = QString("#%1[%2]")
                    .arg(obj->id)
                    .arg(obj->type);  // 문자열 직접 사용
            }
            
            // 객체 색상 사용
            QColor labelColor(obj->color.x * 255, obj->color.y * 255, obj->color.z * 255);
            
            // RenderUtils 사용 (중복 제거)
            RenderUtils::drawStraightConnectorLine(painter, objectScreenPos, labelScreenPos, labelColor);
            
            // 라벨 그리기
            drawObjectLabel(painter, labelText, labelColor, labelScreenPos, textSize);
        }
    }
}

void InterestObjectRenderer::drawObjectLabel(
    QPainter& painter,
    const QString& text,
    const QColor& color,
    const QPoint& screenPos,
    float textSize) {
    
    // 텍스트 설정 - 더 읽기 쉽게 조정
    QFont font("Consolas", static_cast<int>(textSize + 2), QFont::Bold);
    painter.setFont(font);
    
    // RenderUtils 사용해서 텍스트 영역 계산
    QRect textRect = RenderUtils::calculateMultilineTextRect(text, font, screenPos);
    
    // RenderUtils 사용해서 배경 그리기
    RenderUtils::drawLabelBackground(painter, textRect, color);
    
    // 멀티라인 텍스트 그리기
    QStringList lines = text.split('\n');
    QFontMetrics metrics(font);
    
    int yOffset = textRect.top() + metrics.ascent();
    
    painter.setPen(QPen(Qt::white));
    for (const QString& line : lines) {
        QRect lineRect = metrics.boundingRect(line);
        int xPos = textRect.center().x() - lineRect.width() / 2;
        painter.drawText(xPos, yOffset, line);
        yOffset += metrics.height();
    }
}

void InterestObjectRenderer::drawInterestObjects(
    const QHash<QString, Types::InterestObjectPtr>& allObjects,
    const QString& currentRobot) {
    
    if (allObjects.isEmpty()) return;
    
    // OpenGL 상태 설정
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_DEPTH_TEST);
    
    for (auto it = allObjects.cbegin(); it != allObjects.cend(); ++it) {
        const auto& obj = it.value();
        if (!obj || !obj->isActive) continue;
        
        // 로봇 필터링: COMBINED 모드가 아니면 해당 로봇이 발견한 객체만 표시
        if (currentRobot != "COMBINED" && obj->discoveredBy != currentRobot) {
            continue;
        }
        
        // Interest Object 렌더링
        drawSingleInterestObject(obj, true);
    }
    
    glDisable(GL_BLEND);
}

void InterestObjectRenderer::drawSingleInterestObject(
    const Types::InterestObjectPtr& obj,
    bool drawWireframe) {
    
    if (!obj) return;
    
    // 타입에 따른 색상 조정
    Types::ColorRGB renderColor = obj->color;
    float alpha = 0.8f;

    Types::Vec4 renderColorAlpha(
        renderColor.x, renderColor.y, renderColor.z, alpha
    );
    // 정육면체 렌더링 (ShapeHelper 사용)
    ShapeHelper::SimpleShape::drawCube(obj->position, obj->size, renderColorAlpha);
}

// 나머지 함수들은 기존과 동일하되, RenderUtils 사용하도록 수정...

} // namespace RenderHelper
