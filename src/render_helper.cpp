#include "render_helper.hpp"
#include "shape_helper.hpp"
#include "grid_map_processor.hpp"  // GridMap 기능 사용
#include <QFontMetrics>
#include <QBrush>
#include <QPen>
#include <QRect>
#include <algorithm>
#include <limits>

namespace RenderHelper {

// ============================================================================
// PointCloudRenderer Implementation
// ============================================================================

void PointCloudRenderer::drawPoints(
    const QHash<QString, CloudConstPtr>& clouds,
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

void PointCloudRenderer::drawRobotNamesAtPositions(
    QPainter& painter,
    const QHash<QString, PathConstPtr>& paths,
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
        
        // ROS → OpenGL 좌표 변환
        Types::Vec3 worldPosition(
            -currentPose.pose.position.y,
            currentPose.pose.position.z + 2.0f,
            -currentPose.pose.position.x
        );
        
        // 3D 위치를 화면 좌표로 변환
        QPoint screenPos = worldToScreen(worldPosition, viewMatrix, projectionMatrix, 
                                       screenWidth, screenHeight);
        
        // 화면 범위 내에 있는지 확인
        if (screenPos.x() >= 0 && screenPos.x() < screenWidth && 
            screenPos.y() >= 0 && screenPos.y() < screenHeight) {
            
            // 로봇 색상 설정
            Types::ColorRGB color = robotColors.value(robotName, Types::ColorRGB(0.0f, 1.0f, 0.0f));
            QColor robotColor(color.x * 255, color.y * 255, color.z * 255);
            
            // 로봇 이름 텍스트 그리기
            drawRobotNameText(painter, robotName, robotColor, screenPos, textSize);
        }
    }
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

// ============================================================================
// PointCloudRenderer 누락된 함수들 구현
// ============================================================================

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

void PointCloudRenderer::drawRobotLabels(
    QPainter& painter, 
    const QString& currentRobot, 
    const QHash<QString, Types::ColorRGB>& robotColors) {
    
    // 좌상단에 로봇 목록과 색상 표시
    const int margin = 10;
    const int circleSize = 12;
    const int textHeight = 16;
    int yOffset = margin;
    
    painter.setFont(QFont("Arial", 10));
    
    // 현재 로봇 먼저 표시
    if (!currentRobot.isEmpty() && currentRobot != "COMBINED") {
        Types::ColorRGB color = robotColors.value(currentRobot, Types::ColorRGB(1.0f, 1.0f, 1.0f));
        QColor qcolor(color.x * 255, color.y * 255, color.z * 255);
        
        // 색상 원
        painter.setBrush(QBrush(qcolor));
        painter.setPen(QPen(Qt::white, 2));
        painter.drawEllipse(margin, yOffset, circleSize, circleSize);
        
        // 로봇 이름 (굵게)
        painter.setPen(QPen(Qt::white));
        painter.setFont(QFont("Arial", 10, QFont::Bold));
        painter.drawText(margin + circleSize + 8, yOffset + circleSize, 
                        currentRobot + " (Current)");
        
        yOffset += textHeight + 5;
    }
    
    // 다른 로봇들 표시
    painter.setFont(QFont("Arial", 9));
    for (auto it = robotColors.cbegin(); it != robotColors.cend(); ++it) {
        const QString& robotName = it.key();
        if (robotName == currentRobot || robotName == "DEFAULT") continue;
        
        Types::ColorRGB color = it.value();
        QColor qcolor(color.x * 255, color.y * 255, color.z * 255);
        
        // 색상 원
        painter.setBrush(QBrush(qcolor));
        painter.setPen(QPen(Qt::white, 1));
        painter.drawEllipse(margin, yOffset, circleSize, circleSize);
        
        // 로봇 이름
        painter.setPen(QPen(Qt::lightGray));
        painter.drawText(margin + circleSize + 8, yOffset + circleSize, robotName);
        
        yOffset += textHeight;
    }
}

QPoint PointCloudRenderer::worldToScreen(
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

void PointCloudRenderer::drawRobotNameText(
    QPainter& painter,
    const QString& robotName,
    const QColor& robotColor,
    const QPoint& screenPos,
    float textSize) {
    
    // 텍스트 설정
    QFont font("Arial", static_cast<int>(textSize), QFont::Bold);
    painter.setFont(font);
    
    // 텍스트 크기 측정
    QFontMetrics metrics(font);
    QRect textRect = metrics.boundingRect(robotName);
    
    // 배경 박스 그리기
    QRect bgRect = textRect;
    bgRect.moveTo(screenPos.x() - bgRect.width() / 2, screenPos.y() - bgRect.height() / 2);
    bgRect = bgRect.adjusted(-4, -2, 4, 2);  // 패딩 추가
    
    // 반투명 배경
    painter.setBrush(QBrush(QColor(0, 0, 0, 180)));
    painter.setPen(QPen(robotColor, 1));
    painter.drawRoundedRect(bgRect, 3, 3);
    
    // 텍스트 그리기
    painter.setPen(QPen(Qt::white));
    painter.drawText(bgRect, Qt::AlignCenter, robotName);
}

// ============================================================================
// GridMapRenderer 누락된 함수들 구현
// ============================================================================

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

} // namespace RenderHelper