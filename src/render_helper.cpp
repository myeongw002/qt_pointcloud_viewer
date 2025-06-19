#include "render_helper.hpp"
#include "shape_helper.hpp"
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
    const QHash<QString, glm::vec3>& robotColors,
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
        glm::vec3 color = robotColors.value(robotName, glm::vec3(0.0f, 1.0f, 0.0f));
        glColor3f(color.x, color.y, color.z);
        
        // 포인트 렌더링 (ROS → OpenGL 좌표 변환)
        for (const auto& point : cloud->points) {
            glVertex3f(-point.y, point.z, -point.x);
        }
    }
    
    glEnd();
}

void PointCloudRenderer::drawPaths(
    const QHash<QString, PathVector>& paths,  // 타입 변경
    const QString& currentRobot,
    const QHash<QString, glm::vec3>& robotColors,
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
        
        const auto& path = it.value();  // 직접 벡터 참조
        if (path.empty()) continue;     // .empty() 사용 가능
        
        // 로봇 색상 설정
        glm::vec3 color = robotColors.value(robotName, glm::vec3(0.0f, 1.0f, 0.0f));
        glColor3f(color.x, color.y, color.z);
        
        // 경로 라인 렌더링
        for (size_t i = 1; i < path.size(); ++i) {     // .size() 사용 가능
            const auto& prev_pose = path[i-1];         // [] 연산자 사용 가능
            const auto& curr_pose = path[i];           // [] 연산자 사용 가능
            
            // ROS → OpenGL 좌표 변환
            glVertex3f(-prev_pose.pose.position.y, prev_pose.pose.position.z, -prev_pose.pose.position.x);
            glVertex3f(-curr_pose.pose.position.y, curr_pose.pose.position.z, -curr_pose.pose.position.x);
        }
    }
    
    glEnd();
}

void PointCloudRenderer::drawPositions(
    const QHash<QString, PathVector>& paths,
    const QString& currentRobot,
    const QHash<QString, glm::vec3>& robotColors,
    PositionMarkerType markerType,
    float radius,           // Marker Size 슬라이더 값 (0.1 ~ 2.0)
    float height,           // Cylinder 전용
    float axesLength,       // 사용하지 않음 (radius로 대체)
    float axesRadius,       // 사용하지 않음 (radius로 대체)
    std::mutex& pathMutex) {
    
    std::lock_guard<std::mutex> lock(pathMutex);
    
    for (auto it = paths.cbegin(); it != paths.cend(); ++it) {
        const QString& robotName = it.key();
        
        // 현재 로봇 필터링
        if (currentRobot != "COMBINED" && robotName != currentRobot) {
            continue;
        }
        
        const auto& path = it.value();
        if (path.empty()) continue;
        
        const auto& currentPose = path.back();
        
        // 위치 및 방향 계산 (ROS → OpenGL 변환)
        glm::vec3 position(
            -currentPose.pose.position.y,
            currentPose.pose.position.z,
            -currentPose.pose.position.x
        );
        
        auto quat = currentPose.pose.orientation;
        glm::quat orientation(quat.w, quat.x, quat.y, quat.z);
        
        glm::vec3 robotColor = robotColors.value(robotName, glm::vec3(0.0f, 1.0f, 0.0f));
        
        // 마커 타입에 따른 렌더링
        switch (markerType) {
            case PositionMarkerType::CYLINDER:
                // Cylinder: 기존과 동일 (radius, height 사용)
                drawCylinderMarker(position, robotColor, radius, height);
                break;
                
            case PositionMarkerType::AXES:
                // Axes: radius 값을 기반으로 적절한 길이와 두께 계산
                {
                    float calculatedLength = radius * 3.0f;  // 슬라이더 값의 3배를 길이로
                    float calculatedRadius = radius * 0.15f; // 슬라이더 값의 15%를 두께로
                    
                    drawPositionAxes(position, orientation, robotName, 
                                   calculatedLength, calculatedRadius);
                }
                break;
        }
    }
}

void PointCloudRenderer::drawAxes(
    const glm::vec3& origin,
    float axesLength,
    float axesRadius) {
    
    glm::mat4 identityMatrix = glm::mat4(1.0f);
    
    ShapeHelper::SimpleShape::drawRosAxes(
        origin,
        identityMatrix,
        axesLength,
        axesRadius,
        true // drawArrowheads
    );
}

void PointCloudRenderer::drawGrid(
    int cellCount,
    float cellSize,
    float lineWidth) {
    
    glColor3f(0.3f, 0.3f, 0.3f);
    glLineWidth(lineWidth);
    
    glBegin(GL_LINES);
    
    float halfSize = cellCount * cellSize * 0.5f;
    
    // X 방향 선들 (Z축을 따라)
    for (int i = -cellCount/2; i <= cellCount/2; ++i) {
        float pos = i * cellSize;
        glVertex3f(-halfSize, 0.0f, pos);
        glVertex3f(halfSize, 0.0f, pos);
    }
    
    // Z 방향 선들 (X축을 따라)
    for (int i = -cellCount/2; i <= cellCount/2; ++i) {
        float pos = i * cellSize;
        glVertex3f(pos, 0.0f, -halfSize);
        glVertex3f(pos, 0.0f, halfSize);
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
    const QHash<QString, glm::vec3>& robotColors) {
    
    if (currentRobot == "COMBINED") {
        QStringList robots = {"TUGV", "MUGV", "SUGV1", "SUGV2", "SUAV"};
        
        for (int i = 0; i < robots.size(); ++i) {
            QString robotName = robots[i];
            glm::vec3 color = robotColors.value(robotName, glm::vec3(0.0f, 1.0f, 0.0f));
            QColor robotColor(color.x * 255, color.y * 255, color.z * 255);
            
            int yOffset = 10 + i * 40;
            drawSingleLabel(painter, robotName, robotColor, QPoint(10, yOffset));  // 이 부분 수정됨
        }
    } else {
        glm::vec3 color = robotColors.value(currentRobot, glm::vec3(0.0f, 1.0f, 0.0f));
        QColor robotColor(color.x * 255, color.y * 255, color.z * 255);
        drawSingleLabel(painter, currentRobot, robotColor, QPoint(10, 10));
    }
}

void PointCloudRenderer::drawRobotNamesAtPositions(
    QPainter& painter,
    const QHash<QString, PathVector>& paths,
    const QString& currentRobot,
    const QHash<QString, glm::vec3>& robotColors,
    const glm::mat4& viewMatrix,
    const glm::mat4& projectionMatrix,
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
        
        const auto& path = it.value();
        if (path.empty()) continue;
        
        // 로봇의 현재 위치 (마지막 경로 포인트)
        const auto& currentPose = path.back();
        
        // ROS → OpenGL 좌표 변환
        glm::vec3 worldPosition(
            -currentPose.pose.position.y,
            currentPose.pose.position.z + 2.0f,  // 로봇 위 50cm에 텍스트 표시
            -currentPose.pose.position.x
        );
        
        // 3D 위치를 화면 좌표로 변환
        QPoint screenPos = worldToScreen(worldPosition, viewMatrix, projectionMatrix, 
                                       screenWidth, screenHeight);
        
        // 화면 범위 내에 있는지 확인
        if (screenPos.x() >= 0 && screenPos.x() < screenWidth && 
            screenPos.y() >= 0 && screenPos.y() < screenHeight) {
            
            // 로봇 색상 설정
            glm::vec3 color = robotColors.value(robotName, glm::vec3(0.0f, 1.0f, 0.0f));
            QColor robotColor(color.x * 255, color.y * 255, color.z * 255);
            
            // 로봇 이름 텍스트 그리기
            drawRobotNameText(painter, robotName, robotColor, screenPos, textSize);
        }
    }
}

QPoint PointCloudRenderer::worldToScreen(
    const glm::vec3& worldPos,
    const glm::mat4& viewMatrix,
    const glm::mat4& projectionMatrix,
    int screenWidth,
    int screenHeight) {
    
    // MVP 변환 매트릭스
    glm::mat4 mvpMatrix = projectionMatrix * viewMatrix;
    
    // 월드 좌표를 동차 좌표로 변환
    glm::vec4 worldPos4(worldPos.x, worldPos.y, worldPos.z, 1.0f);
    
    // MVP 변환 적용
    glm::vec4 clipSpacePos = mvpMatrix * worldPos4;
    
    // 원근 나누기 (Perspective Division)
    if (clipSpacePos.w != 0.0f) {
        clipSpacePos.x /= clipSpacePos.w;
        clipSpacePos.y /= clipSpacePos.w;
        clipSpacePos.z /= clipSpacePos.w;
    }
    
    // NDC [-1, 1] → 화면 좌표 [0, screen_size] 변환
    int screenX = static_cast<int>((clipSpacePos.x + 1.0f) * 0.5f * screenWidth);
    int screenY = static_cast<int>((1.0f - clipSpacePos.y) * 0.5f * screenHeight);  // Y축 뒤집기
    
    return QPoint(screenX, screenY);
}

void PointCloudRenderer::drawRobotNameText(
    QPainter& painter,
    const QString& robotName,
    const QColor& robotColor,
    const QPoint& screenPos,
    float textSize) {
    
    // 폰트 설정
    QFont font("Arial", static_cast<int>(textSize), QFont::Bold);
    painter.setFont(font);
    QFontMetrics fm(font);
    
    // 텍스트 크기 계산
    QRect textRect = fm.boundingRect(robotName);
    int textWidth = textRect.width();
    int textHeight = fm.height();
    
    // 배경 박스 크기 계산
    int padding = 4;
    int boxWidth = textWidth + padding * 2;
    int boxHeight = textHeight + padding * 2;
    
    // 텍스트 중앙 정렬을 위한 위치 조정
    QPoint adjustedPos(screenPos.x() - boxWidth / 2, screenPos.y() - boxHeight / 2);
    QRect backgroundRect(adjustedPos.x(), adjustedPos.y(), boxWidth, boxHeight);
    
    // 배경 박스 그리기 (반투명 검은색)
    painter.setBrush(QBrush(QColor(0, 0, 0, 150)));
    painter.setPen(Qt::NoPen);
    painter.drawRoundedRect(backgroundRect, 3, 3);
    
    // 테두리 그리기 (로봇 색상)
    painter.setBrush(Qt::NoBrush);
    painter.setPen(QPen(robotColor, 2));
    painter.drawRoundedRect(backgroundRect, 3, 3);
    
    // 텍스트 그리기 (흰색)
    painter.setPen(QPen(Qt::white));
    int textX = adjustedPos.x() + padding;
    int textY = adjustedPos.y() + padding + fm.ascent();
    painter.drawText(textX, textY, robotName);
}

// ============================================================================
// Private Helper Functions
// ============================================================================

void PointCloudRenderer::drawCylinderMarker(
    const glm::vec3& position,
    const glm::vec3& robotColor,
    float radius,
    float height) {
    
    // 메인 실린더
    ShapeHelper::SimpleShape::drawCylinder(
        position + glm::vec3(0, height/2, 0),
        glm::vec3(0, 1, 0),
        height,
        radius,
        glm::vec4(robotColor.x, robotColor.y, robotColor.z, 0.8f)
    );
    
    // 상단 표시등
    ShapeHelper::SimpleShape::drawCylinder(
        position + glm::vec3(0, height + 0.05f, 0),
        glm::vec3(0, 1, 0),
        0.05f,
        radius * 0.7f,
        glm::vec4(1.0f, 1.0f, 1.0f, 1.0f)
    );
}

void PointCloudRenderer::drawPositionAxes(
    const glm::vec3& position,
    const glm::quat& orientation,
    const QString& robotName,
    float axesLength,    // 이제 Marker Size 슬라이더 기반 계산된 길이
    float axesRadius) {  // 이제 Marker Size 슬라이더 기반 계산된 두께
    
    // 로봇별 크기 조정 (기존 로직 유지)
    float finalLength = axesLength;
    float finalRadius = axesRadius;
    
    // 바닥에서 살짝 위에 시작점 설정
    glm::vec3 axesStart = position + glm::vec3(0, finalRadius, 0);
    
    ShapeHelper::SimpleShape::drawRosAxes(
        axesStart,
        orientation,
        finalLength,
        finalRadius,
        true  // drawArrowheads
    );
}

void PointCloudRenderer::drawSingleLabel(
    QPainter& painter,
    const QString& text,
    const QColor& robotColor,
    const QPoint& position) {
    
    const int fontSize = 10;
    const int circleSize = 12;
    const int circleMargin = 5;
    const int horizontalMargin = 8;
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

void GridMapRenderer::drawBasicGridMaps(
    const QHash<QString, std::shared_ptr<GridMap::GridMapData>>& gridMaps,
    const QString& currentRobot,
    const QHash<QString, glm::vec3>& robotColors,
    std::mutex& gridMapMutex) {
    
    std::lock_guard<std::mutex> lock(gridMapMutex);
    
    if (gridMaps.isEmpty()) return;
    
    // OpenGL 상태 설정
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    if (currentRobot == "COMBINED") {
        // ✅ 개선: 통합 렌더링 방식 사용
        drawCombinedGridMap(gridMaps, robotColors);
    } else {
        // 특정 로봇의 그리드맵만 렌더링
        if (gridMaps.contains(currentRobot)) {
            auto gridData = gridMaps[currentRobot];
            glm::vec3 gridColor = robotColors.value(currentRobot, glm::vec3(0.5f, 0.5f, 0.5f));
            drawSingleGridMap(gridData, gridColor);
        }
    }
    
    glDisable(GL_BLEND);
}

void GridMapRenderer::drawCombinedGridMap(
    const QHash<QString, std::shared_ptr<GridMap::GridMapData>>& gridMaps,
    const QHash<QString, glm::vec3>& robotColors) {
    
    // 1. 통합 바운딩 박스 계산
    float minX = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::lowest();
    float minY = std::numeric_limits<float>::max();
    float maxY = std::numeric_limits<float>::lowest();
    
    for (auto it = gridMaps.cbegin(); it != gridMaps.cend(); ++it) {
        const auto& gridData = it.value();
        if (!gridData || !gridData->isValid()) continue;
        
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
        
        if (!gridData || !gridData->isValid()) continue;
        
        glm::vec3 robotColor = robotColors.value(robotName, glm::vec3(0.0f, 1.0f, 0.0f));
        drawObstaclesOnly(gridData, robotColor);
    }
}

void GridMapRenderer::drawSingleGridMap(
    const std::shared_ptr<GridMap::GridMapData>& gridData,
    const glm::vec3& robotColor) {
    
    if (!gridData || !gridData->isValid()) return;
    
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

void GridMapRenderer::draw2DProjectedPaths(
    const QHash<QString, PathVector>& paths,  // 타입 변경
    const QString& currentRobot,
    const QHash<QString, glm::vec3>& robotColors,
    std::mutex& pathMutex,
    float pathWidth) {
    
    std::lock_guard<std::mutex> lock(pathMutex);
    
    // 기존 PointCloudRenderer::drawPaths와 동일하되 Z좌표를 고정
    glLineWidth(pathWidth);
    glBegin(GL_LINES);
    
    for (auto it = paths.cbegin(); it != paths.cend(); ++it) {
        const QString& robotName = it.key();
        
        // 현재 로봇 필터링
        if (currentRobot != "COMBINED" && robotName != currentRobot) {
            continue;
        }
        
        const auto& path = it.value();  // 이제 직접 vector 참조
        if (path.empty()) continue;     // 포인터 연산자 제거
        
        // 로봇 색상 설정
        glm::vec3 color = robotColors.value(robotName, glm::vec3(0.0f, 1.0f, 0.0f));
        glColor4f(color.x, color.y, color.z, 0.9f);  // 약간 투명
        
        // 경로 라인 렌더링 (Z좌표 고정으로 2D 평면화)
        for (size_t i = 1; i < path.size(); ++i) {      // 포인터 연산자 제거
            const auto& prev_pose = path[i-1];          // 역참조 연산자 제거
            const auto& curr_pose = path[i];            // 역참조 연산자 제거
            
            // ROS → OpenGL 좌표 변환, Z좌표는 GridMap 위에 고정
            float z = 0.02f;  // GridMap보다 살짝 위
            glVertex3f(-prev_pose.pose.position.y, z, -prev_pose.pose.position.x);
            glVertex3f(-curr_pose.pose.position.y, z, -curr_pose.pose.position.x);
        }
    }
    
    glEnd();
    
    // 방향 표시 (화살표 점들)
    // drawPathDirectionMarkers(paths, currentRobot, robotColors, pathMutex);
}

void GridMapRenderer::drawObstaclesOnly(
    const std::shared_ptr<GridMap::GridMapData>& gridData,
    const glm::vec3& robotColor) {
    
    const cv::Mat& occupancyMap = gridData->occupancyMap;
    float resolution = gridData->resolution;
    float originX = gridData->originX;
    float originY = gridData->originY;
    
    // 장애물만 그리기 (배경 제외)
    glColor4f(robotColor.r, robotColor.g, robotColor.b, 0.7f);  // 약간 투명
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

} // namespace RenderHelper