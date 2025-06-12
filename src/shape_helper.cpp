// shape_helper.cpp - 현재 코드를 재사용
#include "shape_helper.hpp"

namespace ShapeHelper {

void SimpleShape::drawCylinder(const glm::vec3& position, 
                             const glm::vec3& direction, 
                             float length, 
                             float radius, 
                             const glm::vec4& color,
                             int segments) {
    // 투명도 처리를 위한 블렌딩 활성화
    bool blendWasEnabled = glIsEnabled(GL_BLEND);
    if (color.a < 1.0f && !blendWasEnabled) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }
    
    glColor4f(color.r, color.g, color.b, color.a);
    
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    
    glTranslatef(position.x, position.y, position.z);
    
    // 방향에 맞게 회전
    glm::vec3 normalizedDir = glm::normalize(direction);
    glm::vec3 defaultAxis = glm::vec3(0, 1, 0);
    
    if (std::abs(glm::dot(defaultAxis, normalizedDir) - 1.0f) > 0.001f) {
        glm::vec3 rotationAxis = glm::cross(defaultAxis, normalizedDir);
        if (glm::length(rotationAxis) > 0.001f) {
            rotationAxis = glm::normalize(rotationAxis);
            float rotationAngle = acos(glm::dot(defaultAxis, normalizedDir));
            glRotatef(glm::degrees(rotationAngle), rotationAxis.x, rotationAxis.y, rotationAxis.z);
        }
    }
    
    float halfLength = length * 0.5f;
    
    // 측면 그리기
    glBegin(GL_QUAD_STRIP);
    for (int i = 0; i <= segments; ++i) {
        float angle = 2.0f * M_PI * float(i) / float(segments);
        float x = radius * cos(angle);
        float z = radius * sin(angle);
        
        glVertex3f(x, -halfLength, z);
        glVertex3f(x, halfLength, z);
    }
    glEnd();
    
    // 상단/하단 원판
    drawCircle(glm::vec3(0, halfLength, 0), glm::vec3(0, 1, 0), radius, color, segments);
    drawCircle(glm::vec3(0, -halfLength, 0), glm::vec3(0, -1, 0), radius, color, segments);
    
    glPopMatrix();
    
    // 블렌딩 상태 복원
    if (color.a < 1.0f && !blendWasEnabled) {
        glDisable(GL_BLEND);
    }
}

void SimpleShape::drawCircle(const glm::vec3& center,
                           const glm::vec3& normal,
                           float radius,
                           const glm::vec4& color,
                           int segments) {
    // 투명도 처리를 위한 블렌딩 활성화
    bool blendWasEnabled = glIsEnabled(GL_BLEND);
    if (color.a < 1.0f && !blendWasEnabled) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }
    
    glColor4f(color.r, color.g, color.b, color.a);
    
    glBegin(GL_TRIANGLE_FAN);
    glVertex3f(center.x, center.y, center.z);
    
    bool reverse = normal.y < 0;  // 하단면은 역순
    for (int i = 0; i <= segments; ++i) {
        int idx = reverse ? (segments - i) : i;
        float angle = 2.0f * M_PI * float(idx) / float(segments);
        float x = radius * cos(angle);
        float z = radius * sin(angle);
        glVertex3f(center.x + x, center.y, center.z + z);
    }
    glEnd();
    
    // 블렌딩 상태 복원
    if (color.a < 1.0f && !blendWasEnabled) {
        glDisable(GL_BLEND);
    }
}

void SimpleShape::drawAxes(const glm::vec3& position, 
                          const glm::mat4& rotationMatrix,
                          float axesLength,
                          float axesRadius,
                          bool drawArrowheads) {
    // 회전 매트릭스에서 축 벡터 추출
    glm::vec3 xAxis = glm::vec3(rotationMatrix * glm::vec4(1, 0, 0, 0));
    glm::vec3 yAxis = glm::vec3(rotationMatrix * glm::vec4(0, 1, 0, 0));
    glm::vec3 zAxis = glm::vec3(rotationMatrix * glm::vec4(0, 0, 1, 0));
    
    // X축 (빨간색) - Forward 방향
    drawSingleAxis(position, xAxis, axesLength, axesRadius * 1.2f, 
                   glm::vec4(1.0f, 0.0f, 0.0f, 1.0f), drawArrowheads);
    
    // Y축 (초록색) - Left 방향
    drawSingleAxis(position, yAxis, axesLength, axesRadius, 
                   glm::vec4(0.0f, 1.0f, 0.0f, 1.0f), drawArrowheads);
    
    // Z축 (파란색) - Up 방향
    drawSingleAxis(position, zAxis, axesLength, axesRadius, 
                   glm::vec4(0.0f, 0.0f, 1.0f, 1.0f), drawArrowheads);
}

void SimpleShape::drawAxes(const glm::vec3& position, 
                          const glm::quat& orientation,
                          float axesLength,
                          float axesRadius,
                          bool drawArrowheads) {
    // 쿼터니언을 회전 매트릭스로 변환
    glm::mat4 rotationMatrix = glm::mat4_cast(orientation);
    
    // 매트릭스 버전 호출
    drawAxes(position, rotationMatrix, axesLength, axesRadius, drawArrowheads);
}

void SimpleShape::drawSingleAxis(const glm::vec3& start,
                                const glm::vec3& direction,
                                float length,
                                float radius,
                                const glm::vec4& color,
                                bool drawArrowhead) {
    glm::vec3 normalizedDir = glm::normalize(direction);
    
    if (drawArrowhead) {
        // 화살표가 있을 때: 축 몸체 80%, 화살표 20%
        float shaftLength = length * 0.75f;
        float arrowHeight = length * 0.45f;
        float arrowRadius = radius * 2.5f;
        
        // 축 몸체 그리기
        glm::vec3 cylinderCenter = start + normalizedDir * (shaftLength * 0.5f);
        drawCylinder(cylinderCenter, normalizedDir, shaftLength, radius, color);
        
        // 원뿔 화살표 그리기
        glm::vec3 arrowBase = start + normalizedDir * shaftLength;
        
        // 화살표는 더 밝은 색상으로
        glm::vec4 arrowColor = glm::vec4(
            glm::min(color.r + 0.3f, 1.0f),
            glm::min(color.g + 0.3f, 1.0f),
            glm::min(color.b + 0.3f, 1.0f),
            color.a
        );
        
        drawCone(arrowBase, normalizedDir, arrowHeight, arrowRadius, arrowColor);
        
    } else {
        // 화살표가 없을 때: 전체 길이를 축 몸체로
        glm::vec3 cylinderCenter = start + normalizedDir * (length * 0.5f);
        drawCylinder(cylinderCenter, normalizedDir, length, radius, color);
    }
}

void SimpleShape::drawCone(const glm::vec3& position,
                          const glm::vec3& direction,
                          float height,
                          float radius,
                          const glm::vec4& color,
                          int segments) {
    // 투명도 처리
    bool blendWasEnabled = glIsEnabled(GL_BLEND);
    if (color.a < 1.0f && !blendWasEnabled) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }
    
    glColor4f(color.r, color.g, color.b, color.a);
    
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    
    glTranslatef(position.x, position.y, position.z);
    
    // 방향에 맞게 회전
    glm::vec3 normalizedDir = glm::normalize(direction);
    glm::vec3 defaultAxis = glm::vec3(0, 1, 0);
    
    if (std::abs(glm::dot(defaultAxis, normalizedDir) - 1.0f) > 0.001f) {
        glm::vec3 rotationAxis = glm::cross(defaultAxis, normalizedDir);
        if (glm::length(rotationAxis) > 0.001f) {
            rotationAxis = glm::normalize(rotationAxis);
            float rotationAngle = acos(glm::dot(defaultAxis, normalizedDir));
            glRotatef(glm::degrees(rotationAngle), rotationAxis.x, rotationAxis.y, rotationAxis.z);
        }
    }
    
    // 원뿔 측면 그리기
    glBegin(GL_TRIANGLE_FAN);
    glVertex3f(0, height, 0);  // 원뿔 꼭짓점
    
    for (int i = 0; i <= segments; ++i) {
        float angle = 2.0f * M_PI * float(i) / float(segments);
        float x = radius * cos(angle);
        float z = radius * sin(angle);
        glVertex3f(x, 0, z);
    }
    glEnd();
    
    // 원뿔 바닥면 그리기
    drawCircle(glm::vec3(0, 0, 0), glm::vec3(0, -1, 0), radius, color, segments);
    
    glPopMatrix();
    
    // 블렌딩 상태 복원
    if (color.a < 1.0f && !blendWasEnabled) {
        glDisable(GL_BLEND);
    }
}

void SimpleShape::drawRosAxes(const glm::vec3& position, 
                             const glm::mat4& rotationMatrix,
                             float axesLength,
                             float axesRadius,
                             bool drawArrowheads) {
    // ROS 좌표계에서 축 벡터 추출
    glm::vec3 rosX = glm::vec3(rotationMatrix * glm::vec4(1, 0, 0, 0));  // ROS X (forward)
    glm::vec3 rosY = glm::vec3(rotationMatrix * glm::vec4(0, 1, 0, 0));  // ROS Y (left)
    glm::vec3 rosZ = glm::vec3(rotationMatrix * glm::vec4(0, 0, 1, 0));  // ROS Z (up)
    
    // ROS → OpenGL 좌표 변환
    glm::vec3 openglForward = glm::vec3(-rosX.y, rosX.z, -rosX.x);  // ROS X → OpenGL 좌표
    glm::vec3 openglLeft = glm::vec3(-rosY.y, rosY.z, -rosY.x);     // ROS Y → OpenGL 좌표
    glm::vec3 openglUp = glm::vec3(-rosZ.y, rosZ.z, -rosZ.x);       // ROS Z → OpenGL 좌표
    
    // ROS 표준 색상으로 축 그리기
    // X축 (빨간색) - 로봇의 앞 방향 (Forward)
    drawSingleAxis(position, openglForward, axesLength, axesRadius * 1.2f, 
                   glm::vec4(1.0f, 0.0f, 0.0f, 1.0f), drawArrowheads);
    
    // Y축 (초록색) - 로봇의 왼쪽 방향 (Left)
    drawSingleAxis(position, openglLeft, axesLength, axesRadius, 
                   glm::vec4(0.0f, 1.0f, 0.0f, 1.0f), drawArrowheads);
    
    // Z축 (파란색) - 로봇의 위쪽 방향 (Up)
    drawSingleAxis(position, openglUp, axesLength, axesRadius, 
                   glm::vec4(0.0f, 0.0f, 1.0f, 1.0f), drawArrowheads);
}

void SimpleShape::drawRosAxes(const glm::vec3& position, 
                             const glm::quat& orientation,
                             float axesLength,
                             float axesRadius,
                             bool drawArrowheads) {
    glm::mat4 rotationMatrix = glm::mat4_cast(orientation);
    drawRosAxes(position, rotationMatrix, axesLength, axesRadius, drawArrowheads);
}

} // namespace Widget