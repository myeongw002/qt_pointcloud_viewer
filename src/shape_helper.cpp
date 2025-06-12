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

} // namespace Widget