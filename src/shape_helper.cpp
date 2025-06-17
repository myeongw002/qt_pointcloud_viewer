// shape_helper.cpp - Reuse current code
#include "shape_helper.hpp"

namespace ShapeHelper {

void SimpleShape::drawCylinder(const glm::vec3& position, 
                             const glm::vec3& direction, 
                             float length, 
                             float radius, 
                             const glm::vec4& color,
                             int segments) {
    // Enable blending for transparency handling
    bool blendWasEnabled = glIsEnabled(GL_BLEND);
    if (color.a < 1.0f && !blendWasEnabled) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }
    
    glColor4f(color.r, color.g, color.b, color.a);
    
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    
    glTranslatef(position.x, position.y, position.z);
    
    // Rotate according to direction
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
    
    // Draw side faces
    glBegin(GL_QUAD_STRIP);
    for (int i = 0; i <= segments; ++i) {
        float angle = 2.0f * M_PI * float(i) / float(segments);
        float x = radius * cos(angle);
        float z = radius * sin(angle);
        
        glVertex3f(x, -halfLength, z);
        glVertex3f(x, halfLength, z);
    }
    glEnd();
    
    // Top/bottom circles
    drawCircle(glm::vec3(0, halfLength, 0), glm::vec3(0, 1, 0), radius, color, segments);
    drawCircle(glm::vec3(0, -halfLength, 0), glm::vec3(0, -1, 0), radius, color, segments);
    
    glPopMatrix();
    
    // Restore blending state
    if (color.a < 1.0f && !blendWasEnabled) {
        glDisable(GL_BLEND);
    }
}

void SimpleShape::drawCircle(const glm::vec3& center,
                           const glm::vec3& normal,
                           float radius,
                           const glm::vec4& color,
                           int segments) {
    // Enable blending for transparency handling
    bool blendWasEnabled = glIsEnabled(GL_BLEND);
    if (color.a < 1.0f && !blendWasEnabled) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }
    
    glColor4f(color.r, color.g, color.b, color.a);
    
    glBegin(GL_TRIANGLE_FAN);
    glVertex3f(center.x, center.y, center.z);
    
    bool reverse = normal.y < 0;  // Bottom face is in reverse order
    for (int i = 0; i <= segments; ++i) {
        int idx = reverse ? (segments - i) : i;
        float angle = 2.0f * M_PI * float(idx) / float(segments);
        float x = radius * cos(angle);
        float z = radius * sin(angle);
        glVertex3f(center.x + x, center.y, center.z + z);
    }
    glEnd();
    
    // Restore blending state
    if (color.a < 1.0f && !blendWasEnabled) {
        glDisable(GL_BLEND);
    }
}

void SimpleShape::drawAxes(const glm::vec3& position, 
                          const glm::mat4& rotationMatrix,
                          float axesLength,
                          float axesRadius,
                          bool drawArrowheads) {
    // Extract axis vectors from rotation matrix
    glm::vec3 xAxis = glm::vec3(rotationMatrix * glm::vec4(1, 0, 0, 0));
    glm::vec3 yAxis = glm::vec3(rotationMatrix * glm::vec4(0, 1, 0, 0));
    glm::vec3 zAxis = glm::vec3(rotationMatrix * glm::vec4(0, 0, 1, 0));
    
    // X-axis (Red) - Forward direction
    drawSingleAxis(position, xAxis, axesLength, axesRadius * 1.2f, 
                   glm::vec4(1.0f, 0.0f, 0.0f, 1.0f), drawArrowheads);
    
    // Y-axis (Green) - Left direction
    drawSingleAxis(position, yAxis, axesLength, axesRadius, 
                   glm::vec4(0.0f, 1.0f, 0.0f, 1.0f), drawArrowheads);
    
    // Z-axis (Blue) - Up direction
    drawSingleAxis(position, zAxis, axesLength, axesRadius, 
                   glm::vec4(0.0f, 0.0f, 1.0f, 1.0f), drawArrowheads);
}

void SimpleShape::drawAxes(const glm::vec3& position, 
                          const glm::quat& orientation,
                          float axesLength,
                          float axesRadius,
                          bool drawArrowheads) {
    // Convert quaternion to rotation matrix
    glm::mat4 rotationMatrix = glm::mat4_cast(orientation);
    
    // Call matrix version
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
        // With arrowhead: 75% for shaft, 25% for arrow
        float shaftLength = length * 0.75f;
        float arrowHeight = length * 0.45f;
        float arrowRadius = radius * 2.5f;
        
        // Draw axis shaft
        glm::vec3 cylinderCenter = start + normalizedDir * (shaftLength * 0.5f);
        drawCylinder(cylinderCenter, normalizedDir, shaftLength, radius, color);
        
        // Draw cone arrowhead
        glm::vec3 arrowBase = start + normalizedDir * shaftLength;
        
        // Arrow in brighter color
        glm::vec4 arrowColor = glm::vec4(
            glm::min(color.r + 0.3f, 1.0f),
            glm::min(color.g + 0.3f, 1.0f),
            glm::min(color.b + 0.3f, 1.0f),
            color.a
        );
        
        drawCone(arrowBase, normalizedDir, arrowHeight, arrowRadius, arrowColor);
        
    } else {
        // Without arrowhead: use full length for shaft
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
    // Handle transparency
    bool blendWasEnabled = glIsEnabled(GL_BLEND);
    if (color.a < 1.0f && !blendWasEnabled) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }
    
    glColor4f(color.r, color.g, color.b, color.a);
    
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    
    glTranslatef(position.x, position.y, position.z);
    
    // Rotate according to direction
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
    
    // Draw cone side surface
    glBegin(GL_TRIANGLE_FAN);
    glVertex3f(0, height, 0);  // Cone apex
    
    for (int i = 0; i <= segments; ++i) {
        float angle = 2.0f * M_PI * float(i) / float(segments);
        float x = radius * cos(angle);
        float z = radius * sin(angle);
        glVertex3f(x, 0, z);
    }
    glEnd();
    
    // Draw cone bottom
    drawCircle(glm::vec3(0, 0, 0), glm::vec3(0, -1, 0), radius, color, segments);
    
    glPopMatrix();
    
    // Restore blending state
    if (color.a < 1.0f && !blendWasEnabled) {
        glDisable(GL_BLEND);
    }
}

void SimpleShape::drawRosAxes(const glm::vec3& position, 
                             const glm::mat4& rotationMatrix,
                             float axesLength,
                             float axesRadius,
                             bool drawArrowheads) {
    // Extract axis vectors from ROS coordinate system
    glm::vec3 rosX = glm::vec3(rotationMatrix * glm::vec4(1, 0, 0, 0));  // ROS X (forward)
    glm::vec3 rosY = glm::vec3(rotationMatrix * glm::vec4(0, 1, 0, 0));  // ROS Y (left)
    glm::vec3 rosZ = glm::vec3(rotationMatrix * glm::vec4(0, 0, 1, 0));  // ROS Z (up)
    
    // ROS → OpenGL coordinate conversion
    glm::vec3 openglForward = glm::vec3(-rosX.y, rosX.z, -rosX.x);  // ROS X → OpenGL coordinates
    glm::vec3 openglLeft = glm::vec3(-rosY.y, rosY.z, -rosY.x);     // ROS Y → OpenGL coordinates
    glm::vec3 openglUp = glm::vec3(-rosZ.y, rosZ.z, -rosZ.x);       // ROS Z → OpenGL coordinates
    
    // Draw axes with ROS standard colors
    // X-axis (Red) - Robot forward direction
    drawSingleAxis(position, openglForward, axesLength, axesRadius * 1.2f, 
                   glm::vec4(1.0f, 0.0f, 0.0f, 1.0f), drawArrowheads);
    
    // Y-axis (Green) - Robot left direction
    drawSingleAxis(position, openglLeft, axesLength, axesRadius, 
                   glm::vec4(0.0f, 1.0f, 0.0f, 1.0f), drawArrowheads);
    
    // Z-axis (Blue) - Robot up direction
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

} // namespace ShapeHelper