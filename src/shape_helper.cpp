// shape_helper.cpp - Using common_types
#include "shape_helper.hpp"

namespace ShapeHelper {

void SimpleShape::drawCylinder(const Types::Vec3& position, 
                             const Types::Vec3& direction, 
                             float length, 
                             float radius, 
                             const Types::Vec4& color,
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
    Types::Vec3 normalizedDir = glm::normalize(direction);
    Types::Vec3 defaultAxis = Types::Vec3(0, 1, 0);
    
    if (std::abs(glm::dot(defaultAxis, normalizedDir) - 1.0f) > 0.001f) {
        Types::Vec3 rotationAxis = glm::cross(defaultAxis, normalizedDir);
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
    drawCircle(Types::Vec3(0, halfLength, 0), Types::Vec3(0, 1, 0), radius, color, segments);
    drawCircle(Types::Vec3(0, -halfLength, 0), Types::Vec3(0, -1, 0), radius, color, segments);
    
    glPopMatrix();
    
    // Restore blending state
    if (color.a < 1.0f && !blendWasEnabled) {
        glDisable(GL_BLEND);
    }
}

void SimpleShape::drawCircle(const Types::Vec3& center,
                           const Types::Vec3& normal,
                           float radius,
                           const Types::Vec4& color,
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

void SimpleShape::drawAxes(const Types::Vec3& position, 
                          const Types::Mat4& rotationMatrix,
                          float axesLength,
                          float axesRadius,
                          bool drawArrowheads) {
    // Extract axis vectors from rotation matrix
    Types::Vec3 xAxis = Types::Vec3(rotationMatrix * Types::Vec4(1, 0, 0, 0));
    Types::Vec3 yAxis = Types::Vec3(rotationMatrix * Types::Vec4(0, 1, 0, 0));
    Types::Vec3 zAxis = Types::Vec3(rotationMatrix * Types::Vec4(0, 0, 1, 0));
    
    // X-axis (Red) - Forward direction
    drawSingleAxis(position, xAxis, axesLength, axesRadius * 1.2f, 
                   Types::Vec4(1.0f, 0.0f, 0.0f, 1.0f), drawArrowheads);
    
    // Y-axis (Green) - Left direction
    drawSingleAxis(position, yAxis, axesLength, axesRadius, 
                   Types::Vec4(0.0f, 1.0f, 0.0f, 1.0f), drawArrowheads);
    
    // Z-axis (Blue) - Up direction
    drawSingleAxis(position, zAxis, axesLength, axesRadius, 
                   Types::Vec4(0.0f, 0.0f, 1.0f, 1.0f), drawArrowheads);
}

void SimpleShape::drawAxes(const Types::Vec3& position, 
                          const Types::Quat& orientation,
                          float axesLength,
                          float axesRadius,
                          bool drawArrowheads) {
    // Convert quaternion to rotation matrix
    Types::Mat4 rotationMatrix = glm::mat4_cast(orientation);
    
    // Call matrix version
    drawAxes(position, rotationMatrix, axesLength, axesRadius, drawArrowheads);
}

void SimpleShape::drawSingleAxis(const Types::Vec3& start,
                                const Types::Vec3& direction,
                                float length,
                                float radius,
                                const Types::Vec4& color,
                                bool drawArrowhead) {
    Types::Vec3 normalizedDir = glm::normalize(direction);
    
    if (drawArrowhead) {
        // With arrowhead: 75% for shaft, 25% for arrow
        float shaftLength = length * 0.75f;
        float arrowHeight = length * 0.45f;
        float arrowRadius = radius * 2.5f;
        
        // Draw axis shaft
        Types::Vec3 cylinderCenter = start + normalizedDir * (shaftLength * 0.5f);
        drawCylinder(cylinderCenter, normalizedDir, shaftLength, radius, color);
        
        // Draw cone arrowhead
        Types::Vec3 arrowBase = start + normalizedDir * shaftLength;
        
        // Arrow in brighter color
        Types::Vec4 arrowColor = Types::Vec4(
            glm::min(color.r + 0.3f, 1.0f),
            glm::min(color.g + 0.3f, 1.0f),
            glm::min(color.b + 0.3f, 1.0f),
            color.a
        );
        
        drawCone(arrowBase, normalizedDir, arrowHeight, arrowRadius, arrowColor);
        
    } else {
        // Without arrowhead: use full length for shaft
        Types::Vec3 cylinderCenter = start + normalizedDir * (length * 0.5f);
        drawCylinder(cylinderCenter, normalizedDir, length, radius, color);
    }
}

void SimpleShape::drawCone(const Types::Vec3& position,
                          const Types::Vec3& direction,
                          float height,
                          float radius,
                          const Types::Vec4& color,
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
    Types::Vec3 normalizedDir = glm::normalize(direction);
    Types::Vec3 defaultAxis = Types::Vec3(0, 1, 0);
    
    if (std::abs(glm::dot(defaultAxis, normalizedDir) - 1.0f) > 0.001f) {
        Types::Vec3 rotationAxis = glm::cross(defaultAxis, normalizedDir);
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
    drawCircle(Types::Vec3(0, 0, 0), Types::Vec3(0, -1, 0), radius, color, segments);
    
    glPopMatrix();
    
    // Restore blending state
    if (color.a < 1.0f && !blendWasEnabled) {
        glDisable(GL_BLEND);
    }
}

void SimpleShape::drawRosAxes(const Types::Vec3& position, 
                             const Types::Mat4& rotationMatrix,
                             float axesLength,
                             float axesRadius,
                             bool drawArrowheads) {
    // Extract axis vectors from ROS coordinate system
    Types::Vec3 rosX = Types::Vec3(rotationMatrix * Types::Vec4(1, 0, 0, 0));  // ROS X (forward)
    Types::Vec3 rosY = Types::Vec3(rotationMatrix * Types::Vec4(0, 1, 0, 0));  // ROS Y (left)
    Types::Vec3 rosZ = Types::Vec3(rotationMatrix * Types::Vec4(0, 0, 1, 0));  // ROS Z (up)
    
    // ROS → OpenGL coordinate conversion
    Types::Vec3 openglForward = Types::Vec3(-rosX.y, rosX.z, -rosX.x);  // ROS X → OpenGL coordinates
    Types::Vec3 openglLeft = Types::Vec3(-rosY.y, rosY.z, -rosY.x);     // ROS Y → OpenGL coordinates
    Types::Vec3 openglUp = Types::Vec3(-rosZ.y, rosZ.z, -rosZ.x);       // ROS Z → OpenGL coordinates
    
    // Draw axes with ROS standard colors
    // X-axis (Red) - Robot forward direction
    drawSingleAxis(position, openglForward, axesLength, axesRadius * 1.2f, 
                   Types::Vec4(1.0f, 0.0f, 0.0f, 1.0f), drawArrowheads);
    
    // Y-axis (Green) - Robot left direction
    drawSingleAxis(position, openglLeft, axesLength, axesRadius, 
                   Types::Vec4(0.0f, 1.0f, 0.0f, 1.0f), drawArrowheads);
    
    // Z-axis (Blue) - Robot up direction
    drawSingleAxis(position, openglUp, axesLength, axesRadius, 
                   Types::Vec4(0.0f, 0.0f, 1.0f, 1.0f), drawArrowheads);
}

void SimpleShape::drawRosAxes(const Types::Vec3& position, 
                             const Types::Quat& orientation,
                             float axesLength,
                             float axesRadius,
                             bool drawArrowheads) {
    Types::Mat4 rotationMatrix = glm::mat4_cast(orientation);
    drawRosAxes(position, rotationMatrix, axesLength, axesRadius, drawArrowheads);
}

void SimpleShape::drawSphere(const Types::Vec3& center,
                          float radius,
                          const Types::Vec4& color,
                          int segments) {
    // Enable blending for transparency handling
    bool blendWasEnabled = glIsEnabled(GL_BLEND);
    if (color.a < 1.0f && !blendWasEnabled) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }
    
    glColor4f(color.r, color.g, color.b, color.a);
    
    // Draw sphere using multiple slices and stacks
    for (int i = 0; i < segments; ++i) {
        float lat0 = M_PI * (-0.5f + float(i) / float(segments));
        float z0 = radius * sin(lat0);
        float r0 = radius * cos(lat0);
        
        float lat1 = M_PI * (-0.5f + float(i + 1) / float(segments));
        float z1 = radius * sin(lat1);
        float r1 = radius * cos(lat1);
        
        glBegin(GL_QUAD_STRIP);
        for (int j = 0; j <= segments; ++j) {
            float lon = 2 * M_PI * float(j) / float(segments);
            float x = r0 * cos(lon);
            float y = r0 * sin(lon);
            glVertex3f(x + center.x, y + center.y, z0 + center.z);
            x = r1 * cos(lon);
            y = r1 * sin(lon);
            glVertex3f(x + center.x, y + center.y, z1 + center.z);
        }
        glEnd();
    }
    
    // Restore blending state
    if (color.a < 1.0f && !blendWasEnabled) {
        glDisable(GL_BLEND);
    }
}

void SimpleShape::drawCube(const Types::Vec3& center,
                        float size,
                        const Types::Vec4& color) {
    // Calculate half-size
    float halfSize = size * 0.5f;
    
    // Define vertices for the cube
    Types::Vec3 vertices[8] = {
        Types::Vec3(-halfSize, -halfSize, -halfSize),
        Types::Vec3( halfSize, -halfSize, -halfSize),
        Types::Vec3( halfSize,  halfSize, -halfSize),
        Types::Vec3(-halfSize,  halfSize, -halfSize),
        Types::Vec3(-halfSize, -halfSize,  halfSize),
        Types::Vec3( halfSize, -halfSize,  halfSize),
        Types::Vec3( halfSize,  halfSize,  halfSize),
        Types::Vec3(-halfSize,  halfSize,  halfSize)
    };
    
    // Enable blending for transparency handling
    bool blendWasEnabled = glIsEnabled(GL_BLEND);
    if (color.a < 1.0f && !blendWasEnabled) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }
    
    glColor4f(color.r, color.g, color.b, color.a);
    
    // Draw the cube using quads
    glBegin(GL_QUADS);
    
    // Front face (z = -halfSize)
    glNormal3f(0.0f, 0.0f, -1.0f);
    glVertex3f(vertices[0].x + center.x, vertices[0].y + center.y, vertices[0].z + center.z);
    glVertex3f(vertices[1].x + center.x, vertices[1].y + center.y, vertices[1].z + center.z);
    glVertex3f(vertices[2].x + center.x, vertices[2].y + center.y, vertices[2].z + center.z);
    glVertex3f(vertices[3].x + center.x, vertices[3].y + center.y, vertices[3].z + center.z);
    
    // Back face (z = +halfSize)
    glNormal3f(0.0f, 0.0f, 1.0f);
    glVertex3f(vertices[7].x + center.x, vertices[7].y + center.y, vertices[7].z + center.z);
    glVertex3f(vertices[6].x + center.x, vertices[6].y + center.y, vertices[6].z + center.z);
    glVertex3f(vertices[5].x + center.x, vertices[5].y + center.y, vertices[5].z + center.z);
    glVertex3f(vertices[4].x + center.x, vertices[4].y + center.y, vertices[4].z + center.z);
    
    // Top face (y = +halfSize)
    glNormal3f(0.0f, 1.0f, 0.0f);
    glVertex3f(vertices[3].x + center.x, vertices[3].y + center.y, vertices[3].z + center.z);
    glVertex3f(vertices[2].x + center.x, vertices[2].y + center.y, vertices[2].z + center.z);
    glVertex3f(vertices[6].x + center.x, vertices[6].y + center.y, vertices[6].z + center.z);
    glVertex3f(vertices[7].x + center.x, vertices[7].y + center.y, vertices[7].z + center.z);
    
    // Bottom face (y = -halfSize)
    glNormal3f(0.0f, -1.0f, 0.0f);
    glVertex3f(vertices[0].x + center.x, vertices[0].y + center.y, vertices[0].z + center.z);
    glVertex3f(vertices[4].x + center.x, vertices[4].y + center.y, vertices[4].z + center.z);
    glVertex3f(vertices[5].x + center.x, vertices[5].y + center.y, vertices[5].z + center.z);
    glVertex3f(vertices[1].x + center.x, vertices[1].y + center.y, vertices[1].z + center.z);
    
    // Left face (x = -halfSize)
    glNormal3f(-1.0f, 0.0f, 0.0f);
    glVertex3f(vertices[0].x + center.x, vertices[0].y + center.y, vertices[0].z + center.z);
    glVertex3f(vertices[3].x + center.x, vertices[3].y + center.y, vertices[3].z + center.z);
    glVertex3f(vertices[7].x + center.x, vertices[7].y + center.y, vertices[7].z + center.z);
    glVertex3f(vertices[4].x + center.x, vertices[4].y + center.y, vertices[4].z + center.z);
    
    // Right face (x = +halfSize)
    glNormal3f(1.0f, 0.0f, 0.0f);
    glVertex3f(vertices[1].x + center.x, vertices[1].y + center.y, vertices[1].z + center.z);
    glVertex3f(vertices[5].x + center.x, vertices[5].y + center.y, vertices[5].z + center.z);
    glVertex3f(vertices[6].x + center.x, vertices[6].y + center.y, vertices[6].z + center.z);
    glVertex3f(vertices[2].x + center.x, vertices[2].y + center.y, vertices[2].z + center.z);
    
    glEnd();
    
    // Restore blending state
    if (color.a < 1.0f && !blendWasEnabled) {
        glDisable(GL_BLEND);
    }
}

} // namespace ShapeHelper