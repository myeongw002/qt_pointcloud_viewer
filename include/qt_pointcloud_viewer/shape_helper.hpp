// shape_helper.hpp - Simple and practical
#ifndef SHAPE_HELPER_HPP
#define SHAPE_HELPER_HPP

#include <GL/gl.h>
#include "common_types.hpp"  // glm 대신 common_types 사용
#include <cmath>

namespace ShapeHelper {

class SimpleShape {
public:
    // Basic shapes
    static void drawCylinder(const Types::Vec3& position, 
                           const Types::Vec3& direction, 
                           float length, 
                           float radius, 
                           const Types::Vec4& color,
                           int segments = 16);
    
    static void drawCone(const Types::Vec3& position,
                           const Types::Vec3& direction,
                           float height,
                           float radius,
                           const Types::Vec4& color,
                           int segments = 16);
    
    static void drawCircle(const Types::Vec3& center,
                          const Types::Vec3& normal,
                          float radius,
                          const Types::Vec4& color,
                          int segments = 16);
    
    // Draw axes (with cone arrowheads)
    static void drawAxes(const Types::Vec3& position, 
                       const Types::Mat4& rotationMatrix,
                       float axesLength = 1.0f,
                       float axesRadius = 0.05f,
                       bool drawArrowheads = true);
    
    static void drawAxes(const Types::Vec3& position, 
                       const Types::Quat& orientation,
                       float axesLength = 1.0f,
                       float axesRadius = 0.05f,
                       bool drawArrowheads = true);
    
    // Draw ROS standard axes
    static void drawRosAxes(const Types::Vec3& position, 
                              const Types::Mat4& rotationMatrix,
                              float axesLength = 1.0f,
                              float axesRadius = 0.05f,
                              bool drawArrowheads = true);
        
    static void drawRosAxes(const Types::Vec3& position, 
                              const Types::Quat& orientation,
                              float axesLength = 1.0f,
                              float axesRadius = 0.05f,
                              bool drawArrowheads = true);
    
    // Draw single axis (with cone arrowhead)
    static void drawSingleAxis(const Types::Vec3& start,
                             const Types::Vec3& direction,
                             float length,
                             float radius,
                             const Types::Vec4& color,
                             bool drawArrowhead = true);
    
    // 구체 (Sphere)
    static void drawSphere(const Types::Vec3& center,
                          float radius,
                          const Types::Vec4& color,
                          int segments = 16);
    
    // 정육면체 (Cube)
    static void drawCube(const Types::Vec3& center,
                        float size,
                        const Types::Vec4& color);
};

} // namespace ShapeHelper

#endif // SHAPE_HELPER_HPP