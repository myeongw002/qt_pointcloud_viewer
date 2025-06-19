// shape_helper.hpp - Simple and practical
#ifndef SHAPE_HELPER_HPP
#define SHAPE_HELPER_HPP

#include <GL/gl.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/quaternion.hpp>  // Added this header (includes mat4_cast function)
#include <cmath>

namespace ShapeHelper {

class SimpleShape {
public:
    // Basic shapes
    static void drawCylinder(const glm::vec3& position, 
                           const glm::vec3& direction, 
                           float length, 
                           float radius, 
                           const glm::vec4& color,
                           int segments = 16);
    
    static void drawCone(const glm::vec3& position,
                           const glm::vec3& direction,
                           float height,
                           float radius,
                           const glm::vec4& color,
                           int segments = 16);
    
    static void drawCircle(const glm::vec3& center,
                          const glm::vec3& normal,
                          float radius,
                          const glm::vec4& color,
                          int segments = 16);
    
    // Draw axes (with cone arrowheads)
    static void drawAxes(const glm::vec3& position, 
                       const glm::mat4& rotationMatrix,
                       float axesLength = 1.0f,
                       float axesRadius = 0.05f,
                       bool drawArrowheads = true);
    
    static void drawAxes(const glm::vec3& position, 
                       const glm::quat& orientation,
                       float axesLength = 1.0f,
                       float axesRadius = 0.05f,
                       bool drawArrowheads = true);
    
    // Draw ROS standard axes (newly added)
    static void drawRosAxes(const glm::vec3& position, 
                              const glm::mat4& rotationMatrix,
                              float axesLength = 1.0f,
                              float axesRadius = 0.05f,
                              bool drawArrowheads = true);
        
    static void drawRosAxes(const glm::vec3& position, 
                              const glm::quat& orientation,
                              float axesLength = 1.0f,
                              float axesRadius = 0.05f,
                              bool drawArrowheads = true);
    
    // Draw single axis (with cone arrowhead)
    static void drawSingleAxis(const glm::vec3& start,
                             const glm::vec3& direction,
                             float length,
                             float radius,
                             const glm::vec4& color,
                             bool drawArrowhead = true);
    
    // 구체 (Sphere)
    static void drawSphere(const glm::vec3& center,
                          float radius,
                          const glm::vec4& color,
                          int segments = 16);
    
    // 정육면체 (Cube)
    static void drawCube(const glm::vec3& center,
                        float size,
                        const glm::vec4& color);
    
    // 직육면체 (Box)
    static void drawBox(const glm::vec3& center,
                       const glm::vec3& size,
                       const glm::vec4& color);
    
    // 와이어프레임 구체
    static void drawWireSphere(const glm::vec3& center,
                              float radius,
                              const glm::vec4& color,
                              int segments = 16);
    
    // 와이어프레임 큐브
    static void drawWireCube(const glm::vec3& center,
                            float size,
                            const glm::vec4& color);
};

} // namespace ShapeHelper

#endif // SHAPE_HELPER_HPP