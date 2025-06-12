// shape_helper.hpp - 간단하고 실용적
#ifndef SHAPE_HELPER_HPP
#define SHAPE_HELPER_HPP

#include <GL/gl.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/quaternion.hpp>  // ✅ 이 헤더 추가 (mat4_cast 함수 포함)
#include <cmath>

namespace ShapeHelper {

class SimpleShape {
public:
    // 기본 도형들
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
    
    // 축 그리기 (원뿔 화살표 사용)
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
    
    // ROS 표준 축 그리기 (새로 추가)
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
    
    // 개별 축 그리기 (원뿔 화살표 포함)
    static void drawSingleAxis(const glm::vec3& start,
                             const glm::vec3& direction,
                             float length,
                             float radius,
                             const glm::vec4& color,
                             bool drawArrowhead = true);
};

} // namespace Widget

#endif