// shape_helper.hpp - 간단하고 실용적
#ifndef SHAPE_HELPER_HPP
#define SHAPE_HELPER_HPP

#include <glm/glm.hpp>
#include <GL/gl.h>
#include <cmath>

namespace ShapeHelper {

class SimpleShape {
public:
    // 실린더 (축, 인디케이터용)
    static void drawCylinder(const glm::vec3& position, 
                           const glm::vec3& direction, 
                           float length, 
                           float radius, 
                           const glm::vec4& color,
                           int segments = 12);
                           
    // 원판 (바닥, 상단용)
    static void drawCircle(const glm::vec3& center,
                          const glm::vec3& normal,
                          float radius,
                          const glm::vec4& color,
                          int segments = 12);
                          
    // 구체 (미래 확장용)
    static void drawSphere(const glm::vec3& position,
                          float radius,
                          const glm::vec3& color,
                          int segments = 8);
};

} // namespace Widget

#endif