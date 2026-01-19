#include "model.h"
#include <algorithm>

Model::Model(
    float acceleration_x,
    float acceleration_y,
    float drag
) noexcept:
m_forces{
    .acceleration_x = acceleration_x,
    .acceleration_y = acceleration_y,
    .drag = drag
} {}

void Model::contribute(Forces &forces, float dt) const noexcept {
    forces.acceleration_x += m_forces.acceleration_x;
    forces.acceleration_y += m_forces.acceleration_y;
    forces.drag += m_forces.drag;
};