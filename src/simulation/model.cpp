#include "model.h"
#include <algorithm>
#include "force_accumulator.h"

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

void Model::contribute(ForceAccumulator &forces) const noexcept {
    forces.acceleration_x += m_forces.acceleration_x;
    forces.acceleration_y += m_forces.acceleration_y;
    forces.damping_factor *= (1 - m_forces.drag);
};