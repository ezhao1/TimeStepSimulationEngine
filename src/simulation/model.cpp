#include "model.h"
#include <algorithm>
#include "force_accumulator.h"

Model::Model(
    float acceleration_x,
    float acceleration_y,
    float drag
) noexcept:
m_forces{
    .acceleration = {
        .x = acceleration_x,
        .y = acceleration_y,
    },
    .drag = drag,
} {}

void Model::contribute(ForceAccumulator &forces) const noexcept {
    forces.acceleration.x += m_forces.acceleration.x;
    forces.acceleration.y += m_forces.acceleration.y;
    forces.drag += m_forces.drag; // Additive drag
};