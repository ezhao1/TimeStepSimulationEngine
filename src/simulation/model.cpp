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

void Model::apply(SimulationState &state, float dt) const noexcept {
    // State is updated using semi-explicit Euler integration, with velocity first and then position, for numerical stability.
    // Consistent ordering is required to preserve deterministic behavior.
    float dampingFactor = std::max(0.0f, 1.0f - dt * m_forces.drag); // Linear damping approximation (deterministic)
    state.velocity_x += dt * m_forces.acceleration_x;
    state.velocity_x *= dampingFactor;
    state.velocity_y += dt * m_forces.acceleration_y;
    state.velocity_y *= dampingFactor;
    state.pos_x += dt * state.velocity_x;
    state.pos_y += dt * state.velocity_y;
};