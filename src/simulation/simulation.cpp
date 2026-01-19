#include "forces.h"
#include <algorithm>
#include <cassert>
#include "simulation_state.h"
#include "simulation.h"

// Note: position scalars are in units of meters, time is in unit of seconds
constexpr float fixed_dt = 0.01f;
constexpr float max_frame_dt = 0.25f;

Simulation::Simulation(float pos_x, float pos_y, float velocity_x, float velocity_y) noexcept
    : m_curr_state{
        .pos_x = pos_x,
        .pos_y = pos_y,
        .velocity_x = velocity_x,
        .velocity_y = velocity_y,
    },
    m_prev_state{
        .pos_x = pos_x,
        .pos_y = pos_y,
        .velocity_x = velocity_x,
        .velocity_y = velocity_y,
    }
{};

void Simulation::advance(const Forces& forces, float frame_dt) noexcept {
    
    // Clamp frame dt to prevent one hitch from triggering thousands of updates
    // Note: clamping trades real-time equivalence for stability
    frame_dt = std::min(frame_dt, max_frame_dt);
    
    m_accumulator_seconds += frame_dt;
    while (m_accumulator_seconds >= fixed_dt) {
        m_prev_state = m_curr_state;
        semi_explicit_euler_update(forces, fixed_dt);
        m_accumulator_seconds -= fixed_dt;
        m_step_count++;
    }

    assert(m_accumulator_seconds >= 0.0f);
    assert(m_accumulator_seconds < fixed_dt);
};

const SimulationState& Simulation::get_curr_state() const noexcept {
    return m_curr_state;
}

const SimulationState& Simulation::get_prev_state() const noexcept {
    return m_prev_state;
}

int Simulation::get_step_count() const noexcept {
    return m_step_count;
}

float Simulation::get_alpha() const noexcept {
    return m_accumulator_seconds / fixed_dt;
}

// State is updated using semi-explicit Euler integration, with velocity first and then position, for numerical stability.
// Consistent ordering is required to preserve deterministic behavior.
void Simulation::semi_explicit_euler_update(const Forces& forces, float dt) noexcept
{
    float dampingFactor = std::max(0.0f, 1.0f - dt * forces.drag); // Linear damping approximation (deterministic)
    m_curr_state.velocity_x += dt * forces.acceleration_x;
    m_curr_state.velocity_x *= dampingFactor;
    m_curr_state.velocity_y += dt * forces.acceleration_y;
    m_curr_state.velocity_y *= dampingFactor;
    m_curr_state.pos_x += dt * m_curr_state.velocity_x;
    m_curr_state.pos_y += dt * m_curr_state.velocity_y;
}