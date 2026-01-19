#include "forces.h"
#include <algorithm>
#include <cassert>
#include "simulation_state.h"
#include "simulation.h"
#include "model.h"
#include <vector>
#include <utility>
#include "accumulated_forces.h"

// Note: position scalars are in units of meters, time is in unit of seconds
constexpr float max_frame_dt = 0.25f;

Simulation::Simulation(
    float pos_x,
    float pos_y,
    float velocity_x,
    float velocity_y,
    float fixed_dt,
    std::vector<Model> models) noexcept
    : m_curr_state{
        .position = { .x = pos_x, .y = pos_y },
        .velocity = { .x = velocity_x, .y = velocity_y },
    },
    m_prev_state{
        .position = { .x = pos_x, .y = pos_y },
        .velocity = { .x = velocity_x, .y = velocity_y },
    },
    m_fixed_dt(fixed_dt),
    m_models(std::move(models))
{}

void Simulation::advance(float frame_dt) noexcept {
    // Clamp frame dt to prevent one hitch from triggering thousands of updates
    // Note: clamping trades real-time equivalence for stability
    frame_dt = std::min(frame_dt, max_frame_dt);
    
    m_accumulator_seconds += frame_dt;
    while (m_accumulator_seconds >= m_fixed_dt) {
        m_prev_state = m_curr_state;

        AccumulatedForces accumulated_forces{
            .acceleration = { .x = 0, .y = 0 },
            .drag = 0,
        };

        for (const Model& model : m_models) {
            model.contribute(accumulated_forces);
        }

        // State is updated using semi-explicit Euler integration, with velocity first and then position, for numerical stability.
        // Consistent ordering is required to preserve deterministic behavior.
        // Drag: dv/dt = -kv
        // Solved: v(t)=v0⋅e−kt
        float dampingFactor = std::exp(-accumulated_forces.drag * m_fixed_dt);
        m_curr_state.velocity.x += m_fixed_dt * accumulated_forces.acceleration.x;
        m_curr_state.velocity.x *= dampingFactor;
        m_curr_state.velocity.y += m_fixed_dt * accumulated_forces.acceleration.y;
        m_curr_state.velocity.y *= dampingFactor;
        m_curr_state.position.x += m_fixed_dt * m_curr_state.velocity.x;
        m_curr_state.position.y += m_fixed_dt * m_curr_state.velocity.y;
        m_accumulator_seconds -= m_fixed_dt;
        m_step_count++;
    }

    assert(m_accumulator_seconds >= 0.0f);
    assert(m_accumulator_seconds < m_fixed_dt);
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
    return m_accumulator_seconds / m_fixed_dt;
}