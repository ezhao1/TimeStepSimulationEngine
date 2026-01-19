#include "forces.h"
#include <algorithm>
#include <cassert>
#include "simulation_state.h"
#include "simulation.h"
#include "model.h"
#include <vector>
#include <utility>

// Note: position scalars are in units of meters, time is in unit of seconds
constexpr float fixed_dt = 0.01f;
constexpr float max_frame_dt = 0.25f;

Simulation::Simulation(
    float pos_x,
    float pos_y,
    float velocity_x,
    float velocity_y,
    std::vector<const Model*> models) noexcept
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
    },
    m_models(std::move(models))
{}

void Simulation::advance(float frame_dt) noexcept {
    // Clamp frame dt to prevent one hitch from triggering thousands of updates
    // Note: clamping trades real-time equivalence for stability
    frame_dt = std::min(frame_dt, max_frame_dt);
    
    m_accumulator_seconds += frame_dt;
    while (m_accumulator_seconds >= fixed_dt) {
        m_prev_state = m_curr_state;
        // TODO: think about order here - should we apply all the accelerations first together, and then drags together?
        for (const Model* model : m_models) {
            model->apply(m_curr_state, fixed_dt);
        }
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