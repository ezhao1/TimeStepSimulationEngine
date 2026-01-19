#pragma once
#include "forces.h"
#include "simulation_state.h"
#include "model.h"
#include <vector>

class Simulation {
    public:
        Simulation(
            float pos_x,
            float pos_y,
            float velocity_x,
            float velocity_y,
            std::vector<const Model*> models
        ) noexcept;
        void advance(float frame_dt) noexcept;
        const SimulationState& get_curr_state() const noexcept;
        const SimulationState& get_prev_state() const noexcept;
        int get_step_count() const noexcept;
        float get_alpha() const noexcept;
    private:
        void semi_explicit_euler_update(const Forces& forces, float dt) noexcept;
        SimulationState m_prev_state;
        SimulationState m_curr_state;
        const std::vector<const Model*> m_models;
        float m_accumulator_seconds = 0.0f;
        int m_step_count = 0;
};