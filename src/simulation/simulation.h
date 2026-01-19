#pragma once
#include "forces.h"
#include "simulation_state.h"

class Simulation {
    public:
        Simulation(float pos_x, float pos_y, float velocity_x, float velocity_y) noexcept;
        void advance(const Forces& forces, float frame_dt) noexcept;
        const SimulationState& get_state() const noexcept;
        const int get_step_count() const noexcept;
    private:
        void semi_explicit_euler_update(const Forces& forces, float dt) noexcept;
        SimulationState m_state;
        float m_accumulator_seconds = 0.0f;
        int m_step_count = 0;
};