#include <iostream>
#include <vector>
#include <cstddef>
#include <cassert>
#include <cstring>
#include <type_traits>
#include <algorithm>
#include "simulation/simulation_state.h"
#include "simulation/simulation.h"
#include "simulation/forces.h"

std::ostream& operator<<(std::ostream& os, const SimulationState& sim_state) {
    os << "Position: " << sim_state.pos_x << ", " << sim_state.pos_y << "\n";
    os << "Velocity: " << sim_state.velocity_x << ", " << sim_state.velocity_y << "\n";
    return os;
}

SimulationState lerp(const SimulationState& a, const SimulationState& b, float alpha) {
    float d_pos_x = b.pos_x - a.pos_x;
    float d_pos_y = b.pos_y - a.pos_y;
    float d_vel_x = b.velocity_x - a.velocity_x;
    float d_vel_y = b.velocity_y - a.velocity_y;
    return SimulationState{
        .pos_x = a.pos_x + (alpha * d_pos_x),
        .pos_y = a.pos_y + (alpha * d_pos_y),
        .velocity_x = a.velocity_x + (alpha * d_vel_x),
        .velocity_y = a.velocity_y + (alpha * d_vel_y),
    };
}

void run_simulation(
    Simulation& simulation,
    const std::vector<float>& frames)
{
    std::cout << simulation.get_curr_state();

    for (float frame : frames) {
        simulation.advance(frame);
        
        const SimulationState& prev_state = simulation.get_prev_state();
        const SimulationState& curr_state = simulation.get_curr_state();
        const float alpha = simulation.get_alpha();

        std::cout << lerp(prev_state, curr_state, alpha);
    }
}

std::vector<std::byte> serialize_simulation_state(const SimulationState& state) {
    std::vector<std::byte> bytes(sizeof(SimulationState));
    memcpy(bytes.data(), &state, sizeof(SimulationState));
    return bytes;
}

int main()
{
    constexpr float initial_pos_x = 0;
    constexpr float initial_pos_y = 0;
    constexpr float initial_vel_x = 50;
    constexpr float initial_vel_y = 100;
    constexpr float acceleration_x = 0.0f;
    constexpr float acceleration_y = -9.8f; // Due to gravity
    constexpr float drag = 0.1f;
    constexpr float fixed_dt = 0.01f;

    Model model(
        acceleration_x,
        acceleration_y,
        drag
    );

    std::vector<float> frames_a{ 0.06f, 0.11f, 0.13f, 0.05f, 0.15f };
    std::vector<float> frames_b{ 0.1f, 0.1f, 0.1f, 0.1f, 0.1f };
    std::vector<Model> models{model};

    Simulation simulation_a(initial_pos_x, initial_pos_y, initial_vel_x, initial_vel_y, fixed_dt, models);
    Simulation simulation_b(initial_pos_x, initial_pos_y, initial_vel_x, initial_vel_y, fixed_dt, models);

    run_simulation(simulation_a, frames_a);
    run_simulation(simulation_b, frames_b);

    const SimulationState& result_a = simulation_a.get_curr_state();
    const SimulationState& result_b = simulation_b.get_curr_state();

    std::cout << result_a;

    static_assert(std::is_trivially_copyable_v<SimulationState>);
    std::vector<std::byte> bytes_a = serialize_simulation_state(result_a);
    std::vector<std::byte> bytes_b = serialize_simulation_state(result_b);

    assert(bytes_a == bytes_b);
    assert(simulation_a.get_step_count() == simulation_b.get_step_count());

    std::cout << "Determinism check passed." << "\n";
}