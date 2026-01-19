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

void run_simulation(
    Simulation& simulation,
    const Forces& forces,
    const std::vector<float>& frames)
{
    for (float frame : frames) {
        simulation.advance(forces, frame);
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

    constexpr Forces forces{
        .acceleration_x = 0,
        .acceleration_y = -9.8f, // Due to gravity
        .drag = 0.1f,
    };

    std::vector<float> frames_a{ 0.06f, 0.11f, 0.13f, 0.05f, 0.15f };
    std::vector<float> frames_b{ 0.1f, 0.1f, 0.1f, 0.1f, 0.1f };

    Simulation simulation_a(initial_pos_x, initial_pos_y, initial_vel_x, initial_vel_y);
    Simulation simulation_b(initial_pos_x, initial_pos_y, initial_vel_x, initial_vel_y);

    run_simulation(simulation_a, forces, frames_a);
    run_simulation(simulation_b, forces, frames_b);

    const SimulationState& result_a = simulation_a.get_state();
    const SimulationState& result_b = simulation_b.get_state();

    std::cout << result_a;

    static_assert(std::is_trivially_copyable_v<SimulationState>);
    std::vector<std::byte> bytes_a = serialize_simulation_state(result_a);
    std::vector<std::byte> bytes_b = serialize_simulation_state(result_b);

    assert(bytes_a == bytes_b);
    assert(simulation_a.get_step_count() == simulation_b.get_step_count());

    std::cout << "Determinism check passed." << "\n";
}