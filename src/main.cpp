#include <iostream>
#include <vector>
#include <cstddef>
#include <cassert>
#include <cstring>
#include <type_traits>

// Note: position scalars are in units of meters, time is in unit of seconds
struct SimulationState {
    float pos_x;
    float pos_y;
    float velocity_x;
    float velocity_y;
};

std::ostream& operator<<(std::ostream& os, const SimulationState& sim_state) {
    os << "Position: " << sim_state.pos_x << ", " << sim_state.pos_y << "\n";
    os << "Velocity: " << sim_state.velocity_x << ", " << sim_state.velocity_y << "\n";
    return os;
}

// State is updated using semi-explicit Euler integration, with velocity first and then position, for numerical stability.
// Consistent ordering is required to preserve deterministic behavior.
void semi_explicit_euler_with_drag_update(
    SimulationState& state,
    float acceleration_x,
    float acceleration_y,
    float drag,
    float dt)
{
    state.velocity_x += dt * acceleration_x;
    state.velocity_x *= 1 - dt * drag; // drag is opposite the direction of velocity
    state.velocity_y += dt * acceleration_y;
    state.velocity_y *= 1 - dt * drag;
    state.pos_x += dt * state.velocity_x;
    state.pos_y += dt * state.velocity_y;
}

template<typename STATE>
STATE run_simulation(
    STATE state, // passed by value
    void(*update_func)(STATE&, float, float, float, float),
    int steps,
    float acceleration_x, // For now, assume constant acceleration
    float acceleration_y,
    float drag,
    float dt)
{
    for (int i = 0; i < steps; i++) {
        update_func(state, acceleration_x, acceleration_y, drag, dt);
    }

    return state;
}

std::vector<std::byte> serialize_simulation_state(const SimulationState& state) {
    std::vector<std::byte> bytes(sizeof(SimulationState));
    memcpy(bytes.data(), &state, sizeof(SimulationState));
    return bytes;
}

int main()
{
    constexpr int steps = 1000;
    constexpr float dt = 0.01f;
    constexpr float drag = 0.1f;
    constexpr float outside_force_acceleration_x = 0;
    constexpr float outside_force_acceleration_y = -9.8f; // Due to gravity

    SimulationState initialState{
        .pos_x = 0,
        .pos_y = 0,
        .velocity_x = 50,
        .velocity_y = 100,
    };

    SimulationState result_a = run_simulation(initialState, &semi_explicit_euler_with_drag_update, steps, outside_force_acceleration_x, outside_force_acceleration_y, drag, dt);
    SimulationState result_b = run_simulation(initialState, &semi_explicit_euler_with_drag_update, steps, outside_force_acceleration_x, outside_force_acceleration_y, drag, dt);

    std::cout << result_a;

    static_assert(std::is_trivially_copyable_v<SimulationState>);
    std::vector<std::byte> bytes_a = serialize_simulation_state(result_a);
    std::vector<std::byte> bytes_b = serialize_simulation_state(result_b);

    assert(bytes_a == bytes_b);

    std::cout << "Determinism check passed." << "\n";
}