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

struct Forces {
    float acceleration_x;
    float acceleration_y;
    float drag;
};

std::ostream& operator<<(std::ostream& os, const SimulationState& sim_state) {
    os << "Position: " << sim_state.pos_x << ", " << sim_state.pos_y << "\n";
    os << "Velocity: " << sim_state.velocity_x << ", " << sim_state.velocity_y << "\n";
    return os;
}

// State is updated using semi-explicit Euler integration, with velocity first and then position, for numerical stability.
// Consistent ordering is required to preserve deterministic behavior.
void semi_explicit_euler_update(
    SimulationState& state,
    const Forces& forces,
    float dt)
{
    state.velocity_x += dt * forces.acceleration_x;
    state.velocity_x *= 1 - dt * forces.drag; // drag is opposite the direction of velocity
    state.velocity_y += dt * forces.acceleration_y;
    state.velocity_y *= 1 - dt * forces.drag;
    state.pos_x += dt * state.velocity_x;
    state.pos_y += dt * state.velocity_y;
}

template<typename STATE>
STATE run_simulation(
    STATE state, // passed by value
    void(*update_func)(STATE&, const Forces&, float),
    const Forces& forces,
    int steps,
    float dt)
{
    for (int i = 0; i < steps; i++) {
        update_func(state, forces, dt);
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

    constexpr SimulationState initialState{
        .pos_x = 0,
        .pos_y = 0,
        .velocity_x = 50,
        .velocity_y = 100,
    };

    constexpr Forces forces{
        .acceleration_x = 0,
        .acceleration_y = -9.8f, // Due to gravity
        .drag = 0.1f,
    };

    SimulationState result_a = run_simulation(initialState, &semi_explicit_euler_update, forces, steps, dt);
    SimulationState result_b = run_simulation(initialState, &semi_explicit_euler_update, forces, steps, dt);

    std::cout << result_a;

    static_assert(std::is_trivially_copyable_v<SimulationState>);
    std::vector<std::byte> bytes_a = serialize_simulation_state(result_a);
    std::vector<std::byte> bytes_b = serialize_simulation_state(result_b);

    assert(bytes_a == bytes_b);

    std::cout << "Determinism check passed." << "\n";
}