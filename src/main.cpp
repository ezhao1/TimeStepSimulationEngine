#include <iostream>
#include <vector>
#include <cstddef>
#include <cassert>
#include <cstring>
#include <type_traits>

struct SimulationState {
    float pos_x;
    float pos_y;
    float velocity_x;
    float velocity_y;
    float acceleration_x;
    float acceleration_y;
};

std::ostream& operator<<(std::ostream& os, const SimulationState& simState) {
    os << "Position: " << simState.pos_x << ", " << simState.pos_y << "\n";
    os << "Velocity: " << simState.velocity_x << ", " << simState.velocity_y << "\n";
    os << "Acceleration: " << simState.acceleration_x << ", " << simState.acceleration_y << "\n";
    return os;
}

// State is updated using semi-explicit Euler integration, with velocity first and then position, for numerical stability.
// Consistent ordering is required to preserve deterministic behavior.
void semi_explicit_euler_update(SimulationState& state, float dt) {
    state.velocity_x += dt * state.acceleration_x;
    state.velocity_y += dt * state.acceleration_y;
    state.pos_x += dt * state.velocity_x;
    state.pos_y += dt * state.velocity_y;
}

template<typename STATE>
STATE run_simulation(
    STATE initial,
    void(*update_func)(STATE&, float),
    float dt,
    int steps)
{
    STATE state = initial; // Passed in by value

    for (int i = 0; i < steps; i++) {
        update_func(state, dt);
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
    constexpr int steps = 100;
    constexpr float dt = 0.01f;

    SimulationState initialState{
        .pos_x = 0,
        .pos_y = 0,
        .velocity_x = 5,
        .velocity_y = 10,
        .acceleration_x = 1,
        .acceleration_y = 2,
    };

    SimulationState result_a = run_simulation(initialState, &semi_explicit_euler_update, dt, steps);
    SimulationState result_b = run_simulation(initialState, &semi_explicit_euler_update, dt, steps);

    std::cout << result_a;

    static_assert(std::is_trivially_copyable_v<SimulationState>);
    std::vector<std::byte> bytes_a = serialize_simulation_state(result_a);
    std::vector<std::byte> bytes_b = serialize_simulation_state(result_b);

    assert(bytes_a == bytes_b);

    std::cout << "Determinism check passed." << "\n";
}