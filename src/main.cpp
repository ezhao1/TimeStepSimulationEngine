#include <iostream>
#include <vector>
#include <cstddef>
#include <cassert>
#include <cstring>
#include <type_traits>
#include <algorithm>

// Note: position scalars are in units of meters, time is in unit of seconds
constexpr float fixed_dt = 0.01f;

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

class Simulation {
    public:
        Simulation(float pos_x, float pos_y, float velocity_x, float velocity_y)
            : m_state{
                .pos_x = pos_x,
                .pos_y = pos_y,
                .velocity_x = velocity_x,
                .velocity_y = velocity_y,
            }
        {};

        void advance(const Forces& forces, float frame_dt) noexcept {
            float target = m_accumulator + frame_dt;
            while (m_accumulator < target) {
                semi_explicit_euler_update(forces, fixed_dt);
                m_accumulator += fixed_dt;
            }
        };

        const SimulationState& get_state() {
            return m_state;
        }
    private:
        // State is updated using semi-explicit Euler integration, with velocity first and then position, for numerical stability.
        // Consistent ordering is required to preserve deterministic behavior.
        void semi_explicit_euler_update(const Forces& forces, float dt) noexcept
        {
            float dampingFactor = std::max(0.0f, 1.0f - dt * forces.drag); // Linear damping approximation (deterministic)
            m_state.velocity_x += dt * forces.acceleration_x;
            m_state.velocity_x *= dampingFactor;
            m_state.velocity_y += dt * forces.acceleration_y;
            m_state.velocity_y *= dampingFactor;
            m_state.pos_x += dt * m_state.velocity_x;
            m_state.pos_y += dt * m_state.velocity_y;
        }

        SimulationState m_state;
        float m_accumulator = 0;
};

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

    std::vector<float> frames_a{ 6, 11, 13 };
    std::vector<float> frames_b{ 10, 10, 10 };

    Simulation simulation1(initial_pos_x, initial_pos_y, initial_vel_x, initial_vel_y);
    Simulation simulation2(initial_pos_x, initial_pos_y, initial_vel_x, initial_vel_y);

    run_simulation(simulation1, forces, frames_a);
    run_simulation(simulation2, forces, frames_b);

    const SimulationState& result_a = simulation1.get_state();
    const SimulationState& result_b = simulation2.get_state();

    std::cout << result_a;

    static_assert(std::is_trivially_copyable_v<SimulationState>);
    std::vector<std::byte> bytes_a = serialize_simulation_state(result_a);
    std::vector<std::byte> bytes_b = serialize_simulation_state(result_b);

    assert(bytes_a == bytes_b);

    std::cout << "Determinism check passed." << "\n";
}