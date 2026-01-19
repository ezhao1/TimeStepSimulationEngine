#pragma once
#include "forces.h"
#include "simulation_state.h"

class Model {
    public:
        Model(
            float acceleration_x,
            float acceleration_y,
            float drag
        ) noexcept;

        void contribute(Forces &forces, float dt) const noexcept;
    private:
        const Forces m_forces;
};