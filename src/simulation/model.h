#pragma once
#include "forces.h"
#include "accumulated_forces.h"
#include "simulation_state.h"

class Model {
    public:
        Model(
            float acceleration_x,
            float acceleration_y,
            float drag
        ) noexcept;

        void contribute(AccumulatedForces &forces) const noexcept;
    private:
        const Forces m_forces;
};