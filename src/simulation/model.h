#pragma once
#include "forces.h"
#include "force_accumulator.h"
#include "simulation_state.h"

class Model {
    public:
        Model(
            float acceleration_x,
            float acceleration_y,
            float drag
        ) noexcept;

        void contribute(ForceAccumulator &forces) const noexcept;
    private:
        const Forces m_forces;
};