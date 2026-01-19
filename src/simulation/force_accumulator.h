#pragma once
#include "vec2.h"

struct ForceAccumulator {
    Vec2 acceleration;
    float drag;
};