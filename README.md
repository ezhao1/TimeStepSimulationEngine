# Summary

Deterministic time step physics simulation engine that allows simulating the position + velocity of an object across time steps.

# What guarantees determinism?

Determinism is guaranteed by a check that runs the simulation with the same inputs twice, compares the output state bytes, and asserts they are equal between runs.

# Who owns the state?

The state is owned by 
